#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <ctime>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

static bool ensureDir(const std::string& dir){
  struct stat st;
  if (stat(dir.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
  return mkdir(dir.c_str(), 0755) == 0;
}

static std::string nowStamp(){
  std::time_t t = std::time(nullptr);
  std::tm tm;
  localtime_r(&t, &tm);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
                tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
  return std::string(buf);
}

enum class State { WAIT_HAND, WAIT_SMILE, COOLDOWN };
enum class PatrolPhase { TURN, PAUSE };

// Auto adjust while WAIT_SMILE
enum class AdjustPhase { IDLE, MOVING, SETTLING };
enum class AdjustAction { NONE, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT };

static const char* toStr(AdjustPhase p){
  switch(p){
    case AdjustPhase::IDLE: return "IDLE";
    case AdjustPhase::MOVING: return "MOVING";
    case AdjustPhase::SETTLING: return "SETTLING";
  }
  return "UNK";
}
static const char* toStr(AdjustAction a){
  switch(a){
    case AdjustAction::NONE: return "NONE";
    case AdjustAction::FORWARD: return "FORWARD";
    case AdjustAction::BACKWARD: return "BACKWARD";
    case AdjustAction::TURN_LEFT: return "TURN_LEFT";
    case AdjustAction::TURN_RIGHT: return "TURN_RIGHT";
  }
  return "UNK";
}

int main(int argc, char** argv){
  ros::init(argc, argv, "smile_photo_node");
  ros::NodeHandle nh("~");

  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  auto stopCmd = [&](){
    geometry_msgs::Twist z;
    z.linear.x = 0.0; z.angular.z = 0.0;
    return z;
  };

  // ---------------- Params: camera ----------------
  std::string cam_dev; nh.param<std::string>("camera_device", cam_dev, "/dev/video0");
  int width; nh.param("width", width, 1280);
  int height; nh.param("height", height, 720);
  int fps; nh.param("fps", fps, 30);
  bool rotate180; nh.param("rotate180", rotate180, true);

  int fourcc = cv::VideoWriter::fourcc('M','J','P','G');

  // ---------------- Params: cascades & save ----------------
  std::string face_xml, smile_xml, save_dir;
  nh.param<std::string>("face_cascade",  face_xml,  "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml");
  nh.param<std::string>("smile_cascade", smile_xml, "/usr/share/opencv/haarcascades/haarcascade_smile.xml");
  nh.param<std::string>("save_dir", save_dir, "/home/group11/photos");

  // ---------------- Params: patrol scan ----------------
  bool patrol_enable; nh.param("patrol_enable", patrol_enable, true);
  double turn_speed; nh.param("turn_speed", turn_speed, 0.45);
  double turn_duration; nh.param("turn_duration", turn_duration, 0.25);
  double pause_duration; nh.param("pause_duration", pause_duration, 0.90);
  int sweep_steps; nh.param("sweep_steps", sweep_steps, 8);
  bool patrol_only_when_wait_hand; nh.param("patrol_only_when_wait_hand", patrol_only_when_wait_hand, true);

  // 每次剛停下來先穩定多久才做 hand detect（秒）
  double pause_settle_sec; nh.param("pause_settle_sec", pause_settle_sec, 0.50);

  // ---------------- Params: hand gate (skin+motion+geometry) ----------------
  bool require_hand; nh.param("require_hand", require_hand, true);
  int hand_frames_required; nh.param("hand_frames_required", hand_frames_required, 4);

  double hand_area_min_ratio; nh.param("hand_area_min_ratio", hand_area_min_ratio, 0.0065);
  double hand_area_max_ratio; nh.param("hand_area_max_ratio", hand_area_max_ratio, 0.20);
  double hand_center_y_max_ratio; nh.param("hand_center_y_max_ratio", hand_center_y_max_ratio, 0.85);

  int motion_thresh; nh.param("motion_thresh", motion_thresh, 18);
  int motion_min_pixels; nh.param("motion_min_pixels", motion_min_pixels, 1200);

  // 全畫面 motion 上限（比例），超過代表是機器在動，不是手在動
  double motion_global_max_ratio; nh.param("motion_global_max_ratio", motion_global_max_ratio, 0.25);

  // ---------------- Params: smile detection ----------------
  int smile_frames_required; nh.param("smile_frames_required", smile_frames_required, 8);
  double smile_scale; nh.param("smile_scale", smile_scale, 1.15);
  int smile_neighbors; nh.param("smile_neighbors", smile_neighbors, 25);

  double face_area_min_ratio; nh.param("face_area_min_ratio", face_area_min_ratio, 0.02);
  double face_area_max_ratio; nh.param("face_area_max_ratio", face_area_max_ratio, 0.35);
  double face_center_margin;  nh.param("face_center_margin",  face_center_margin, 0.18);
  int cooldown_sec; nh.param("cooldown_sec", cooldown_sec, 2);

  double log_hz; nh.param("log_hz", log_hz, 1.0);

  // ---------------- Params: auto adjust (WAIT_SMILE) ----------------
  bool auto_adjust_enable; nh.param("auto_adjust_enable", auto_adjust_enable, true);

  // 前進微調：走一小段 -> 停 -> 等一下 -> 再判斷
  double approach_speed; nh.param("approach_speed", approach_speed, 0.07);          // m/s
  double approach_step_sec; nh.param("approach_step_sec", approach_step_sec, 0.25); // s
  double approach_settle_sec; nh.param("approach_settle_sec", approach_settle_sec, 0.45); // s

  int no_person_frames_to_step; nh.param("no_person_frames_to_step", no_person_frames_to_step, 18);
  int framed_bad_frames_to_step; nh.param("framed_bad_frames_to_step", framed_bad_frames_to_step, 12);
  int approach_max_steps; nh.param("approach_max_steps", approach_max_steps, 20);

  // 可選：對準畫面中心（預設關閉，避免你不想轉）
  bool align_enable; nh.param("align_enable", align_enable, false);
  double align_ang_speed; nh.param("align_ang_speed", align_ang_speed, 0.25);     // rad/s
  double align_step_sec; nh.param("align_step_sec", align_step_sec, 0.18);        // s

  // 可選：太近就後退（預設關閉）
  bool retreat_enable; nh.param("retreat_enable", retreat_enable, false);
  double retreat_speed; nh.param("retreat_speed", retreat_speed, 0.06);           // m/s (正值)

  // 失敗保護：太久都找不到人臉就回 WAIT_HAND（避免一路慢慢撞上去）
  bool abort_on_fail; nh.param("abort_on_fail", abort_on_fail, true);
  int abort_no_person_frames; nh.param("abort_no_person_frames", abort_no_person_frames, 300); // ~10s @30Hz

  // ---------------- OpenCV init ----------------
  cv::CascadeClassifier face_cascade, smile_cascade;
  if (!face_cascade.load(face_xml)){
    ROS_ERROR("Failed to load face cascade: %s", face_xml.c_str());
    return 1;
  }
  if (!smile_cascade.load(smile_xml)){
    ROS_ERROR("Failed to load smile cascade: %s", smile_xml.c_str());
    return 1;
  }
  if (!ensureDir(save_dir)){
    ROS_ERROR("Failed to create save dir: %s", save_dir.c_str());
    return 1;
  }

  cv::VideoCapture cap(cam_dev, cv::CAP_V4L2);
  if (!cap.isOpened()){
    ROS_ERROR("Failed to open camera: %s", cam_dev.c_str());
    return 1;
  }
  cap.set(cv::CAP_PROP_FOURCC, fourcc);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap.set(cv::CAP_PROP_FPS, fps);

  ROS_INFO("Camera opened: %s (%dx%d@%d) rotate180=%d", cam_dev.c_str(), width, height, fps, (int)rotate180);
  ROS_INFO("Cascade face:  %s", face_xml.c_str());
  ROS_INFO("Cascade smile: %s", smile_xml.c_str());
  ROS_INFO("Save dir:      %s", save_dir.c_str());

  ROS_INFO("Hand gate: require=%d frames=%d area_min=%.4f area_max=%.2f y_max=%.2f motion_thresh=%d motion_min_pixels=%d motion_global_max_ratio=%.2f pause_settle=%.2fs",
           (int)require_hand, hand_frames_required,
           hand_area_min_ratio, hand_area_max_ratio, hand_center_y_max_ratio,
           motion_thresh, motion_min_pixels, motion_global_max_ratio, pause_settle_sec);

  ROS_INFO("Patrol: enable=%d turn_speed=%.2f turn=%.2fs pause=%.2fs sweep_steps=%d",
           (int)patrol_enable, turn_speed, turn_duration, pause_duration, sweep_steps);

  ROS_INFO("AutoAdjust: enable=%d approach=%.2fm/s x %.2fs settle=%.2fs no_person_frames=%d framed_bad_frames=%d max_steps=%d align=%d retreat=%d abort=%d abort_no_person_frames=%d",
           (int)auto_adjust_enable, approach_speed, approach_step_sec, approach_settle_sec,
           no_person_frames_to_step, framed_bad_frames_to_step, approach_max_steps,
           (int)align_enable, (int)retreat_enable, (int)abort_on_fail, abort_no_person_frames);

  // ---------------- State machine vars ----------------
  State state = State::WAIT_HAND;
  PatrolPhase phase = PatrolPhase::PAUSE;
  ros::Time phase_deadline = ros::Time::now() + ros::Duration(pause_duration);
  ros::Time pause_started = ros::Time::now();  // 記住 PAUSE 開始時間（用來 settle）

  int sweep_count = 0;
  int turn_dir = +1;

  int hand_streak = 0;
  int smile_streak = 0;

  ros::Time cooldown_until(0);

  // motion baseline (for hand)
  cv::Mat prev_gray_blur;
  bool have_prev = false;

  // Auto adjust vars (WAIT_SMILE)
  AdjustPhase adj_phase = AdjustPhase::IDLE;
  AdjustAction adj_action = AdjustAction::NONE;
  ros::Time adj_deadline(0);
  ros::Time adj_settle_until(0);
  int adjust_steps = 0;

  int no_person_count = 0;
  int framed_bad_count = 0;

  // For deciding why FRAMED=0
  double fa_ratio = 0.0;
  double cxr = 0.5, cyr = 0.5;
  bool area_ok = false, center_ok = false;

  // hand debug
  int best_area = 0;
  double area_ratio = 0.0;
  double cy_ratio = 1.0;
  bool hand_now = false;
  int motion_pixels = 0;
  bool motion_ok = false;

  ros::Rate loop(30);
  ros::Time last_log(0);

  while (ros::ok()){
    ros::Time now = ros::Time::now();

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()){
      ROS_WARN_THROTTLE(2.0, "Camera frame empty");
      ros::spinOnce();
      loop.sleep();
      continue;
    }

    if (rotate180){
      cv::rotate(frame, frame, cv::ROTATE_180);
    }

    geometry_msgs::Twist cmd_out = stopCmd();

    // ---------------- Patrol command (only sets cmd_out when WAIT_HAND & TURN) ----------------
    bool do_patrol = patrol_enable;
    if (patrol_only_when_wait_hand && state != State::WAIT_HAND) do_patrol = false;

    bool is_turning = false;

    if (do_patrol){
      if (now >= phase_deadline){
        if (phase == PatrolPhase::TURN){
          phase = PatrolPhase::PAUSE;
          pause_started = now;
          have_prev = false; // 重置 motion baseline，避免 turn->pause 巨差分
          phase_deadline = now + ros::Duration(pause_duration);
        }else{
          phase = PatrolPhase::TURN;
          phase_deadline = now + ros::Duration(turn_duration);

          sweep_count++;
          if (sweep_count >= sweep_steps){
            sweep_count = 0;
            turn_dir = -turn_dir;
          }
        }
      }

      if (state == State::WAIT_HAND && phase == PatrolPhase::TURN){
        cmd_out.angular.z = turn_speed * (double)turn_dir;
        is_turning = true;
      }
    }

    // ---------------- Detection prep ----------------
    const double det_scale = 0.5;
    cv::Mat small;
    cv::resize(frame, small, cv::Size(), det_scale, det_scale, cv::INTER_LINEAR);

    cv::Mat gray;
    cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);

    // motion baseline update (for hand)
    cv::Mat gray_blur;
    cv::GaussianBlur(gray, gray_blur, cv::Size(7,7), 0);

    // ---------------- Hand detection (ONLY in PAUSE & after settle) ----------------
    best_area = 0;
    area_ratio = 0.0;
    cy_ratio = 1.0;
    hand_now = false;
    motion_pixels = 0;
    motion_ok = false;

    bool allow_hand_detect =
      (state == State::WAIT_HAND) &&
      require_hand &&
      (phase == PatrolPhase::PAUSE) &&
      (!is_turning) &&
      ((now - pause_started).toSec() >= pause_settle_sec);

    if (!allow_hand_detect){
      hand_streak = 0;
    }else{
      cv::Mat ycrcb;
      cv::cvtColor(small, ycrcb, cv::COLOR_BGR2YCrCb);

      cv::Mat skin;
      cv::inRange(ycrcb, cv::Scalar(0, 133, 77), cv::Scalar(255, 173, 127), skin);

      cv::erode(skin, skin, cv::Mat(), cv::Point(-1,-1), 1);
      cv::dilate(skin, skin, cv::Mat(), cv::Point(-1,-1), 2);

      cv::Mat motion;
      if (have_prev){
        cv::Mat diff;
        cv::absdiff(gray_blur, prev_gray_blur, diff);
        cv::threshold(diff, motion, motion_thresh, 255, cv::THRESH_BINARY);
        cv::dilate(motion, motion, cv::Mat(), cv::Point(-1,-1), 1);
        motion_pixels = cv::countNonZero(motion);
      }else{
        motion_pixels = 0;
      }

      cv::Mat comb;
      if (have_prev && !motion.empty()){
        cv::bitwise_and(skin, motion, comb);
      }else{
        comb = cv::Mat::zeros(skin.size(), skin.type());
      }

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(comb, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      int w = comb.cols;
      int h = comb.rows;
      double img_area = (double)w * (double)h;

      int motion_global_max_pixels = (int)(motion_global_max_ratio * img_area);
      motion_ok = have_prev &&
                  (motion_pixels >= motion_min_pixels) &&
                  (motion_pixels <= motion_global_max_pixels);

      cv::Rect best_rect;
      for (auto &c : contours){
        double a = cv::contourArea(c);
        if ((int)a > best_area){
          best_area = (int)a;
          best_rect = cv::boundingRect(c);
        }
      }

      if (best_area > 0){
        area_ratio = (double)best_area / img_area;
        double cy = best_rect.y + best_rect.height * 0.5;
        cy_ratio = cy / (double)h;

        bool area_ok_hand = (area_ratio >= hand_area_min_ratio) && (area_ratio <= hand_area_max_ratio);
        bool y_ok = (cy_ratio <= hand_center_y_max_ratio);

        hand_now = area_ok_hand && y_ok && motion_ok;
      }else{
        area_ratio = 0.0;
        cy_ratio = 1.0;
        hand_now = false;
      }

      if (hand_now) hand_streak++;
      else hand_streak = 0;

      if (hand_streak >= hand_frames_required){
        ROS_WARN("HAND TRIGGERED -> start smile detection (phase=PAUSE settle_ok=1 motion_pixels=%d motion_ok=%d best_area=%d area_ratio=%.4f cy_ratio=%.3f)",
                 motion_pixels, (int)motion_ok, best_area, area_ratio, cy_ratio);

        state = State::WAIT_SMILE;
        smile_streak = 0;
        hand_streak = 0;

        // reset auto adjust
        adj_phase = AdjustPhase::IDLE;
        adj_action = AdjustAction::NONE;
        adjust_steps = 0;
        no_person_count = 0;
        framed_bad_count = 0;
      }
    }

    // ---------------- Smile detection + Auto adjust ----------------
    bool person = false;
    bool smile = false;
    bool framed = false;

    if (state == State::WAIT_SMILE){
      // If currently moving for auto-adjust, override cmd_out and skip detection
      if (auto_adjust_enable && adj_phase == AdjustPhase::MOVING){
        // override
        cmd_out = stopCmd();
        if (adj_action == AdjustAction::FORWARD){
          cmd_out.linear.x = approach_speed;
        }else if (adj_action == AdjustAction::BACKWARD){
          cmd_out.linear.x = -std::abs(retreat_speed);
        }else if (adj_action == AdjustAction::TURN_LEFT){
          cmd_out.angular.z = std::abs(align_ang_speed);
        }else if (adj_action == AdjustAction::TURN_RIGHT){
          cmd_out.angular.z = -std::abs(align_ang_speed);
        }

        if (now >= adj_deadline){
          adj_phase = AdjustPhase::SETTLING;
          adj_settle_until = now + ros::Duration(approach_settle_sec);
          adj_action = AdjustAction::NONE;
          cmd_out = stopCmd();
          // 走動後，要求重新累積 smile streak（更穩）
          smile_streak = 0;
        }
      }else{
        // settling
        if (auto_adjust_enable && adj_phase == AdjustPhase::SETTLING){
          cmd_out = stopCmd();
          if (now >= adj_settle_until){
            adj_phase = AdjustPhase::IDLE;
          }
        }

        // Only run detection when not MOVING
        if (!(auto_adjust_enable && adj_phase == AdjustPhase::SETTLING && now < adj_settle_until)){
          // We still allow detection during settling after stop (frames should be stable)
        }

        // Face detect (always run unless MOVING)
        std::vector<cv::Rect> faces;
        face_cascade.detectMultiScale(gray, faces, 1.1, 4, 0, cv::Size(40,40));

        person = !faces.empty();
        smile = false;
        framed = false;
        fa_ratio = 0.0;
        cxr = 0.5; cyr = 0.5;
        area_ok = false;
        center_ok = false;

        if (person){
          cv::Rect best = faces[0];
          int bestA = best.area();
          for (auto &r : faces){
            if (r.area() > bestA){ bestA = r.area(); best = r; }
          }

          double imgA = (double)gray.cols * (double)gray.rows;
          fa_ratio = (double)best.area() / imgA;

          double cx = best.x + best.width * 0.5;
          double cy = best.y + best.height * 0.5;
          cxr = cx / (double)gray.cols;
          cyr = cy / (double)gray.rows;

          area_ok = (fa_ratio >= face_area_min_ratio) && (fa_ratio <= face_area_max_ratio);
          center_ok = (std::abs(cxr - 0.5) <= face_center_margin) && (std::abs(cyr - 0.5) <= face_center_margin);
          framed = area_ok && center_ok;

          cv::Mat faceROI = gray(best);
          std::vector<cv::Rect> smiles;
          smile_cascade.detectMultiScale(faceROI, smiles, smile_scale, smile_neighbors, 0, cv::Size(15,15));
          smile = !smiles.empty();

          if (smile && framed) smile_streak = std::min(smile_frames_required, smile_streak + 1);
          else smile_streak = std::max(0, smile_streak - 1);

          if (smile_streak >= smile_frames_required){
            std::string path = save_dir + "/photo_" + nowStamp() + ".jpg";
            bool ok = cv::imwrite(path, frame);
            if (ok) ROS_WARN("CAPTURED: %s", path.c_str());
            else    ROS_ERROR("Failed to save: %s", path.c_str());

            state = State::COOLDOWN;
            cooldown_until = now + ros::Duration((double)cooldown_sec);

            // reset auto adjust
            adj_phase = AdjustPhase::IDLE;
            adj_action = AdjustAction::NONE;
            adjust_steps = 0;
            no_person_count = 0;
            framed_bad_count = 0;

            cmd_out = stopCmd();
          }
        }else{
          smile_streak = std::max(0, smile_streak - 1);
        }

        // ---- Auto adjust decision (only when IDLE, not during settling/moving) ----
        if (auto_adjust_enable && state == State::WAIT_SMILE && adj_phase == AdjustPhase::IDLE){
          if (!person){
            no_person_count++;
            framed_bad_count = 0;
          }else{
            no_person_count = 0;
            if (!framed) framed_bad_count++;
            else framed_bad_count = 0;
          }

          // fail-safe
          if (abort_on_fail && no_person_count >= abort_no_person_frames){
            ROS_WARN("AutoAdjust abort: too long PERSON=0 -> back to WAIT_HAND");
            state = State::WAIT_HAND;
            smile_streak = 0;
            hand_streak = 0;

            // 回 WAIT_HAND，從 PAUSE 開始、重置 baseline
            phase = PatrolPhase::PAUSE;
            pause_started = now;
            have_prev = false;
            phase_deadline = now + ros::Duration(pause_duration);

            adj_phase = AdjustPhase::IDLE;
            adj_action = AdjustAction::NONE;
            adjust_steps = 0;
            no_person_count = 0;
            framed_bad_count = 0;
          }else{
            bool need_step = false;
            if (!person && no_person_count >= no_person_frames_to_step) need_step = true;
            if (person && !framed && framed_bad_count >= framed_bad_frames_to_step) need_step = true;

            if (need_step && adjust_steps < approach_max_steps){
              AdjustAction act = AdjustAction::FORWARD;

              if (person && !framed){
                // Priority: distance (area) first, then optional centering
                if (fa_ratio < face_area_min_ratio){
                  act = AdjustAction::FORWARD;
                }else if (fa_ratio > face_area_max_ratio){
                  act = retreat_enable ? AdjustAction::BACKWARD : AdjustAction::FORWARD;
                }else if (align_enable && !center_ok){
                  // face on left -> turn left; face on right -> turn right
                  act = (cxr < 0.5) ? AdjustAction::TURN_LEFT : AdjustAction::TURN_RIGHT;
                }else{
                  // default: still try forward (most cases are "too far")
                  act = AdjustAction::FORWARD;
                }
              }else{
                // PERSON=0
                act = AdjustAction::FORWARD;
              }

              adj_action = act;
              adj_phase = AdjustPhase::MOVING;

              double dur = approach_step_sec;
              if (act == AdjustAction::TURN_LEFT || act == AdjustAction::TURN_RIGHT) dur = align_step_sec;

              adj_deadline = now + ros::Duration(dur);
              adjust_steps++;

              // reset counters so it won't spam steps instantly
              no_person_count = 0;
              framed_bad_count = 0;

              // override cmd this cycle immediately
              cmd_out = stopCmd();
              if (act == AdjustAction::FORWARD){
                cmd_out.linear.x = approach_speed;
              }else if (act == AdjustAction::BACKWARD){
                cmd_out.linear.x = -std::abs(retreat_speed);
              }else if (act == AdjustAction::TURN_LEFT){
                cmd_out.angular.z = std::abs(align_ang_speed);
              }else if (act == AdjustAction::TURN_RIGHT){
                cmd_out.angular.z = -std::abs(align_ang_speed);
              }

              // movement -> reset smile streak for stability
              smile_streak = 0;

              ROS_INFO("AutoAdjust STEP #%d action=%s (PERSON=%d FRAMED=%d fa_ratio=%.4f cxr=%.3f cyr=%.3f area_ok=%d center_ok=%d)",
                       adjust_steps, toStr(act), (int)person, (int)framed, fa_ratio, cxr, cyr, (int)area_ok, (int)center_ok);
            }
          }
        }
      }
    }

    if (state == State::COOLDOWN){
      cmd_out = stopCmd();
      if (now >= cooldown_until){
        state = State::WAIT_HAND;
        smile_streak = 0;
        hand_streak = 0;

        // 回 WAIT_HAND，從 PAUSE 開始、重置 baseline
        phase = PatrolPhase::PAUSE;
        pause_started = now;
        have_prev = false;
        phase_deadline = now + ros::Duration(pause_duration);

        // reset auto adjust
        adj_phase = AdjustPhase::IDLE;
        adj_action = AdjustAction::NONE;
        adjust_steps = 0;
        no_person_count = 0;
        framed_bad_count = 0;
      }
    }

    // publish cmd once per loop (priority already resolved)
    cmd_pub.publish(cmd_out);

    // update prev (for hand)
    prev_gray_blur = gray_blur.clone();
    have_prev = true;

    // ---------------- Logging ----------------
    if ((now - last_log).toSec() >= (1.0 / std::max(0.1, log_hz))){
      last_log = now;

      if (state == State::WAIT_HAND){
        const char* ph = (phase == PatrolPhase::TURN) ? "TURN" : "PAUSE";
        ROS_INFO("STATE=WAIT_HAND  patrol=%s  hand=%d  streak=%d/%d | best_area=%d  area_ratio=%.4f  cy_ratio=%.3f",
                 ph, (int)hand_now, hand_streak, hand_frames_required,
                 best_area, area_ratio, cy_ratio);
      }else if (state == State::WAIT_SMILE){
        ROS_INFO("STATE=WAIT_SMILE  PERSON=%d  SMILE=%d  FRAMED=%d  smile_streak=%d/%d | fa_ratio=%.4f cxr=%.3f cyr=%.3f area_ok=%d center_ok=%d | adj=%s steps=%d noP=%d framedBad=%d",
                 (int)person, (int)smile, (int)framed, smile_streak, smile_frames_required,
                 fa_ratio, cxr, cyr, (int)area_ok, (int)center_ok,
                 toStr(adj_phase), adjust_steps, no_person_count, framed_bad_count);
      }else{
        double remain = (cooldown_until - now).toSec();
        if (remain < 0) remain = 0;
        ROS_INFO("STATE=COOLDOWN  remain=%.1fs", remain);
      }
    }

    ros::spinOnce();
    loop.sleep();
  }

  // stop at exit
  cmd_pub.publish(stopCmd());
  return 0;
}

