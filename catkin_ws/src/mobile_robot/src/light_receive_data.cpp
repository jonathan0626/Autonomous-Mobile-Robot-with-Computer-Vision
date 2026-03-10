#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <wiringPi.h>
#include <deque>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <limits>
#include <cstdlib>
#include <cmath>

// GPIO pins (BCM)
static const int PIN_BUMP_R = 17; // right bumper (active low)
static const int PIN_BUMP_L = 27; // left bumper (active low)
static const int PIN_KILL   = 23; // force beacon mode

// light parameters
static const double START_DELAY_MS = 600.0;
static const double BACK_MS        = 2000.0;
static const double ONE_TURN_MS    = 5000.0;
static const double SPIN_TURNS     = 2.1;
static const double SPIN_BLOCK_MS  = ONE_TURN_MS * SPIN_TURNS;

static int    LIGHT_WIN      = 5;
static int    LIGHT_BRIGHT_T = 800;
static int    LIGHT_DARK_T   = 900;
static double GLARE_FACTOR   = 0.85;

static const double V_CRUISE   = 0.90;
static const double V_APPROACH = 0.90;
static const double V_BACK     = 0.25;
static const double W_SEARCH   = 0.25;

static double FWD_TRIM_LEFT = 0;
bool TURN_RIGHT_POSITIVE = true;
int  BUMP_PULL_PARAM     = 0;

// puck capture parameters
static const double CAPTURE_HOLD_MS = 800.0;
static const int    CAPTURE_BRIGHT_MARGIN = 10;
static const double PUCK_LOST_MS = 350.0;

// beacon navigation
static const double COARSE_DRIVE_MS = 1600.0;
static const double V_COARSE        = 0.45;

static const float  IR_NOISE_FLOOR  = 0.05f;
static const float  IR_GOOD_FRAC    = 0.95f;
static const double V_BEACON_FWD    = 1.00;

static const double V_BEACON_SEEK   = 0.45;
static const double V_BEACON_NEAR   = 0.85;

static const double FALLBACK_DRIVE_MS = 2600.0;
static const double V_FALLBACK        = 0.45;
static const double W_FALLBACK_BIAS   = 0.15;

static const double PI = 3.14159265358979323846;
static const double W_BEACON_SCAN = 1.00;
static const double BEACON_SCAN_MS = (2.0 * PI / W_BEACON_SCAN) * 1000.0;

// IR smoothing
static const float IR_PI_ALPHA = 0.50f;

// beacon windows
static double BEACON_RATIO_MIN = 0.27;
static double BEACON_RATIO_MAX = 0.32;
static double BEACON_EXIT_MIN  = 0.24;
static double BEACON_EXIT_MAX  = 0.35;
static const double BEACON_EXIT_HOLD_MS = 1200.0;

// beacon profiles
struct BeaconProfile {
  const char* name;
  double enter_min, enter_max;
  double exit_min, exit_max;
};

static const BeaconProfile PROF_BEACON1_600  = {"BEACON1_600", 0.23,0.32,0.21,0.35};
static const BeaconProfile PROF_BEACON2_1500 = {"BEACON2_1500",0.38,0.48,0.37,0.50};

// state machine
enum State {
  WAIT_START=0, PRE_CALIB_FWD, CALIB_RUN,
  CRUISE, BACK, SPIN_FIND,
  APPROACH, APPROACH_FINE_TUNE, FINAL_APPROACH,
  SPIN_ALIGN_MIN, MIN_APPROACH,
  PUCK_COARSE_DRIVE,
  BEACON_SEARCH, BEACON_FALLBACK_DRIVE, BEACON_APPROACH
};

static inline const char* stateName(State s){
  switch(s){
    case WAIT_START: return "WAIT_START";
    case PRE_CALIB_FWD: return "PRE_CALIB_FWD";
    case CALIB_RUN: return "CALIB_RUN";
    case CRUISE: return "CRUISE";
    case BACK: return "BACK";
    case SPIN_FIND: return "SPIN_FIND";
    case APPROACH: return "APPROACH";
    case APPROACH_FINE_TUNE: return "APPROACH_FINE_TUNE";
    case FINAL_APPROACH: return "FINAL_APPROACH";
    case SPIN_ALIGN_MIN: return "SPIN_ALIGN_MIN";
    case MIN_APPROACH: return "MIN_APPROACH";
    case PUCK_COARSE_DRIVE: return "PUCK_COARSE_DRIVE";
    case BEACON_SEARCH: return "BEACON_SEARCH";
    case BEACON_FALLBACK_DRIVE: return "BEACON_FALLBACK_DRIVE";
    case BEACON_APPROACH: return "BEACON_APPROACH";
    default: return "UNKNOWN";
  }
}

static inline void stopTwist(geometry_msgs::Twist &tw){
  tw.linear.x = 0.0;
  tw.angular.z = 0.0;
}
// light filter
static int g_LIGHT_WIN_runtime = 7;

struct LightFilter {
  std::deque<int> win;
  int sum=0, avg=0;
  bool light_on=false;
  int consec_on=0, consec_off=0;

  void push(int v){
    win.push_back(v);
    sum += v;

    if ((int)win.size() > g_LIGHT_WIN_runtime){
      sum -= win.front();
      win.pop_front();
    }

    int n = (int)win.size();
    avg = (n>0) ? (sum/n) : v;

    if (!light_on){
      if (avg <= LIGHT_BRIGHT_T){ consec_on++; consec_off=0; }
      else                      { consec_off++; consec_on=0; }
      if (consec_on >= 2) light_on = true;
    } else {
      if (avg >= LIGHT_DARK_T)   { consec_off++; consec_on=0; }
      else                       { consec_on++;  consec_off=0; }
      if (consec_off >= 6) light_on = false;
    }
  }
} g_light;

volatile int   g_last_light  = 0;
volatile float g_ir_ratio    = 0.0f;

void lightCb(const std_msgs::Int16::ConstPtr& msg){
  g_last_light = msg->data;
}

void irCb(const std_msgs::Float32::ConstPtr& msg){
  g_ir_ratio = msg->data;
}

// keyboard input handling
static termios g_old_tio;
static bool g_raw_on=false;

void enableStdinRaw(){
  if (g_raw_on) return;

  termios new_tio{};
  tcgetattr(STDIN_FILENO, &g_old_tio);
  new_tio = g_old_tio;

  new_tio.c_lflag &= ~(ICANON | ECHO);
  new_tio.c_cc[VMIN]=0;
  new_tio.c_cc[VTIME]=0;

  tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
  g_raw_on = true;
}

void restoreStdin(){
  if (!g_raw_on) return;
  tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio);
  g_raw_on = false;
}

int readKeyNonBlock(){
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(STDIN_FILENO, &rfds);

  timeval tv{0,0};

  int r = select(STDIN_FILENO+1, &rfds, nullptr, nullptr, &tv);

  if (r>0 && FD_ISSET(STDIN_FILENO, &rfds)){
    unsigned char c;
    if (read(STDIN_FILENO, &c, 1)==1) return (int)c;
  }

  return -1;
}

// beacon profile selection
enum BeaconChoice { CH_BEACON1=1, CH_BEACON2=2 };
static BeaconChoice g_beacon_choice = CH_BEACON1;

void applyBeaconProfile(BeaconChoice ch){
  const BeaconProfile* p =
    (ch==CH_BEACON2) ? &PROF_BEACON2_1500 : &PROF_BEACON1_600;

  BEACON_RATIO_MIN = p->enter_min;
  BEACON_RATIO_MAX = p->enter_max;
  BEACON_EXIT_MIN  = p->exit_min;
  BEACON_EXIT_MAX  = p->exit_max;

  ROS_INFO("[BEACON PROFILE] %s enter=[%.2f~%.2f] exit=[%.2f~%.2f]",
           p->name,
           p->enter_min,
           p->enter_max,
           p->exit_min,
           p->exit_max);
}

int main(int argc, char** argv){

  ros::init(argc, argv, "rpi_light_nav");
  ros::NodeHandle nh;

  nh.param("turn_right_positive", TURN_RIGHT_POSITIVE, true);
  nh.param("bump_pull", BUMP_PULL_PARAM, 0);
  nh.param("light_bright_thresh", LIGHT_BRIGHT_T, LIGHT_BRIGHT_T);
  nh.param("light_dark_thresh",   LIGHT_DARK_T,   LIGHT_DARK_T);
  nh.param("glare_factor",        GLARE_FACTOR,   GLARE_FACTOR);
  nh.param("fwd_trim_left",       FWD_TRIM_LEFT,  FWD_TRIM_LEFT);
  nh.param("light_win",           g_LIGHT_WIN_runtime, LIGHT_WIN);

  nh.param("beacon_ratio_min", BEACON_RATIO_MIN, BEACON_RATIO_MIN);
  nh.param("beacon_ratio_max", BEACON_RATIO_MAX, BEACON_RATIO_MAX);
  nh.param("beacon_exit_min",  BEACON_EXIT_MIN,  BEACON_EXIT_MIN);
  nh.param("beacon_exit_max",  BEACON_EXIT_MAX,  BEACON_EXIT_MAX);

  setenv("WIRINGPI_GPIOMEM", "1", 1);

  if (wiringPiSetupGpio()!=0){
    ROS_ERROR("wiringPiSetupGpio() failed");
    return 1;
  }

  pinMode(PIN_BUMP_R, INPUT);
  pullUpDnControl(PIN_BUMP_R,
                  (BUMP_PULL_PARAM==1)?PUD_UP:PUD_OFF);

  pinMode(PIN_BUMP_L, INPUT);
  pullUpDnControl(PIN_BUMP_L,
                  (BUMP_PULL_PARAM==1)?PUD_UP:PUD_OFF);

  pinMode(PIN_KILL, INPUT);
  pullUpDnControl(PIN_KILL,
                  (BUMP_PULL_PARAM==1)?PUD_UP:PUD_OFF);

  ros::Publisher vel_pub =
    nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

  ros::Subscriber sub_lux =
    nh.subscribe<std_msgs::Int16>("light_data",10,&lightCb);

  ros::Subscriber sub_ir =
    nh.subscribe<std_msgs::Float32>("ir_ratio",10,&irCb);

  ros::Rate rate(20.0);

  auto yaw_right =
    [&](double w){ return TURN_RIGHT_POSITIVE ? +w : -w; };

  State state = WAIT_START;

  ros::Time t_state = ros::Time::now();
  ros::Time t_loop_prev = t_state;

  bool beacon_mode_forced = false;
  bool beacon_out_timing  = false;

  ros::Time beacon_out_start;

    // timers
  double capture_ms = 0.0;
  double puck_lost_ms = 0.0;

  // spin tracking
  double spin_elapsed_ms = 0.0;
  int spin_min_avg = std::numeric_limits<int>::max();
  bool spin_min_valid = false;

  const int ALIGN_EPS_LUX = 15;
  const double ALIGN_TIMEOUT_MS = 3000.0;
  const double NO_IMPROVE_MS = 700.0;

  const double W_SEARCH_ALIGN = W_SEARCH * 0.65;

  int align_dir = +1;
  int align_best = std::numeric_limits<int>::max();
  ros::Time align_t_start;
  ros::Time align_t_noimp;

  const double MIN_APPROACH_MS = 1500.0;

  // IR smoothing + best value
  float ir_pi_ema = 0.0f;
  float ir_best = 0.0f;
  ros::Time t_best = ros::Time(0);

  double beacon_scan_ms = 0.0;

  auto resetBeaconHelpers = [&](){
    ir_best = 0.0f;
    t_best = ros::Time::now();
    puck_lost_ms = 0.0;
    beacon_scan_ms = 0.0;
  };

  auto startBeaconPipeline =
  [&](ros::Time now, const char* reason){

    if (!beacon_mode_forced){
      beacon_mode_forced = true;
      applyBeaconProfile(g_beacon_choice);
    }

    resetBeaconHelpers();
    beacon_out_timing=false;

    state = PUCK_COARSE_DRIVE;
    t_state = now;

    ROS_WARN("[BEACON] start (%s)", reason);
  };

  ROS_INFO("READY: press 's' (600 beacon) or 'd' (1500 beacon)");
  enableStdinRaw();

  while (ros::ok()){

    ros::spinOnce();

    ros::Time now = ros::Time::now();

    double dt_ms =
      (now - t_loop_prev).toSec() * 1000.0;

    double elapsed_ms =
      (now - t_state).toSec() * 1000.0;

    t_loop_prev = now;

    g_light.push(g_last_light);

    bool glare =
      (g_light.avg <= (int)(LIGHT_DARK_T * GLARE_FACTOR));

    float ir_raw = g_ir_ratio;

    ir_pi_ema =
      IR_PI_ALPHA * ir_raw +
      (1.0f - IR_PI_ALPHA) * ir_pi_ema;

    if (ir_pi_ema < 0.0f) ir_pi_ema = 0.0f;
    if (ir_pi_ema > 1.0f) ir_pi_ema = 1.0f;

    float ir = ir_pi_ema;

    if ((now - t_best).toSec() > 3.0)
      ir_best = 0.0f;

    if (ir > ir_best){
      ir_best = ir;
      t_best = now;
    }

    bool bump_r =
      (digitalRead(PIN_BUMP_R) == 0);

    bool bump_l =
      (digitalRead(PIN_BUMP_L) == 0);

    bool bumped =
      bump_l || bump_r;

    geometry_msgs::Twist cmd;
    stopTwist(cmd);

    auto inEnterWindow = [&](){
      return (ir >= (float)BEACON_RATIO_MIN &&
              ir <= (float)BEACON_RATIO_MAX);
    };

    auto inExitWindow = [&](){
      return (ir >= (float)BEACON_EXIT_MIN &&
              ir <= (float)BEACON_EXIT_MAX);
    };

    // manual trigger
    if (digitalRead(PIN_KILL)==0){
      if (!beacon_mode_forced){
        startBeaconPipeline(now,"KILL");
      }
    }

    switch(state){

      case WAIT_START:{
        int ch = readKeyNonBlock();

        if (ch=='s' || ch=='S'){
          g_beacon_choice = CH_BEACON1;
          applyBeaconProfile(g_beacon_choice);

          state = PRE_CALIB_FWD;
          t_state = now;

          restoreStdin();

          ROS_INFO("Beacon profile: 600");
        }

        else if (ch=='d' || ch=='D'){
          g_beacon_choice = CH_BEACON2;
          applyBeaconProfile(g_beacon_choice);

          state = PRE_CALIB_FWD;
          t_state = now;

          restoreStdin();

          ROS_INFO("Beacon profile: 1500");
        }

        break;
      }

      case PRE_CALIB_FWD:

        if (bumped){
          state = BACK;
          t_state = now;
          break;
        }

        cmd.linear.x  = V_CRUISE;
        cmd.angular.z = -FWD_TRIM_LEFT;

        if (elapsed_ms >= 8000.0){
          state = CALIB_RUN;
          t_state = now;
        }

        break;

      case CRUISE:

        if (bumped){
          state = BACK;
          t_state = now;
          break;
        }

        if (!glare && g_light.light_on){

          state = APPROACH;
          t_state = now;

          capture_ms = 0.0;

          break;
        }

        cmd.linear.x  = V_CRUISE;
        cmd.angular.z = -FWD_TRIM_LEFT;

        break;

      case BACK:

        cmd.linear.x = -V_BACK;

        if (elapsed_ms >= BACK_MS){

          if (beacon_mode_forced){

            state = BEACON_SEARCH;

            t_state = now;

            resetBeaconHelpers();
          }

          else{

            state = SPIN_FIND;

            t_state = now;

            spin_elapsed_ms = 0.0;
            spin_min_avg = std::numeric_limits<int>::max();
            spin_min_valid = false;
          }
        }

        break;

      case BEACON_SEARCH:{

        if (inEnterWindow()){

          state = BEACON_APPROACH;
          t_state = now;

          beacon_out_timing=false;

          break;
        }

        bool ir_detected =
          (ir >= IR_NOISE_FLOOR);

        bool near_best =
          (ir_best > 0.0f &&
           ir >= ir_best * IR_GOOD_FRAC);

        if (!ir_detected){

          beacon_scan_ms += dt_ms;

          cmd.angular.z =
            yaw_right(W_BEACON_SCAN);

          if (beacon_scan_ms >= BEACON_SCAN_MS){

            state = BEACON_FALLBACK_DRIVE;
            t_state = now;

            beacon_scan_ms = 0.0;
          }
        }

        else{

          beacon_scan_ms = 0.0;

          if (near_best){

            cmd.linear.x = V_BEACON_NEAR;
          }

          else{

            cmd.linear.x = V_BEACON_SEEK;
          }

          cmd.angular.z = -FWD_TRIM_LEFT;
        }

        break;
      }

      case BEACON_APPROACH:{

        if (!inExitWindow()){

          if (!beacon_out_timing){

            beacon_out_timing=true;
            beacon_out_start=now;
          }

          else{

            if ((now - beacon_out_start).toSec()*1000.0
                >= BEACON_EXIT_HOLD_MS){

              state = BEACON_SEARCH;
              t_state = now;

              resetBeaconHelpers();
              beacon_out_timing=false;

              break;
            }
          }
        }

        else{

          beacon_out_timing=false;
        }

        cmd.linear.x = V_BEACON_FWD;
        cmd.angular.z = -FWD_TRIM_LEFT;

        if (bumped){

          state = BACK;
          t_state = now;

          beacon_out_timing=false;
        }

        break;
      }

      default:

        state = CRUISE;
        t_state = now;

        break;
    }

    vel_pub.publish(cmd);

    ROS_INFO_THROTTLE(
      1.0,
      "[STATE] %s lux=%d avg=%d IR=%.3f v=%.2f w=%.2f",
      stateName(state),
      g_last_light,
      g_light.avg,
      (double)ir,
      cmd.linear.x,
      cmd.angular.z
    );

    rate.sleep();
  }

  restoreStdin();
  return 0;
}