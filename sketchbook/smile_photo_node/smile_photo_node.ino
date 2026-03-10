#include <Arduino.h>
#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

/* ===== 可調參數（動了就有感） ===== */
const bool INVERT_LEFT  = false;    // 左輪方向反了就改 true
const bool INVERT_RIGHT = false;    // 右輪方向反了就改 true
const int  PWM_MAX      = 220;      // PWM 上限

// 針對不同動作給「最小門檻」（克服靜摩擦）
const int  PWM_MIN_FWD  = 60;       // 直行
const int  PWM_MIN_BACK = 90;       // 倒退
const int  PWM_MIN_TURN = 80;       // 原地/單輪轉

const int  MAX_SETPOINT = 260;      // cmd_vel=1.0 對應每100ms脈衝數
const int  SAMPLE_MS    = 100;      // 取樣/更新週期

// ★安全：多久沒收到 cmd_vel 就停車（ms）
const unsigned long CMD_TIMEOUT_MS = 500;

/* ===== L298N 腳位 ===== */
const int enA = 5,  in1 = 8,  in2 = 9;     // 左輪
const int enB = 6,  in3 = 10, in4 = 11;    // 右輪

/* ===== 編碼器腳位（UNO: D2/D3 中斷） ===== */
const byte encoderA_A = 2, encoderA_B = 4;   // 左：D2=INT0, D4
const byte encoderB_A = 3, encoderB_B = 7;   // 右：D3=INT1, D7
volatile long countA = 0, countB = 0;
volatile byte lastA = LOW, lastB = LOW;
volatile bool dirA = true, dirB = true;

/* ===== PID ===== */
double SetpointA=0, InputA=0, OutputA=0;
double SetpointB=0, InputB=0, OutputB=0;
double Kp=0.9, Ki=6.5, Kd=0.0;
PID pidA(&InputA,&OutputA,&SetpointA,Kp,Ki,Kd,DIRECT);
PID pidB(&InputB,&OutputB,&SetpointB,Kp,Ki,Kd,DIRECT);

/* ===== ROS ===== */
ros::NodeHandle nh;
volatile float cmd_lin=0.0f, cmd_ang=0.0f;
volatile unsigned long last_cmd_ms = 0;

/* ===== 工具：依動作挑 PWM 最小門檻 ===== */
inline int pickFloor(double sp, double lin, double ang){
  if (sp == 0) return 0;
  if (lin < -0.05 && fabs(ang) < 0.1) return PWM_MIN_BACK;
  if (fabs(lin) < 0.05 && fabs(ang) >= 0.1) return PWM_MIN_TURN;
  return PWM_MIN_FWD;
}
inline int withFloor(double out, double sp, double lin, double ang){
  if (sp == 0) return 0;
  int pwm = (int)out;
  int floor = pickFloor(sp, lin, ang);
  if (pwm < floor) pwm = floor;
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  return pwm;
}

inline void setWheel(int pwmPin,int inA,int inB,double setpoint,int pwmCmd,bool invert){
  bool forward = (setpoint >= 0);
  if (invert) forward = !forward;
  if (forward){ digitalWrite(inA,LOW);  digitalWrite(inB,HIGH); }
  else        { digitalWrite(inA,HIGH); digitalWrite(inB,LOW);  }
  analogWrite(pwmPin, pwmCmd);
}

inline void stopMotors(){
  SetpointA = 0; SetpointB = 0;
  cmd_lin = 0; cmd_ang = 0;
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  // 方向腳位保持也行，這裡不強制改
}

/* ===== 編碼器 ISR ===== */
void isrEncA(){
  int s=digitalRead(encoderA_A);
  if((lastA==LOW)&&(s==HIGH)) dirA=(digitalRead(encoderA_B)==HIGH);
  lastA=s; if(!dirA) countA++; else countA--;
}
void isrEncB(){
  int s=digitalRead(encoderB_A);
  if((lastB==LOW)&&(s==HIGH)) dirB=(digitalRead(encoderB_B)==HIGH);
  lastB=s; if(!dirB) countB++; else countB--;
}

/* ===== /cmd_vel → 左右目標 ===== */
void cmdVelCb(const geometry_msgs::Twist& m){
  last_cmd_ms = millis();

  cmd_lin=(float)m.linear.x;
  cmd_ang=(float)m.angular.z;

  float L = cmd_lin - cmd_ang;
  float R = cmd_lin + cmd_ang;

  if (L>1) L=1; if (L<-1) L=-1;
  if (R>1) R=1; if (R<-1) R=-1;

  SetpointA = L * MAX_SETPOINT;
  SetpointB = R * MAX_SETPOINT;
}
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel",&cmdVelCb);

/* ===== PID 更新 ===== */
void updatePID(){
  // watchdog：太久沒收到 cmd_vel 就停
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS){
    stopMotors();
    return;
  }

  long pa,pb;
  noInterrupts(); pa=countA; countA=0; pb=countB; countB=0; interrupts();

  InputA=abs(pa);
  InputB=abs(pb);

  pidA.Compute();
  pidB.Compute();

  int pwmA = withFloor(OutputA, SetpointA, cmd_lin, cmd_ang);
  int pwmB = withFloor(OutputB, SetpointB, cmd_lin, cmd_ang);

  setWheel(enA,in1,in2,SetpointA,pwmA,INVERT_LEFT);
  setWheel(enB,in3,in4,SetpointB,pwmB,INVERT_RIGHT);
}

/* ===== 主程式 ===== */
void setup(){
  pinMode(in1,OUTPUT); pinMode(in2,OUTPUT); pinMode(enA,OUTPUT);
  pinMode(in3,OUTPUT); pinMode(in4,OUTPUT); pinMode(enB,OUTPUT);
  analogWrite(enA,0); analogWrite(enB,0);

  pinMode(encoderA_A,INPUT); pinMode(encoderA_B,INPUT);
  pinMode(encoderB_A,INPUT); pinMode(encoderB_B,INPUT);

  attachInterrupt(0, isrEncA, CHANGE); // D2
  attachInterrupt(1, isrEncB, CHANGE); // D3

  pidA.SetSampleTime(SAMPLE_MS); pidB.SetSampleTime(SAMPLE_MS);
  pidA.SetOutputLimits(0,PWM_MAX); pidB.SetOutputLimits(0,PWM_MAX);
  pidA.SetMode(AUTOMATIC); pidB.SetMode(AUTOMATIC);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  last_cmd_ms = millis();
}

unsigned long lastPID=0;
void loop(){
  nh.spinOnce();
  unsigned long now=millis();
  if(now-lastPID>=SAMPLE_MS){ lastPID=now; updatePID(); }
}

