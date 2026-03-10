#include <Arduino.h>
#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

/* ===== 亮度類比輸入 ===== */
const int LIGHT_PIN = A0;                  // 類比腳位
const unsigned long LIGHT_PUB_MS = 50;     // 20 Hz
std_msgs::Int16 light_msg;

/* ===== IR 數位輸入（Beacon ratio）===== */
const int IR_PIN = 13;                     // IR receiver Signal/Output 在 D13（Active-Low）
const unsigned long IR_PUB_MS = 50;        // 20 Hz
std_msgs::Float32 ir_ratio_msg;

/* ===== 可調參數 ===== */
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = false;
const int  PWM_MAX      = 250;
const int  PWM_MIN_FWD  = 60;
const int  PWM_MIN_BACK = 100;
const int  PWM_MIN_TURN = 80;
const int  MAX_SETPOINT = 260;
const int  SAMPLE_MS    = 100;

/* ===== L298N 腳位 ===== */
const int enA = 5, in1 = 8,  in2 = 9;     // 左
const int enB = 6, in3 = 10, in4 = 11;    // 右

/* ===== 編碼器（UNO: D2/D3） ===== */
const byte encoderA_A = 2, encoderA_B = 4; // 左 D2=INT0, D4
const byte encoderB_A = 3, encoderB_B = 7; // 右 D3=INT1, D7
volatile long countA = 0, countB = 0;
volatile byte lastA = LOW, lastB = LOW;
volatile bool dirA = true, dirB = true;

/* ===== PID ===== */
double SetpointA=0, InputA=0, OutputA=0;
double SetpointB=0, InputB=0, OutputB=0;
double Kp=0.6, Ki=6.5, Kd=0.0;
PID pidA(&InputA,&OutputA,&SetpointA,Kp,Ki,Kd,DIRECT);
PID pidB(&InputB,&OutputB,&SetpointB,Kp,Ki,Kd,DIRECT);

/* ===== ROS ===== */
ros::NodeHandle nh;
ros::Publisher  pub_light("light_data", &light_msg);
ros::Publisher  pub_ir("ir_ratio", &ir_ratio_msg);

volatile float cmd_lin=0.0f, cmd_ang=0.0f;

/* ===== 工具函式 ===== */
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

/* ===== 編碼器 ISR ===== */
void isrEncA(){
  int s=digitalRead(encoderA_A);
  if((lastA==LOW)&&(s==HIGH)) dirA=(digitalRead(encoderA_B)==HIGH);
  lastA=s;
  if(!dirA) countA++; else countA--;
}
void isrEncB(){
  int s=digitalRead(encoderB_A);
  if((lastB==LOW)&&(s==HIGH)) dirB=(digitalRead(encoderB_B)==HIGH);
  lastB=s;
  if(!dirB) countB++; else countB--;
}

/* ===== /cmd_vel → 左右目標 ===== */
void cmdVelCb(const geometry_msgs::Twist& m){
  cmd_lin=(float)m.linear.x;
  cmd_ang=(float)m.angular.z;
  float L = cmd_lin - cmd_ang;
  float R = cmd_lin + cmd_ang;
  if (L>1) L=1; if (L<-1) L=-1;
  if (R>1) R=1; if (R<-1) R=-1;
  SetpointA = L * MAX_SETPOINT;
  SetpointB = R * MAX_SETPOINT;
}
ros::Subscriber<geometry_msgs::Twist> sub_cmd("/cmd_vel", &cmdVelCb);

/* ===== PID 更新 ===== */
void updatePID(){
  long pa,pb;
  noInterrupts();
  pa = countA; countA = 0;
  pb = countB; countB = 0;  // 修正：避免誤用三元運算子
  interrupts();

  InputA = abs(pa);
  InputB = abs(pb);
  pidA.Compute();
  pidB.Compute();

  int pwmA = withFloor(OutputA, SetpointA, cmd_lin, cmd_ang);
  int pwmB = withFloor(OutputB, SetpointB, cmd_lin, cmd_ang);

  setWheel(enA,in1,in2,SetpointA,pwmA,INVERT_LEFT);
  setWheel(enB,in3,in4,SetpointB,pwmB,INVERT_RIGHT);
}

/* ===== IR ratio：短窗×多窗平均 + EMA（計 LOW 佔比；Active-Low） ===== */
float sampleIrRatioMicros(unsigned long window_us){
  unsigned long t_start = micros();
  unsigned long t_high = 0, t_low = 0;

  int last = digitalRead(IR_PIN);
  unsigned long t_edge = micros();

  while ((micros() - t_start) < window_us){
    int v = digitalRead(IR_PIN);
    unsigned long now = micros();
    if (v != last){
      if (last == HIGH) t_high += (now - t_edge);
      else              t_low  += (now - t_edge);
      last  = v;
      t_edge = now;
    }
  }
  unsigned long now = micros();
  if (last == HIGH) t_high += (now - t_edge);
  else              t_low  += (now - t_edge);

  unsigned long total = t_high + t_low;
  if (total == 0) return 0.0f;
  return (float)t_low / (float)total;  // 回傳 LOW 佔比
}

float ir_ema = 0.0f;                 // 指數平滑
const float IR_EMA_ALPHA = 0.6f;     // 越大越靈敏（0.5~0.7 建議）
const unsigned long IR_ONE_WINDOW_US = 15000; // 每窗 15ms
const int IR_MULTI_N = 3;                       // 3 窗平均（總 ~45ms）

float measureIrRatio(){
  float acc = 0.0f;
  for (int i=0;i<IR_MULTI_N;i++){
    acc += sampleIrRatioMicros(IR_ONE_WINDOW_US);
  }
  float avg = acc / (float)IR_MULTI_N;
  // EMA
  ir_ema = IR_EMA_ALPHA * avg + (1.0f - IR_EMA_ALPHA) * ir_ema;
  if (ir_ema < 0.0f) ir_ema = 0.0f;
  if (ir_ema > 1.0f) ir_ema = 1.0f;
  return ir_ema;
}

/* ===== 全域定時 ===== */
unsigned long lastPID=0, lastLightPub=0, lastIrPub=0;

void setup(){
  // 馬達
  pinMode(in1,OUTPUT); pinMode(in2,OUTPUT); pinMode(enA,OUTPUT);
  pinMode(in3,OUTPUT); pinMode(in4,OUTPUT); pinMode(enB,OUTPUT);
  analogWrite(enA,0);  analogWrite(enB,0);

  // 編碼器
  pinMode(encoderA_A,INPUT); pinMode(encoderA_B,INPUT);
  pinMode(encoderB_A,INPUT); pinMode(encoderB_B,INPUT);
  attachInterrupt(0, isrEncA, CHANGE); // D2
  attachInterrupt(1, isrEncB, CHANGE); // D3

  // 類比亮度
  pinMode(LIGHT_PIN, INPUT);
  analogReference(DEFAULT); // UNO 預設 5V

  // IR 數位輸入
  pinMode(IR_PIN, INPUT);   // 模組通常自帶上拉

  // PID
  pidA.SetSampleTime(SAMPLE_MS);
  pidB.SetSampleTime(SAMPLE_MS);
  pidA.SetOutputLimits(0, PWM_MAX);
  pidB.SetOutputLimits(0, PWM_MAX);
  pidA.SetMode(AUTOMATIC);
  pidB.SetMode(AUTOMATIC);

  // ROS
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.advertise(pub_light);
  nh.advertise(pub_ir);
}

void loop(){
  nh.spinOnce();
  unsigned long now = millis();

  // 亮度：超取樣去雜訊（丟極值）
  if (now - lastLightPub >= LIGHT_PUB_MS){
    lastLightPub = now;
    const int N = 10;
    int sum = 0, vmin = 1023, vmax = 0;
    for(int i=0;i<N;i++){
      int v = analogRead(LIGHT_PIN);
      sum += v;
      if (v<vmin) vmin=v;
      if (v>vmax) vmax=v;
      delayMicroseconds(500);
    }
    int avg = (sum - vmin - vmax) / (N - 2);
    light_msg.data = avg;
    pub_light.publish(&light_msg);
  }

  // IR ratio：每 50ms 量 ~45ms 總窗的 LOW 佔比（含 EMA）
  if (now - lastIrPub >= IR_PUB_MS){
    lastIrPub = now;
    float ratio = measureIrRatio();
    ir_ratio_msg.data = ratio;
    pub_ir.publish(&ir_ratio_msg);
  }

  // 週期性更新 PID
  if (now - lastPID >= SAMPLE_MS){
    lastPID = now;
    updatePID();
  }
}

