#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//灰度引脚
#define L_HUI 34
#define R_HUI 35
// 定义电机引脚
// 6 颗 Sharp IR GP2Y0A41SK0F 引脚定义
#define L_PIN 19
#define R_PIN 20
#define BL_PIN 15
#define BR_PIN 16
#define FL_PIN 17
#define FR_PIN 18

/* ============ 完全独立的 6 份代码 ============ */
#define PWM1 6  // rightforward
#define INA1 7
#define INB1 8

#define PWM2 3  // leftforward
#define INA2 4
#define INB2 5

#define PWM3 12 // rightback
#define INA3 13
#define INB3 14

#define PWM4 9  // leftback
#define INA4 10
#define INB4 11

#define guan1 36
#define guan2 38
#define guan3 39
#define guan4 40
// ========== OLED 屏幕定义 ==========
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ========== MPU6050 对象 ==========
MPU6050 mpu;

// ========== 全局变量 ==========
float alpha = 0.98;
float dt = 0.02;
unsigned long lastTime = 0, lastDisplay = 0;

float pitch = 0, roll = 0, yaw = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

int a = 1;
String mode;
unsigned long dropStartTime = 0;  // 记录 drop_check() 开始运行的时间
bool dropRunning = false;         // 标记 drop_check 是否正在运行

unsigned long upConfirmTime = 0; 
const unsigned long CONFIRM_DURATION = 1200; // 确认期1.2秒

void setupArena_mpu6050u();
void update_mpu6050();
void oled_mpu6050();
void autoFightLoop();
void autoFightLoop_2();
void checkIRandFight();
void checkArenaSafety_2();
void rushToArena();

void DelayImprove(int time) //delay函数的进阶版本，当检测到光电传感器的变化时，delay终止，继续下一步程序的运行
{

  bool bl_original = readbackwardleft();   // guan1
  bool fr_original = readforwardright();   // guan2
  bool fl_original = readforwardleft();    // guan3
  bool br_original = readbackwardright();  // guan4
  for (int i = 0; (i < time) && (fr_original == readforwardright()) && (fl_original == readforwardleft() && (br_original == readbackwardright()) && (bl_original == readbackwardleft())); i++) {
    delay(1);
  }
}

// 以下为红外检测距离函数
int getDist_FL() {
  int raw = analogRead(FL_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  return map(cm, 4, 30, 0, 35);
}

int getDist_FR() {
  int raw = analogRead(FR_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  return map(cm, 4, 30, 0, 80);
}

int getDist_L() {
  int raw = analogRead(L_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  return map(cm, 4, 30, 0, 80);
}

int getDist_R() {
  int raw = analogRead(R_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  return map(cm, 4, 30, 0, 80);
}

int getDist_BL() {
  int raw = analogRead(BL_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  return map(cm, 4, 30, 0, 80);
}

int getDist_BR() {
  int raw = analogRead(BR_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  return map(cm, 4, 30, 0, 80);
}
//以下为初始化函数
void setupArena() {
  pinMode(guan1, INPUT);
  pinMode(guan2, INPUT);
  pinMode(guan3, INPUT);
  pinMode(guan4, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);

  pinMode(PWM2, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);

  pinMode(PWM3, OUTPUT);
  pinMode(INA3, OUTPUT);
  pinMode(INB3, OUTPUT);

  pinMode(PWM4, OUTPUT);
  pinMode(INA4, OUTPUT);
  pinMode(INB4, OUTPUT);
  setupArena_mpu6050u();
  mode="autofight";
  show_oled("autofight");
}
//以下是红外测距传感器函数，若距离很近则返回1
int getio_FL() {
  int raw = analogRead(FL_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  if (map(cm, 4, 30, 0, 80) < 50)
    return 1;
  if (map(cm, 4, 30, 0, 80) >= 50)
    return 0;
}

int getio_FR() {
  int raw = analogRead(FR_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  if (map(cm, 4, 30, 0, 80) < 50)
    return 1;
  if (map(cm, 4, 30, 0, 80) >= 50)
    return 0;
}

int getio_L() {
  int raw = analogRead(L_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  if (map(cm, 4, 30, 0, 80) < 50)
    return 1;
  if (map(cm, 4, 30, 0, 80) >= 50)
    return 0;
}

int getio_R() {
  int raw = analogRead(R_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  if (map(cm, 4, 30, 0, 80) < 50)
    return 1;
  if (map(cm, 4, 30, 0, 80) >= 50)
    return 0;
}

int getio_BL() {
  int raw = analogRead(BL_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  if (map(cm, 4, 30, 0, 80) < 50)
    return 1;
  if (map(cm, 4, 30, 0, 80) >= 50)
    return 0;
}

int getio_BR() {
  int raw = analogRead(BR_PIN);
  float volts = raw * 0.0008056640625f;
  int cm = (int)(29.988f * pow(volts, -1.173f));
  if (cm > 30) cm = 30;
  if (cm < 4) cm = 4;
  if (map(cm, 4, 30, 0, 80) < 50)
    return 1;
  if (map(cm, 4, 30, 0, 80) >= 50)
    return 0;
}
//以下为光电传感器函数
bool readbackwardleft() {
  Serial.println("guan1:");
  Serial.println(digitalRead(guan1));
  return digitalRead(guan1);
}

bool readforwardright() {
  Serial.println("guan2:");
  Serial.println(digitalRead(guan2));
  return digitalRead(guan2);
}

bool readforwardleft() {
  Serial.println("guan3:");
  Serial.println(digitalRead(guan3));
  return digitalRead(guan3);
}

bool readbackwardright() {
  Serial.println("guan4:");
  Serial.println(digitalRead(guan4));
  return digitalRead(guan4);
}

// 控制单个电机的转速和方向
void controlMotor(int pwmPin, int inaPin, int inbPin, int speed, bool direction) {
  // 设置转速（PWM值）
  analogWrite(pwmPin, speed);

  // 设置方向
  if (direction) {
    digitalWrite(inaPin, HIGH);
    digitalWrite(inbPin, LOW);
  } else {
    digitalWrite(inaPin, LOW);
    digitalWrite(inbPin, HIGH);
  }
}

// 控制四个电机的转速和方向
void controlMotors(int speed1, bool direction1, int speed2, bool direction2, int speed3, bool direction3, int speed4, bool direction4) {
  controlMotor(PWM1, INA1, INB1, speed1, direction1);
  controlMotor(PWM2, INA2, INB2, speed2, direction2);
  controlMotor(PWM3, INA3, INB3, speed3, direction3);
  controlMotor(PWM4, INA4, INB4, speed4, direction4);
}

void superbackward() {
  delay(50);
  // controlMotors(170, false, 255, false, 200, false, 255, true);
  // controlMotors(150, false, 150, false, 150, false, 150, true);
  controlMotors(255, 0, 255, 0, 255, 1, 255, 0);
}

void superforward() {
  delay(50);
  // controlMotors(255, true, 170, true, 0, true, 0, false);
  controlMotors(170, 1, 255, true, 200, 0, 255, 1);
}

void backward() {
  delay(50);
  controlMotors(75,0, 75, false, 62, 1, 75, 0);
  // controlMotors(30, false, 30, false, 30, false, 30, true);
}

void forward() {
  delay(50);
  controlMotors(75, 1, 75, true, 62, 0, 75, 1);
  // controlMotors(30, true, 30, true, 30, true, 30, false);
}

void left() {
  delay(50);
  // controlMotors(170, false, 255, true, 200, false, 255, false);
  controlMotors(85, 0, 127, true, 100, 1, 127, 1);
}

void right() {
  delay(50);
  // controlMotors(255, true, 255, false, 220, true, 0, true);
  controlMotors(120, 1, 127, 0, 100, 0, 127, 0);
}

void stop() {
  delay(50);
  controlMotors(0, 1, 0, false, 0, 1, 0, 0);
}
//控制小车旋转固定方向
void rotateright(int angle) {
  int delayTime = angle * 4;
  right();
  delay(delayTime);
  stop();
}
void rotateLeft(int angle) {
  int delayTime = angle * 4;
  left();
  delay(delayTime);
  stop();
}
//此函数用于检测擂台
void checkArenaSafety() {
  bool bl = readbackwardleft();   // guan1
  bool fr = readforwardright();   // guan2
  bool fl = readforwardleft();    // guan3
  bool br = readbackwardright();  // guan4

  // 当前侧两个或后侧两个同时检测到擂台
  if ((fl && fr)) {
    stop();
    Serial.println("⚠️ 前/后同时检测到擂台 → 后退并旋转180°");
    backward();
    DelayImprove(100);
    stop();
    rotateLeft(180);//180
    mode="autofight";
    show_oled("autofight");
    return;
  }
  if ((bl && br)) {
    stop();
    Serial.println("⚠️ 前/后同时检测到擂台 → 后退并旋转180°");
    forward();
    DelayImprove(100);
    mode="autofight";
    show_oled("autofight");
    return;
  }
  // 左侧两个同时检测到
  if (fl && bl) {
    stop();
    Serial.println("⚠️ 左侧检测到擂台 → 右转90°");
    rotateright(90);//90
    mode="autofight";
    show_oled("autofight");
    return;
  }

  // 右侧两个同时检测到
  if (fr && br) {
    stop();
    Serial.println("⚠️ 右侧检测到擂台 → 左转90°");
    rotateLeft(90);
    mode="autofight";
    show_oled("autofight");
    return;
  }

  // 单个传感器触发的情况
  if (fl) {
    stop();
    Serial.println("⚠️ 前左检测到擂台 → 右转135°");
    backward();
    DelayImprove(100);
    stop();
    rotateright(135);//135
    mode="autofight";
    show_oled("autofight");
    return;
  }

  if (fr) {
    stop();
    Serial.println("⚠️ 前右检测到擂台 → 左转135°");
    backward();
    DelayImprove(200);
    stop();
    rotateLeft(135);//135
    mode="autofight";
    show_oled("autofight");
    return;
  }

  if (bl) {
    stop();
    Serial.println("⚠️ 后左检测到擂台 → 右转45°");
    forward();
    DelayImprove(130);
    stop();
    rotateright(45);//45
    mode="autofight";
    show_oled("autofight");
    return;
  }

  if (br) {
    stop();
    Serial.println("⚠️ 后右检测到擂台 → 左转45°");
    forward();
    DelayImprove(100);
    stop();
    rotateLeft(45);//45
    mode="autofight";
    show_oled("autofight");
    return;
  }
}
//此函数用于连续循环检测擂台和障碍，若无边缘或障碍则一直前进
void autoFight() {
    // checkIRandFight();
    forward();   // 前进
    delay(50);  // 短暂前进后继续检测（数值可调）
    checkArenaSafety();  // 检测是否接近擂台边缘并执行避让动作
  
}
//此函数用于冲台，冲上去之后旋转180度
void rushToArena() {
  Serial.println("🚀 比赛开始！小车正在冲上擂台！");
  // superbackward();     // 高速前进
  // delay(1200);
  // 冲刺时间，可根据距离微调（ms）
  rotateLeft(75);
  delay(1000);
  backward();
  delay(1200);
  rotateright(90);
    superbackward();
  delay(1300);
 
  stop();
  // stop();             // 到达后立即停止

  Serial.println("✅ 已成功冲上擂台并停止！");
}
// ====================================================
// 封装函数 1：初始化 MPU6050 + OLED
// ====================================================
void setupArena_mpu6050u() {
  Wire.begin(1, 2); // SDA=1, SCL=2（可根据实际改）
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050连接失败");
    while (1);
  }

  // 校准
  Serial.println("校准中，请保持静止...");
  for (int i = 0; i < 200; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    gx_offset += gx;
    gy_offset += gy;
    gz_offset += gz;
    delay(5);
  }
  gx_offset /= 200;
  gy_offset /= 200;
  gz_offset /= 200;
  Serial.println("✅ 校准完成");

  lastTime = millis();
  Serial.println("✅ MPU6050 + OLED 初始化完成");
}

// 封装函数 2：更新 MPU6050 数据
void update_mpu6050() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;
  float gyroX = (gx - gx_offset) / 131.0;
  float gyroY = (gy - gy_offset) / 131.0;
  float gyroZ = (gz - gz_offset) / 131.0;

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // 加速度角计算（减少耦合）
  float accPitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / M_PI;
  float accRoll  = atan2(-accX, accZ) * 180 / M_PI;

  // 互补滤波
  pitch = alpha * (pitch + gyroX * dt) + (1 - alpha) * accPitch;
  roll  = alpha * (roll + gyroY * dt) + (1 - alpha) * accRoll;

  // Yaw 积分 + 漂移衰减
  yaw += gyroZ * dt;
  yaw *= 0.9995;
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}
void oled_mpu6050() {
  unsigned long now = millis();
  if (now - lastDisplay < 100) return;
  lastDisplay = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("MPU6050 Orientation");

  display.setCursor(0, 16);
  display.printf("Pitch: %.1f deg\n", pitch);
  display.setCursor(0, 32);
  display.printf("Roll : %.1f deg\n", roll);
  display.setCursor(0, 48);
  display.printf("Yaw  : %.1f deg", yaw);
  display.display();
  delay(10);
}
//此函数检测是否掉下擂台，若检测到大角度波动则进入drop_rush模式
void check_mpu_drop(float limit){
  if((fabs(pitch) > limit || fabs(roll) > limit) && (mode=="autofight"||mode=="push")){
    mode="drop_rush";
    show_oled("drop_rush");
    dropRunning = true;
  }
}
void drop_rush_simple(){
  controlMotors(225, 1, 225, false, 225, 1, 225, true);
  delay(3000);
  mode="autofight";
  show_oled("autofight");
  pitch=0;
  roll=0;
}
//此函数在autofight模式下检测棋子，若检测到，则调整方向后进入push模式
void check_qizi() {

  // 一次性读取红外传感器数据
  int fl = getio_FL();
  int fr = getio_FR();
  int l  = getio_L();
  int r  = getio_R();

  // ====== 条件判断 ======

  // ✅ 当前面两个任意一个为 1，说明检测到旗子，停止
  if (fl == 1 || fr == 1) {
    mode="push";
    show_oled("push");
    return;
  }

  // ✅ 当前左侧传感器为 1
  if (l == 1) {
    rotateLeft(100);
    delay(200);
    mode="push";
    show_oled("push");
    return;
  }

  // ✅ 当前右侧传感器为 1
  if (r == 1) {
    rotateright(90);
    delay(200);
    mode="push";
    show_oled("push");
    return;
  }
}
void show_oled(String currentMode) {

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // 标题
  display.setCursor(0, 0);
  display.println("Current Mode:");

  // 模式内容（放大显示）
  display.setTextSize(2);
  display.setCursor(20, 30);
  display.println(currentMode);

  // 刷新屏幕
  display.display();
}
// ====================================================
// 启动检测函数：当右侧红外传感器为 1 时启动
// ====================================================
void start() //当右侧红外传感器检测到返回 1（表示右侧很近）时，程序才正式启动运行。
{
  Serial.println("⚙️ 等待启动信号（右侧传感器）...");
  delay(200);

  // 一直循环检测右侧传感器
  while (true) {
    int rightState = getio_R();  // 检测右侧红外距离
    if (rightState == 1) {       // 检测到物体靠近（启动信号）
      Serial.println("✅ 检测到启动信号！程序开始运行！");
      delay(500);                // 稍作延迟以稳定启动信号
      break;                     // 跳出循环，进入正式运行
    }
    delay(50); // 每 50ms 轮询一次，防止CPU占满
  }
}
void start_2() {
  Serial.println("⚙️ 等待启动信号（右侧传感器）...");
  int stableCount = 0;

  while (true) {
    if (getio_R() == 1) {
      stableCount++;
    } else {
      stableCount = 0;
    }

    // 连续5次检测到1（约0.25秒）才确认启动
    if (stableCount >= 5) {
      Serial.println("✅ 稳定启动信号确认，程序开始！");
      delay(500);
      break;
    }
    delay(50);
  }
}
// 调用时机：放在 loop() 里即可，函数内部自带刷新节拍控制
void oled_show_IR_io()
{
  static unsigned long lastRefresh = 0;
  const  unsigned long INTERVAL    = 100;          // 100 ms → 10 Hz

  if (millis() - lastRefresh < INTERVAL) return;   // 未到刷新时刻直接退出
  lastRefresh = millis();

  // 一次性读取 4 个 IO 值
  uint8_t fl = getio_FL();
  uint8_t fr = getio_FR();
  uint8_t l  = getio_L();
  uint8_t r  = getio_R();

  display.clearDisplay();
  display.setTextSize(2);                
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("FL:");
  display.println(fl);

  display.setCursor(64, 0);
  display.print("FR:");
  display.println(fr);

  display.setCursor(0, 32);
  display.print(" L:");
  display.println(l);

  display.setCursor(64, 32);
  display.print(" R:");
  display.println(r);

  display.display();                     
}
void setup() {
  Serial.begin(115200);  // 初始化串口通信
  setupArena();          // 初始化电机引脚
  start();
 rushToArena();//冲台
 //rotateLsft(80);
}
void loop() {
  update_mpu6050();
 if(mode == "autofight")//正常循台模式
   {
     autoFight();//防止掉台
     // check_mpu_drop(20);//检测是否掉台
     check_qizi();//检测是否有棋子
   }
  // // if(mode=="drop_rush")
  // // {
  // //   drop_rush_simple();//掉台调整方向后重新冲台
  // // }
   if(mode=="push")
   {
     autoFight();//正常循台
  // //  check_mpu_drop(20);//检测是否掉台
   }
  // oled_show_IR_io();
}