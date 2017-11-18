#include <math.h>
#include <Wire.h>
#include <EEPROM.h>
#include <IRremote.h>
#include <LiquidCrystal_I2C.h>

#include <TM1637Display.h>
#include <L3G.h>
#include "LedControl.h"
#include "Timer.h"
#include "MiniLFRV2.h"

#define FIRMWARE "Linefollow V2.7\r\n"
#define SETUPLEN 32

LiquidCrystal_I2C lcd(0x20, 16, 2); // set the LCD address to 0x20 for a 16 chars and 2 line display
L3G gyro; // gyro for calibrate forward movement

Timer timer;
IRsend irsend;
IRrecv irrecv(2);
MiniLFRV2 mini;

/*用于dc马达校准同步*/
static union {
  struct {
    unsigned int sign;//标致 读EEPROM
    float dcdiff;
    unsigned int irThreshold[5];
  } data;             //共用体结构体
  char buf[SETUPLEN];
} robotSetup;

decode_results results;  //红外解码 返回结果
uint32_t irdecoded = 0xffffffff;
/*读取五路寻迹传感器数值换算成二进制字符串形式*/
int echoTrace() {
  int ret;
  ret = mini.echoTrace();
  return ret;
}
/*PID*/
void pidWork() {
  mini.pidWork((robotSetup.data.dcdiff));
}
/*返回版本号*/
void echoVersion() {
  Serial.print("M0 ");
  Serial.print(FIRMWARE);
}

void gyroCalibrate() {
  int cnt = 0;
  int filteredCount = 0;
  float diff;
  float diffInte;
  float gz = 0;
  float gzStill = 0;
  long lastmillis;
  // 1. stop and init gyro
  //  delay(5000);
  mini.doCarMove(0, 0, robotSetup.data.dcdiff);
  if (!gyro.init()) {
    Serial.println("Gyro Init Fail");
    return;
  }
  gyro.enableDefault();
  // 2. get the still z-axis value
  while (cnt++ < 20) {
    gyro.read();
    gz = gz * 0.7 + (float)gyro.g.z * 0.3;
    delay(50);
  }
  // 3. output initial values
  gzStill = gz;
  Serial.print("Dc Diff: "); Serial.println(robotSetup.data.dcdiff);
  Serial.print("Init Gz: "); Serial.println(gzStill);
  lastmillis = millis();
  robotSetup.data.dcdiff = 1.0;
  diffInte = 0;
  mini.doCarMove(120, 0, robotSetup.data.dcdiff);
  // 4. start auto calibrate
  while (1) {
    gyro.read();
    gz = gz * 0.7 + (float)gyro.g.z * 0.3;
    delay(50);
    if (millis() - lastmillis > 100) {
      diff = gz - gzStill;
      lastmillis = millis();
      if (diff > 1000) {
        filteredCount = 0;
        diffInte += 0.02;
      } else if (diff < -1000) {
        filteredCount = 0;
        diffInte -= 0.02;
      } else {
        filteredCount++;
        if (filteredCount == 6) {
          robotSetup.data.dcdiff = 1.0 + diffInte;
          syncRobotSetup();
          mini.updateMotorSpeed(0, 0);
          Serial.println("M300");
          return;
        }
      }
      Serial.print(" Z: "); Serial.print(diff);
      Serial.print(" D: "); Serial.println(robotSetup.data.dcdiff);
      robotSetup.data.dcdiff = 1.0 + diffInte;
      mini.doCarMove(120, 0, robotSetup.data.dcdiff);
    }

    mini.updateMotorSpeed();
  }
}

/*获取每一位传感器数值*/
void doGetSensor(char * cmd) {
  int idx;
  sscanf(cmd, "%d\n", &idx);
  Serial.print("M1 "); Serial.print(idx);
  Serial.print(" "); Serial.println(mini.GetSensor(idx));
}
/*设置PID默认值*/
void doGetPid() {
  float Kp, Ki, Kd;
  mini.PidValue(&Kp, &Ki, &Kd);
  Serial.print("M2 ");
  Serial.print(Kp); Serial.print(" ");
  Serial.print(Ki); Serial.print(" ");
  Serial.println(Kd);
}
/*设置PID用户值*/
void doSetPid(char * cmd) {
  char * tmp;
  char * str;
  float p, i, d;
  str = tmp = cmd;
  while (str != NULL) {
    str = strtok_r(0, " ", &tmp);
    if (str[0] == 'P') {
      p = atof(str + 1);
    }
    else if (str[0] == 'I') {
      i = atof(str + 1);
    }
    else if (str[0] == 'D') {
      d = atof(str + 1);
    }
  }
  mini.SetPid(p, i, d);
}

/*获取默认域值*/
void doGetThreshold(char * cmd) {
  int m;
  sscanf(cmd, "%d\n", &m);
  Serial.print("M4 "); Serial.print(m);
  Serial.print(" ");
  Serial.println(mini.GetThreshold(m));
}
/*设置用户域值*/
void doSetThreshold(char * cmd) {
  int idx, val;
  sscanf(cmd, "%d %d\n", &idx, &val);
  mini.SetThreshold(idx, val);
  robotSetup.data.irThreshold[idx] = val;
  syncRobotSetup();
}
/*设置N20速度*/
void doDcSpeed(char * cmd) {
  int idx, speed;
  sscanf(cmd, "%d %d\n", &idx, &speed);
  mini.SetDCSpeed(idx, speed, robotSetup.data.dcdiff);
  //Serial.print(idx);Serial.print(",");Serial.println(speed);
}

void doCarMove(char * cmd) {
  int fw, lr;
  sscanf(cmd, "%d %d\n", &fw, &lr);
  mini.doCarMove(fw, lr, robotSetup.data.dcdiff);
  Serial.println("M201");
}

void doEye(char * cmd) {
  int left, right;
  sscanf(cmd, "%d %d\n", &left, &right);
  mini.doEye(left, right);
  Serial.println("M6");
}

void doDistance() {
  float distance;
  distance = mini.doDistance();
  Serial.print("M7 ");
  Serial.println(distance);
}

void doBattery() {
  float v ;
  v = mini.doBattery();
  Serial.print("M8 ");
  Serial.println(v);
}
/*底部RGB*/
void doHoverLight(char * cmd) {
  int pix, r, g, b;
  sscanf(cmd, "%d %d %d %d\n", &pix, &r, &g, &b);
  mini.HoverLight(pix, r, g, b);
  Serial.println("M13");
}

void doBuzzer(char * cmd) {
  int freq, t;
  sscanf(cmd, "%d %d\n", &freq, &t);
  Serial.println("M18");
  mini.buzz(freq, t);

}

void doButton1() {
  Serial.print("M9 "); Serial.println(mini.GetButton1());
}

void doButton2() {
  Serial.print("M10 "); Serial.println(mini.GetButton2());
}

void doInfraSend(char * cmd) {
  int n;
  sscanf(cmd, "%x\n", &n);
  irsend.sendNEC(n, 32);
  Serial.println("M12");
  irrecv.enableIRIn();
}

void doLCD(char * cmd) {
  int i = 0;
  lcd.clear();
  while (cmd[i] != '\0' && cmd[i] != '\r' && cmd[i] != '\n') {
    lcd.write(cmd[i]);
    i++;
  }
  Serial.println("M15");
}

void doFrontRGB(char * cmd) {
  int pix, r, g, b;
  sscanf(cmd, "%d %d %d %d\n", &pix, &r, &g, &b);
  mini.FrontRGB(pix, r, g, b);
  Serial.println("M16");
}

void doWriteDcAdjust(char *cmd)
{
  robotSetup.data.dcdiff = atof(cmd);
  syncRobotSetup();
}

void doJoystick(char * cmd) {
  int posX, posY, fw, lr;
  sscanf(cmd, "%d %d\n", &posX, &posY );
  /* // too costly
    float angle = atan((float)posY/(float)posX)/PI*360;
    float vec = sqrt(sq(posX)+sq(posY));
    Serial.print("joy: ");Serial.print(angle);Serial.print(" ");Serial.println(vec);
  */
  fw = posY * 2;
  lr = posX;
  mini.doCarMove(fw, lr, robotSetup.data.dcdiff);
}

void doReadDcAdjust()
{
  Serial.print("M210 ");
  Serial.println(robotSetup.data.dcdiff);
}

void initRobotSetup() {
  int i;
  for (i = 0; i < SETUPLEN; i++) {
    robotSetup.buf[i] = EEPROM.read(i);
  }
  if (robotSetup.data.sign != 1223) {
    Serial.println("Init robot setup");
    memset(robotSetup.buf, 0, 16);
    robotSetup.data.sign = 1223;
    robotSetup.data.dcdiff = 1.0;
    for (int idx = 0; idx < 5; idx++) {
      robotSetup.data.irThreshold[idx] = 100;
    }
    syncRobotSetup();
  }
  for (int idx = 0; idx < 5; idx++) {
    mini.SetThreshold(idx, robotSetup.data.irThreshold[idx]);
  }
}

void syncRobotSetup()
{
  int i;
  for (i = 0; i < SETUPLEN; i++) {
    //Serial.print(robotSetup.buf[i] & 0xFF, HEX);Serial.print(" ");
    EEPROM.write(i, robotSetup.buf[i]);
  }
}
void setThresholdAll(char * cmd)
{
  int num;
  sscanf(cmd, "%d\n", &num);
  for (int idx = 0; idx < 5; idx++)
  {
    mini.SetThreshold(idx, 150);
    robotSetup.data.irThreshold[idx] = num;
  }
}
int ADValMax[5];
int ADValMin[5];
int ADValAve[5];
int AD[5] = { A3, A2, A1, A0, A6};
void doCalibrationThreshold()
{
  int temp;//临时变量 确保if比较完读取到的值不会变化

  for (int idx = 0; idx < 5; idx++)
  {
    temp = analogRead(AD[idx]);
    if ( temp < ADValMin[idx])
    {
      ADValMin[idx] = temp;
    }
    if ( temp > ADValMax[idx])
    {
      ADValMax[idx] = temp;
    }
  }
  //    for(int i=0;i<5;i++)
  //    {
  //    Serial.print(ADVal[i]);
  //    Serial.print(" ");
  //    }
  //    Serial.println("");
}
int8_t dcMark = 0;          // 每自加一 时间过了450ms
bool turnMark = 0;          //小车每450ms进入一次 这个标签决定小车是否执行旋转程序
void doCarTurn()
{
  if (!turnMark)
    return;
  switch (dcMark)
  {
    case 1:                    //左90
      for (int idx = 0; idx < 5; idx++) //转之前读一次巡线传感器初始值    在这之前 储存的传感器值无效
      {
        ADValMax[idx] = ADValMin[idx] = analogRead(AD[idx]);
      }
      mini.SetDCSpeed(1, -50, robotSetup.data.dcdiff);
      mini.SetDCSpeed(2, 50, robotSetup.data.dcdiff);
      break;
    case 3:                    //右90
      mini.SetDCSpeed(1, 50, robotSetup.data.dcdiff);
      mini.SetDCSpeed(2, -50, robotSetup.data.dcdiff);
      break;
    case 5:                    //右90
      mini.SetDCSpeed(1, 50, robotSetup.data.dcdiff);
      mini.SetDCSpeed(2, -50, robotSetup.data.dcdiff);
      break;
    case 7:                    //左90
      mini.SetDCSpeed(1, -50, robotSetup.data.dcdiff);
      mini.SetDCSpeed(2, 50, robotSetup.data.dcdiff);
      break;
    default:                   //每转一次 停450ms
      mini.SetDCSpeed(0, 0, robotSetup.data.dcdiff);
      break;
  }
  dcMark++;
  if (dcMark > 8)                   //小车旋转结束
  {
    turnMark = 0;                 //小车旋转结束
    dcMark = 0;
    Serial.println("Result");    //EEPROM写阈值

    for (int idx = 0; idx < 5; idx++)
    {
      ADValAve[idx] = (ADValMax[idx] + ADValMin[idx]) / 2;
      mini.SetThreshold(idx, ADValAve[idx]);
      robotSetup.data.irThreshold[idx] = ADValAve[idx];
      Serial.print(robotSetup.data.irThreshold[idx]);
    }
    syncRobotSetup();
    mini.setMode(0);               //开始巡线
  }
}

void doRingRGB(char * cmd) {
  int pix, r, g, b;
  sscanf(cmd, "%d %d %d %d\n", &pix, &r, &g, &b);
  mini.RingRGB(pix, r, g, b);
  Serial.println("M217");
}

bool irEnable = 0;

void doInfraRead(){
  if (!irEnable)
  {
    irrecv.enableIRIn();
    irEnable = 1;
  }
  Serial.print("M11 ");
  Serial.println(irdecoded, HEX);
  irdecoded = -1; 
}

void parseCode(char * cmd) {
  int code;
  char * tmp;
  code = atoi(cmd);
  cmd = strtok_r(cmd, " ", &tmp);

  switch (code) {
    case 0:
      echoVersion();
      mini._codingMode = 1;
      break;
    case 1:
      doGetSensor(tmp);
      break;
    case 2:
      doGetPid();
      break;
    case 3:
      doSetPid(tmp);
      break;
    case 4:
      doGetThreshold(tmp);
      break;
    case 5:
      doSetThreshold(tmp);
      break;
    // peripherals control
    case 6: // front eye command
      doEye(tmp);
      break;
    case 7: // ultrasonci sensor
      doDistance();
      break;
    case 8: // battery
      doBattery();
      break;
    case 9: // button 1 status
      doButton1();
      break;
    case 10: // button 2 status
      doButton2();
      break;
    case 11: // irvalue
      doInfraRead();
      break;
    case 12: // ir send
      doInfraSend(tmp);
      break;
    case 13: // hover light
      doHoverLight(tmp);
      break;
    case 15: // lcd
      doLCD(tmp);
      break;
    case 16: // front rgb
      doFrontRGB(tmp);
      break;
    case 18: // buzzer
      doBuzzer(tmp);
      break;
    case 200:
      doDcSpeed(tmp);
      break;
    case 201:
      doCarMove(tmp);
      break;
    case 209:
      doWriteDcAdjust(tmp);
      break;
    case 210:
      doReadDcAdjust();
      break;
    case 214:
      doJoystick(tmp);
      break;
    case 215:
      setThresholdAll(tmp);
      break;
    case 217:
      doRingRGB(tmp);
      break;
    case 300:
      gyroCalibrate();
      break;
    case 999:
      asm volatile ("  jmp 0");
      break;
    default:
      break;
  }

}

void parseCmd(char * cmd) {
  if (cmd[0] == 'M') {
    parseCode(cmd + 1);
  }
}
void setup() {
  digitalWrite(EYE_LEFT, 1);
  digitalWrite(EYE_RIGHT, 1);
  Serial.begin(115200);
  echoVersion();
  initRobotSetup();
  lcd.init();
  lcd.backlight();
  lcd.print("Hello MiniLFR\0");
  timer.every(5, pidWork);
  timer.every(450, doCarTurn);
  // irrecv.enableIRIn();
}

char buf[64];
int8_t bufindex;


void loop() {
  // pos = echoTrace();
  timer.update();
  if (mini.mode() == 1)      //避障模式
  {
    mini.avoid_mode(robotSetup.data.dcdiff);
    return ;
  }
  if (mini.mode() == 2)      //校准阈值模式
  {
    turnMark = 1;            //小车旋转
    doCalibrationThreshold();//读传感器值 储存
    return ;
  }




  while (Serial.available()) {
    char c = Serial.read();
    buf[bufindex++] = c;
    if (c == '\n') {
      buf[bufindex] = '\0';
      parseCmd(buf);
      memset(buf, 0, 64);
      bufindex = 0;
    }
    if (bufindex >= 64) {
      bufindex = 0;
    }
  }

  if (irrecv.decode(&results))
  {
    if (results.value != 0xFFFFFFFF)
    {
      irdecoded = results.value;
      
    }
    irrecv.resume(); // Receive the next value
  }
}


