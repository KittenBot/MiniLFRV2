#include <Wire.h>
#include "MiniLFRV2.h"
#include "Timer.h"
#include "L3G.h"

#define FIRMWARE "Linefollow V3.0\r\n"

const char ode[] = "e4 e f g g f e d c c d e e:6 d:2 d:8 e:4 e f g g f e d c c d e d:6 c:2 c:8 ";
const char birthday[] = "c4:3 c:1 d:4 c:4 f e:8 c:3 c:3 d:4 c:4 g f:8 c:3 c:1 c5:4 a4 f e d a:3 a:1 a:4 f g f:8 ";
const char wedding[] = "c4:4 f:3 f:1 f:8 c:4 g:3 e:1 f:8 c:4 f:3 a:1 c5:4 a4:3 f:1 f:4 e:3 f:1 g:8 ";
const char powerup[] = "g4:1 c5 e g:2 e:1 g:3 ";
const char powerdown[] = "g5:1 d c g4:2 b:1 c5:3 ";
const char bdding[] = "b5:1 e6:3 ";
const char baddy[] = "c3:3 r d:2 d r c r f:8 ";


enum mode {
  IDLE,
  CODING,
  LINEFOLLOW,
  OBJECTAVOID
};

int pidTh;
int mode = IDLE;
Timer timer;
MiniLFRV2 mini;
L3G gyro; // gyro for calibrate forward movement


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
  mini.stopMotor();
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
  //Serial.print("Dc Diff: "); Serial.println(mini.motorDiffGet());
  //Serial.print("Init Gz: "); Serial.println(gzStill);
  lastmillis = millis();
  mini.motorDiffSet(1.0);;
  diffInte = 0;
  mini.speedSet(120,120);
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
          mini.motorDiffSet(1.0 + diffInte);
          mini.syncSetup();
          mini.stopMotor();
          Serial.println("M300");
          return;
        }
      }
      //Serial.print(" Z: "); Serial.print(diff);
      //Serial.print(" D: "); Serial.println(mini.motorDiffGet());
      mini.motorDiffSet(1.0 + diffInte);
      mini.speedSet(120,120);
    }
  }
}


void pidWork() {
  int ret = mini.pidLoop();
  if(ret == -1){
    mini.playMusic(powerdown);
    mode = IDLE;  
  }
}

void echoVersion() {
  Serial.print("M0 ");
  Serial.print(FIRMWARE);
}

void doGetSensor(char * cmd) {
  int idx;
  sscanf(cmd, "%d\n", &idx);
  Serial.print("M1 "); Serial.print(idx);
  Serial.print(" "); Serial.println(mini.getSensor(idx));
}

void doGetPid() {
  Serial.print("M2 ");
  Serial.print(mini.Kp); Serial.print(" ");
  Serial.print(mini.Ki); Serial.print(" ");
  Serial.println(mini.Kd);
}

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
  mini.updatePid(p, i, d);
}

void doGetThreshold(char * cmd) {
  int m;
  sscanf(cmd, "%d\n", &m);
  Serial.print("M4 "); Serial.print(m);
  Serial.print(" ");
  Serial.println(mini.getSensorThreshold(m));
}

void doSetThreshold(char * cmd) {
  int idx, val;
  sscanf(cmd, "%d %d\n", &idx, &val);
  mini.setSensorThreshold(idx, val);
  mini.syncSetup();
}

void doEye(char * cmd) {
  int left, right;
  sscanf(cmd, "%d %d\n", &left, &right);
  mini.eyeLedSet(left, right);
}

void doDistance() {
  float distance;
  distance = mini.distance();
  Serial.print("M7 ");
  Serial.println(distance);
}

void doBattery() {
  float v ;
  v = mini.batteryVoltage();
  Serial.print("M8 ");
  Serial.println(v);
}

void doButton1() {
  Serial.print("M9 "); Serial.println(mini.buttonGet(1));
}

void doButton2() {
  Serial.print("M10 "); Serial.println(mini.buttonGet(2));
}

void doInfraRead(){
  Serial.print("M11 ");
  Serial.println(mini.infraReceive(), HEX);
}

void doInfraSend(char * cmd) {
  int n;
  sscanf(cmd, "%x\n", &n);
  mini.infraSend(n);
}

void doHoverLight(char * cmd) {
  int pix, r, g, b;
  sscanf(cmd, "%d %d %d %d\n", &pix, &r, &g, &b);
  mini.hoverRgbShow(pix, r, g, b);
}

void doFrontRGB(char * cmd) {
  int pix, r, g, b;
  sscanf(cmd, "%d %d %d %d\n", &pix, &r, &g, &b);
  mini.headRgbShow(pix, r, g, b);
}

void doBuzzer(char * cmd) {
  int freq, t;
  sscanf(cmd, "%d %d\n", &freq, &t);
  mini.buzz(freq, t);
}

void doNote(char * cmd){
  int note, clap;
  sscanf(cmd, "%d %d\n", &note, &clap);
  mini.playNote(note, clap);
}

void doDcSpeed(char * cmd) {
  int spdl, spdr;
  sscanf(cmd, "%d %d\n", &spdl, &spdr);
  mini.speedSet(spdl, spdr);
}

void setMotorDiff(char *cmd)
{
  mini.motorDiffSet(atof(cmd));
  mini.syncSetup();
}

void getMotorDiff()
{
  Serial.print("M210 ");
  Serial.println(mini.motorDiffGet());
}

void doJoystick(char * cmd) {
  int posX, posY, fw, lr;
  sscanf(cmd, "%d %d\n", &posX, &posY );
  fw = posY * 2;
  lr = posX;
  mini.speedSet(fw + lr, -(fw - lr));
}

void setThresholdAll(char * cmd)
{
  int num;
  sscanf(cmd, "%d\n", &num);
  for (int idx = 0; idx < 5; idx++)
  {
    mini.setSensorThreshold(idx, num);
  }
}

void parseCode(char * cmd) {
  int code;
  char * tmp;
  code = atoi(cmd);
  cmd = strtok_r(cmd, " ", &tmp);

  switch (code) {
    case 0:
      echoVersion();
      mode = CODING;
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
      //doLCD(tmp);
      break;
    case 16: // front rgb
      doFrontRGB(tmp);
      break;
    case 18: // buzzer
      doBuzzer(tmp);
      break;
    case 19: // play note
      doNote(tmp);
      break;
    case 200:
      doDcSpeed(tmp);
      break;
    case 209:
      setMotorDiff(tmp);
      break;
    case 210:
      getMotorDiff();
      break;
    case 214:
      doJoystick(tmp);
      break;
    case 215:
      setThresholdAll(tmp);
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

void setup(){
  Wire.begin();
  Serial.begin(115200);
  mini.init();
  pidTh = timer.every(5, pidWork);
  echoVersion();
}

void avoidLoop(){
  float temp = mini.distance();
  if (temp < 10){
    mini.stopMotor();
    delay(20);
    mini.speedSet(-100, -100, 200);
  }else{
    mini.speedSet(random(50,150), random(50,150));
  }
  delay(20);
}

char buf[64];
int8_t bufindex;

void loop(){
  if(mode == IDLE){
    if(mini.buttonGet(1) == 0){
      mini.playMusic(powerup);
      if(mini.buttonGet(1) == 0){
        // todo: go threshold calibrate  
      }else{
        mini.startLineFollow();
        mode = LINEFOLLOW;
      }
    }
    if(mini.buttonGet(2) == 0){
      mini.playMusic(bdding);
      mode = OBJECTAVOID;
    }
  }else if(mode == LINEFOLLOW){
    timer.update();  
  }else if(mode == OBJECTAVOID){
    avoidLoop();
    if(mini.buttonGet(2) == 0){
      mode = IDLE;
      mini.playMusic(baddy);
      mini.stopMotor();
    }
  }else{
    mini.loop();  
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
}


