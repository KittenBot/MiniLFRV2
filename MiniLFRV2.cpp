#include "EEPROM.h"
#include "Adafruit_NeoPixel.h"
#include "MiniLFRV2.h"

#define SETUPLEN 32

Adafruit_NeoPixel hoverRgb(2);
Adafruit_NeoPixel headRgb(2);

static union {
  struct {
    unsigned int sign;
    float dcdiff;
    unsigned int irThreshold[5];
  } data;
  char buf[SETUPLEN];
} robotSetup;


MiniLFRV2::MiniLFRV2() {

}

void MiniLFRV2::init() {
  analogWrite(PIN_M1A, 0);
  analogWrite(PIN_M1B, 0);
  analogWrite(PIN_M2A, 0);
  analogWrite(PIN_M2B, 0);
  // eye led
  pinMode(EYE_LEFT, OUTPUT);
  pinMode(EYE_RIGHT, OUTPUT);
  //button
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  // rgb
  hoverRgb.begin();
  hoverRgb.setPin(PIN_RGB);
  headRgb.begin();
  headRgb.setPin(EYE_RIGHT);
  loadSetup();
}

void MiniLFRV2::loadSetup(){
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
      robotSetup.data.irThreshold[idx] = 120;
    }
    syncSetup();
  }
}

void MiniLFRV2::syncSetup(){
  int i;
  for (i = 0; i < SETUPLEN; i++) {
    EEPROM.write(i, robotSetup.buf[i]);
  }
}

void MiniLFRV2::motorDiffSet(float diff){
  robotSetup.data.dcdiff = diff;
}

float MiniLFRV2::motorDiffGet(){
  return robotSetup.data.dcdiff;
}

void MiniLFRV2::speedSet(int spdL, int spdR) {
  spdR = spdR*robotSetup.data.dcdiff;
  if(spdL < 0) {
    analogWrite(PIN_M1B, 0);
    analogWrite(PIN_M1A, -spdL);
  }else{
    analogWrite(PIN_M1B, spdL);
    analogWrite(PIN_M1A, 0);
  }
  if(spdR < 0) {
    analogWrite(PIN_M2B, 0);
    analogWrite(PIN_M2A, -spdR);
  }else{
    analogWrite(PIN_M2B, spdR);
    analogWrite(PIN_M2A, 0);
  }
}

void MiniLFRV2::speedSet(int spdl, int spdr, int t) {
  speedSet(spdl, spdr);
  delay(t);
  speedSet(0,0);
}

void MiniLFRV2::stop(){
  speedSet(0,0);
}

int MiniLFRV2::buttonGet(int btn)
{
  if(btn == 1){
    return digitalRead(PIN_BTN1);
  }else if(btn == 2){
    return digitalRead(PIN_BTN2);
  }
  return -1;
}

void MiniLFRV2::eyeLedSet(int left, int right){
  pinMode(EYE_LEFT, OUTPUT);
  pinMode(EYE_RIGHT, OUTPUT);
  digitalWrite(EYE_LEFT, left);
  digitalWrite(EYE_RIGHT, right);
}

float MiniLFRV2::distance(){
  float distance;
  unsigned int temp;
  pinMode(EYE_LEFT, OUTPUT);
  digitalWrite(EYE_LEFT, HIGH);
  delayMicroseconds(10);
  pinMode(EYE_LEFT, INPUT);
  temp = pulseIn(EYE_LEFT, HIGH);
  distance = (float)temp / 58.2;
  return distance;
}

float MiniLFRV2::batteryVoltage()
{
  int a = analogRead(A7);
  float v = float(a) / 1024.0 * 5.2;
  return v;
}

void MiniLFRV2::buzz(int freq, int duration){
  tone(PIN_BUZZ, freq, duration);
}

void MiniLFRV2::buzz(int freq, int duration, int delayms){
  buzz(freq, duration);
  delay(delayms);
}









