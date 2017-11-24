#include <Arduino.h>
#include <math.h>
#include "EEPROM.h"
#include "Adafruit_LEDBackpackSw.h"
#include "Adafruit_NeoPixel.h"
#include "MiniLFRV2.h"
#include "IRremote.h"

#define SETUPLEN 32

Adafruit_NeoPixel hoverRgb(2);
Adafruit_NeoPixel headRgb(2);
// start from up-left
Kittenbot_16x8matrix ledMat = Kittenbot_16x8matrix();

decode_results irresult;
uint32_t irdecoded = 0xffffffff;
IRsend irsend;
IRrecv irrecv(2);

int adval[5];

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
  ledMat.begin(0x70);
  hoverRgbShow(0,0,0,0);
  eyeLedSet(0,0);
}

void MiniLFRV2::loadSetup(){
  int i;
  for (i = 0; i < SETUPLEN; i++) {
    robotSetup.buf[i] = EEPROM.read(i);
  }
  if (robotSetup.data.sign != 1223) {
    //Serial.println("Init robot setup");
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
    analogWrite(PIN_M2B, -spdR);
    analogWrite(PIN_M2A, 0);
  }else{
    analogWrite(PIN_M2B, 0);
    analogWrite(PIN_M2A, spdR);
  }
}

void MiniLFRV2::speedSet(int spdL, int spdR, int duration){
  speedSet(spdL, spdR);
  delay(duration);
  speedSet(0,0);
}

void MiniLFRV2::stopMotor(){
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
  digitalWrite(EYE_LEFT, !left);
  digitalWrite(EYE_RIGHT, !right);
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

void MiniLFRV2::setRgbBrightness(int value){
  hoverRgb.setBrightness(value);
  headRgb.setBrightness(value);
}

void MiniLFRV2::hoverRgbShow(int pix, int r, int g, int b){
  if (pix == 0) {
    for (int i = 0;i < 2;i++) {
      hoverRgb.setPixelColor(i, r, g, b);
    }
  }
  else {
    hoverRgb.setPixelColor(pix - 1, r, g, b);
  }
  hoverRgb.show();
}

void MiniLFRV2::headRgbShow(int pix, int r, int g, int b){
  if (pix == 0) {
    for (int i = 0;i < 2;i++) {
      headRgb.setPixelColor(i, r, g, b);
    }
  }
  else {
    headRgb.setPixelColor(pix - 1, r, g, b);
  }
  headRgb.show();
}

int MiniLFRV2::getSensor(int index){
  return analogRead(AD[index]);
}

void MiniLFRV2::setSensorThreshold(int index, int value){
  robotSetup.data.irThreshold[index] = value;
}

int MiniLFRV2::getSensorThreshold(int index){
  return robotSetup.data.irThreshold[index];
}

void MiniLFRV2::matrixShow(uint8_t * data){
  ledMat.clear();
  ledMat.drawBitmap(0, 0, data, 8, 8, LED_ON);
  ledMat.writeDisplay();
}

void MiniLFRV2::updatePid(float p, float i, float d){
  Kp = p;
  Ki = i;
  Kd = d;
}

int MiniLFRV2::getTrace(){
  int ret = 0;
  for (int i = 0; i < 5; i++) {
    adval[i] = analogRead(AD[i]);
    if (adval[i] < robotSetup.data.irThreshold[i]) ret += (0x1 << i);
  }
  return ret;  
}

float MiniLFRV2::calcPid(float input) {
  float errorDiff;
  float output;
  error = error * 0.7 + input * 0.3; // filter
                     //error = input;
  errorDiff = error - errorLast;
  erroInte = constrain(erroInte + error, -50, 50);
  output = Kp * error + Ki * erroInte + Kd * errorDiff;

  errorLast = error;

  return output;
}

int MiniLFRV2::pidLoop(){
  int spdL, spdR;
  int bias;
  int pos = getTrace();
  switch (pos) {
    case B00000:
      outlineCnt++;
      break;
    case B11111:
      outlineCnt++;
      break;
    case B00010:
    case B00110:
      outlineCnt = 0;
      bias = 1;
      break;
    case B00001:
    case B00011:
      outlineCnt = 0;
      bias = 2;
      break;
    case B00100:
      outlineCnt = 0;
      bias = 0;
      break;
    case B01000:
    case B01100:
      outlineCnt = 0;
      bias = -1;
      break;
    case B10000:
    case B11000:
      outlineCnt = 0;
      bias = -2;
      break;
    default:
      outlineCnt++;
      break;
  }
  if (outlineCnt > 100) {
    speedSet(0,0);
    return -1;
  }else{
    float ff = 150; //150
    float ctrl = -calcPid(bias);
    spdL = ff + ctrl;
    spdR = (ff - ctrl);
    speedSet(spdL, spdR);
    return 0;
  }
  
}

void MiniLFRV2::startLineFollow(){
  outlineCnt = 0;
  erroInte = errorLast = error = 0;
}

void MiniLFRV2::loop(){
  if (irrecv.decode(&irresult))
  {
    if (irresult.value != 0xFFFFFFFF)
    {
      irdecoded = irresult.value;
    }
    irrecv.resume(); // Receive the next value
  }
}

void MiniLFRV2::infraSend(int hex){
  irsend.sendNEC(hex, 32);
  delay(5);
  irrecv.enableIRIn();
}

uint32_t MiniLFRV2::infraReceive(){
  uint32_t ret;
  ret = irdecoded;
  irdecoded = -1;   
  return ret;
}

// note map in ascii format a to g and octave 4
const int noteMap[] = {440, 494, 262, 294, 330, 349, 392};

void MiniLFRV2::playNote(int note, int clap){
  int freq = 440.0f*pow(2, float(note-69)/12);
  int duration = clap * 125;
  //Serial.println("freq "+String(freq));
  buzz(freq, duration, duration);
}

void MiniLFRV2::playMusic(const char * notes){
  int freq;
  int len = strlen(notes);
  int octave = 4;
  int duration = 500;
  for(int i=0;i<len;i++){
    if(notes[i]>='a' && notes[i]<='g'){
      freq = noteMap[notes[i]-'a'];
      //Serial.println("freq:"+String(freq));
    }else if(notes[i]=='r'){
      freq = 0;
    }else if(notes[i]>='2' && notes[i]<='6'){
      octave = notes[i] - '0';
      //Serial.println("octave:"+String(octave));
    }else if(notes[i]==':'){
      i++;
      duration = (notes[i] - '0')*125;
      //Serial.println("duration:"+String(duration));
    }else if(notes[i]==' '){ // play until we meet a space
      freq *= pow(2, octave-4);
      //Serial.println("play:"+String(freq)+", "+String(duration)+", "+String(octave));
      buzz(freq, duration, duration);
    }
  }
}




