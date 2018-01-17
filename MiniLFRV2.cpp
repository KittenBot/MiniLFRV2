#include <Arduino.h>
#include <math.h>
#include "EEPROM.h"
#include "Adafruit_LEDBackpackSw.h"
#include "Adafruit_NeoPixel.h"
#include "MiniLFRV2.h"
#include "IRremote.h"

#define SETUPLEN 48

Adafruit_NeoPixel hoverRgb(2);
Adafruit_NeoPixel headRgb(2);
Adafruit_NeoPixel ringRgb(16);

// start from up-left
Kittenbot_16x8matrix ledMat = Kittenbot_16x8matrix();

decode_results irresult;
uint32_t irdecoded = 0xffffffff;
IRsend irsend;
IRrecv irrecv(2);

static union {
  struct {
    unsigned int sign;
    float dcdiff;
    unsigned int irThreshold[5];
    unsigned int irMax[5];
    unsigned int irMin[5];
  } data;
  char buf[SETUPLEN];
} robotSetup;


struct CallbackDict{
	uint32_t key;
	void (*fun)();
};

CallbackDict callbackAry[8];

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
  hoverRgb.setBrightness(100);
  headRgb.begin();
  headRgb.setPin(EYE_RIGHT);
  headRgb.setBrightness(100);
  ringRgb.begin();
  ringRgb.setPin(EYE_LEFT);
  ringRgb.setBrightness(100);
  loadSetup();
  hoverRgbShow(0,0,0,0);
  spotlightSet(0,0);
  irrecv.enableIRIn();
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
    return !digitalRead(PIN_BTN1);
  }else if(btn == 2){
    return !digitalRead(PIN_BTN2);
  }
  return -1;
}

void MiniLFRV2::spotlightSet(int left, int right){
  pinMode(EYE_LEFT, OUTPUT);
  pinMode(EYE_RIGHT, OUTPUT);
  // inverse left right
  digitalWrite(EYE_RIGHT, !left);
  digitalWrite(EYE_LEFT, !right);
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
  if(distance == 0){
	  distance = 999;
  }
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
  ringRgb.setBrightness(value);
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

void MiniLFRV2::ringRgbShow(int pix, int r, int g, int b){
  if (pix == 0) {
    for (int i = 0;i < 16;i++) {
      ringRgb.setPixelColor(i, r, g, b);
    }
  }
  else {
    ringRgb.setPixelColor(pix - 1, r, g, b);
  }
  ringRgb.show();
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

void MiniLFRV2::setSensorMax(int index, int value){
  robotSetup.data.irMax[index] = value;  
}
int MiniLFRV2::getSensorMax(int index){
  return robotSetup.data.irMax[index];
}
void MiniLFRV2::setSensorMin(int index, int value){
  robotSetup.data.irMin[index] = value;  
}
int MiniLFRV2::getSensorMin(int index){
  return robotSetup.data.irMin[index];
}

uint16_t mat[8];
char tmp[5] = "0000";
void MiniLFRV2::matrixShow(const char * cmd){
  int index = 0;
  //memset(mat, 0, sizeof(mat));
  //Serial.println(cmd);
  for(int i=0;i<32;i+=4){
    tmp[2] = cmd[i];
    tmp[3] = cmd[i+1];
    tmp[0] = cmd[i+2];
    tmp[1] = cmd[i+3];
	//Serial.println(cmd+i);
	//Serial.println(tmp);
    mat[index] = strtol(tmp, NULL, 16);
    //Serial.print(String(mat[index], 16)+" ");
    index++;
  }
  //Serial.println("M21");
  ledMat.begin(0x70);
  ledMat.clear();
  ledMat.drawBitmap(0, 0, (uint8_t *)mat, 16, 8, LED_ON);
  ledMat.writeDisplay();
}

void MiniLFRV2::matrixShowString(const char * str){
  ledMat.begin(0x70);
  ledMat.setTextSize(0);
  ledMat.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  ledMat.setTextColor(LED_ON);
  int offset = -(strlen(str)+1)*5;
  for (int x=15; x>=offset; x--) {
    ledMat.clear();
    ledMat.setCursor(x,0);
    ledMat.print(str);
    ledMat.writeDisplay();
    delay(100);
  }
}

void MiniLFRV2::updatePid(float p, float i, float d){
  Kp = p;
  Ki = i;
  Kd = d;
}

uint8_t MiniLFRV2::getTrace(int * bias){
  uint8_t trace = 0;
  int tr[5];
  int adval[5];
  long avg = 0;
  int sum = 0;
  int offset;
  for (int i = 0; i < 5; i++) {
	adval[i] = analogRead(AD[i]);
	tr[i] = 100 - map(adval[i], robotSetup.data.irMin[i], robotSetup.data.irMax[i], 0, 100);
	avg += ((long)tr[i]*100*i);
	sum += tr[i];
	if(adval[i]<robotSetup.data.irThreshold[i]){
		trace |= (1<<i);
	}
  }
  
  offset = avg/sum - 200;
  if(trace == 0){ // out off line
	if(bias<0){
		offset = 0;
	} else {
		offset = 200;
	}
  }
  *bias = offset;
  return trace;
}

float MiniLFRV2::calcPid(float input) {
  float errorDiff;
  float output;
  error = error * 0.5 + input * 0.5; // filter
  //error = input;
  errorDiff = error - errorLast;
  erroInte = constrain(erroInte + error, -50, 50);
  output = Kp * error + Ki * erroInte + Kd * errorDiff;

  errorLast = error;
  output = constrain(output, -150, 150);
  return output;
}

int MiniLFRV2::findLine(int dir){
	uint8_t tr;
	int temp;
	if(dir == 0){
		speedSet(-60, -60);
	}else if(dir == 1){
		speedSet(-60, 60);
	}else if(dir == 2){
		speedSet(60, -60);
	}
	bool online = false;
	while(!online){
		tr = getTrace(&temp);
		online = (tr & 0b00001110);
		//delay(2);
	}
	return 0;
}

int MiniLFRV2::lineFollow(){
  int spdL, spdR;
  static int bias;
  uint8_t trace = getTrace(&bias);
  if (trace == 0 || trace == 0b00011111) {
	outlineCnt++;
	findLine(0);
  }else if(trace == 0b00000001){
	findLine(1);
  }else if(trace == 0b00010000){
	findLine(2);  
  }else{
	spdL = spdR = 150;
    float ctrl = calcPid(bias);
	if(ctrl>0){
		spdR -= ctrl;
	}else{
		spdL += ctrl;
	}
    speedSet(spdL, spdR);
	outlineCnt=0;
    return 0;
  }
  //Serial.println(String(trace, BIN)+" "+String(outlineCnt));
  if(outlineCnt>10){
	speedSet(0, 0);
	return -1;
  }

}

void MiniLFRV2::startLineFollow(){
  outlineCnt = 0;
  erroInte = errorLast = error = 0;
}

int MiniLFRV2::registerCallback(uint32_t key, void * fun()){
	for(int i=0;i<8;i++){
		if(callbackAry[i].key == 0){
			callbackAry[i].key = key;
			callbackAry[i].fun = fun;
			return i;
		}
	}
	return -1;
}

bool MiniLFRV2::loopCallback(uint32_t key){
	for(int i=0;i<8;i++){
		if(callbackAry[i].key == key){
			callbackAry[i].fun();
			return true;
		}
	}
	return false;
}

void MiniLFRV2::loop(){
  if (irrecv.decode(&irresult))
  {
    if (irresult.value != 0xFFFFFFFF)
    {
      irdecoded = irresult.value;
	  // todo: add callback entry for button and others  
	  if(!loopCallback(irdecoded)){
		Serial.println("TRIG 1 "+String(irdecoded, HEX));  
	  }
    }
    irrecv.resume(); // Receive the next value
  }
}

void MiniLFRV2::infraSend(int hex){
  irsend.sendNEC(hex, 32);
  delay(5);
  irrecv.enableIRIn();
}

void MiniLFRV2::extIo(int d12, int d10, int t){
  pinMode(EYE_LEFT, OUTPUT);
  pinMode(EYE_RIGHT, OUTPUT);
  digitalWrite(EYE_LEFT, d12);
  digitalWrite(EYE_RIGHT, d10);
  delay(t);
  digitalWrite(EYE_LEFT, 1);
  digitalWrite(EYE_RIGHT, 1);
}

uint32_t MiniLFRV2::infraReceive(){
  uint32_t ret;
  irrecv.enableIRIn();
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




