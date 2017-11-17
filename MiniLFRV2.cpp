#include"MiniLFRV2.h"
#include "Adafruit_NeoPixel.h"

Adafruit_NeoPixel rgbled(2);
Adafruit_NeoPixel frontled(2);
Adafruit_NeoPixel ringrgb(15);

MiniLFRV2::MiniLFRV2(){
	tracing = false;
	_mode = 0;

	spdL = spdR = 0;
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

	//rgb
	rgbled.begin();
	rgbled.setPin(PIN_RGB);
	frontled.begin();
	frontled.setPin(EYE_RIGHT);
	//ringrgb.begin();
	//ringrgb.setPin(EYE_LEFT);
}

uint8_t MiniLFRV2::mode()
{
	return _mode;
}
uint8_t MiniLFRV2::setMode(int i)
{
	tracing = true;
	outlineCnt = 0;
	_mode = i;
}
void MiniLFRV2::updateMotorSpeed()
{
	if (spdL < 0) {
		analogWrite(PIN_M1B, 0);
		analogWrite(PIN_M1A, -spdL);
	}
	else {
		analogWrite(PIN_M1B, spdL);
		analogWrite(PIN_M1A, 0);
	}

	if (spdR < 0) {
		analogWrite(PIN_M2B, 0);
		analogWrite(PIN_M2A, -spdR);
	}
	else {
		analogWrite(PIN_M2B, spdR);
		analogWrite(PIN_M2A, 0);
	}

}

void MiniLFRV2::updateMotorSpeed(int spdl, int spdr)
{
	spdL = spdl;
	spdR = spdr;
	updateMotorSpeed();
}

int MiniLFRV2::GetSensorValue(int index){
	return analogRead(AD[index]);
}

void MiniLFRV2::buzz(int freq, int t){
	tone(PIN_BUZZ, freq, t);
}

int MiniLFRV2::echoTrace()
{
	int ret = 0;
	for (int i = 0; i < 5; i++) {
		adval[i] = analogRead(AD[i]);
		//Serial.print(a[i]);Serial.print(",");
		if (adval[i] < adcal[i]) ret += (0x1 << i);
	}
	//Serial.println(ret,BIN);
	return ret;
}
void MiniLFRV2::pidWork(float dcdiff)
{

	pos = echoTrace();
	btn1 = digitalRead(PIN_BTN1);
	btn2 = digitalRead(PIN_BTN2);
	if (!tracing) {                                         //巡线标签
		if (btn1 == 0) {
			_mode = 0;
			tone(PIN_BUZZ, 500, 200);delay(500);
			tone(PIN_BUZZ, 500, 200);delay(500);
			tone(PIN_BUZZ, 500, 300);delay(500);
	        tracing = true;outlineCnt = 0;
			if(digitalRead(PIN_BTN1) == 0)                 //长按3s左右
			{
			  
			  tracing = false;//outlineCnt = 0;
			  tone(PIN_BUZZ, 1000, 200);delay(300);
			  tone(PIN_BUZZ, 1000, 200);delay(300);
			  tone(PIN_BUZZ, 1000, 200);delay(500);
			  _mode = 2;                                      //校准阈值
	
             }
			
		}
		if (btn2 == 0) {
			while (!digitalRead(PIN_BTN2));
			_mode = 1;
			if (_mode == 1)
				return;
		}
	                                
	}		
		
	
	else {
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
			spdL /= 2;spdR /= 2;
			updateMotorSpeed();
			tone(PIN_BUZZ, 200, 100);delay(300);
			tone(PIN_BUZZ, 200, 100);
			spdL = 0;
			spdR = 0;
			tracing = false;
		}
		else {
			float ff = 150; //150
			float ctrl = -calcPid(bias);
			spdL = ff + ctrl;
			spdR = -(ff - ctrl);
		}
	}
	updateMotorSpeed();
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

int MiniLFRV2::GetSensor(unsigned char index)
{
	return adval[index];
}

int MiniLFRV2::GetPos()
{
	return pos;
}

void MiniLFRV2::PidValue(float *p, float *i, float *d)
{
	*p = Kp;
	*i = Ki;
	*d = Kd;
}

void MiniLFRV2::SetPid(float p, float i, float d)
{
	Kp = p;
	Ki = i;
	Kd = d;
}

int MiniLFRV2::GetThreshold(int index)
{


	return adcal[index];
}

void MiniLFRV2::SetThreshold(int index, int val)
{

	adcal[index] = val;
}

void MiniLFRV2::SetDCSpeed(int idx, int speed,float dcdiff)
{
	if (idx == 0) {
		spdL = speed;
		spdR = -speed*dcdiff;
	}
	else if (idx == 1) {
		spdL = speed;
	}
	else if (idx == 2) {
		spdR = -speed;
	}
	updateMotorSpeed();
}

void MiniLFRV2::motorSpeed(int idx, int speed){
	SetDCSpeed(idx,speed,1);
}

void MiniLFRV2::doCarMove(int fw, int lr,float dcdiff)
{
	
	spdL = (fw + lr);
	spdR = -(fw - lr)*dcdiff;     //*motorDiff;
}

void MiniLFRV2::move(int fw, int lr){
	spdL = (fw + lr);
	spdR = -(fw - lr);
	updateMotorSpeed();
}

void MiniLFRV2::stop(){
	updateMotorSpeed(0,0);
}

void MiniLFRV2::doEye(int left, int right)
{
	pinMode(EYE_LEFT, OUTPUT);
	pinMode(EYE_RIGHT, OUTPUT);
	digitalWrite(EYE_LEFT, left);
	digitalWrite(EYE_RIGHT, right);
}
float MiniLFRV2::doDistance()
{
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

float MiniLFRV2::doBattery()
{
	int a = analogRead(A7);
	float v = float(a) / 1024.0 * 5.2;
	return v;
}

int MiniLFRV2::GetButton1()
{
	return btn1;
}

int MiniLFRV2::GetButton2()
{
	return btn2;
}


void MiniLFRV2::HoverLight(int pix, int r, int g, int b)
{
	if (pix == 0) {
		for (int i = 0;i < 2;i++) {
			rgbled.setPixelColor(i, r, g, b);
		}
	}
	else {
		rgbled.setPixelColor(pix - 1, r, g, b);
	}

	rgbled.show();
}
void MiniLFRV2::RingRGB(int pix, int r, int g, int b)
{
	if (pix == 0) {
		for (int i = 0;i < 16;i++) {
			ringrgb.setPixelColor(i, r, g, b);
		}
	}
	else {
		ringrgb.setPixelColor(pix - 1, r, g, b);
	}

	ringrgb.show();
}

void MiniLFRV2::FrontRGB(int pix, int r, int g, int b)
{
	if (pix == 0) {
		frontled.setPixelColor(0, r, g, b);
		frontled.setPixelColor(1, r, g, b);
	}
	else {
		frontled.setPixelColor(pix - 1, r, g, b);
	}
	frontled.show();
}


void MiniLFRV2::avoid_mode(float dcdiff)
{
	float temp = doDistance();
	//Serial.println(temp);
	if (temp < 10)
	{
		updateMotorSpeed(100, 100*dcdiff);
	}
	else
	{
		updateMotorSpeed(150, -150*dcdiff);
	}
	delay(5);
}