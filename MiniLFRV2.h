#ifndef MiniLFRV2_h
#define MiniLFRV2_h

#include <Arduino.h>

#define PIN_BUZZ  4
#define EYE_LEFT  12
#define EYE_RIGHT 10

#define PIN_RGB	  13
#define PIN_BTN1  7
#define PIN_BTN2  8

#define PIN_M1A	6
#define PIN_M1B 5
#define PIN_M2A 11
#define PIN_M2B 3

class MiniLFRV2
{
private:
	float Kp = 90; // 80
	float Ki = 0.15; // 0.15
	float Kd = 1200; //1200
	float error, errorLast, erroInte;
	int bias = 0;
	int btn1, btn2;
	int pos;
	int outlineCnt = 0;
	bool tracing = true;    //巡线标签
	int adval[5];
	int adcal[5];
	int spdR, spdL;
	uint8_t _mode;
public:
	uint8_t _codingMode;
	MiniLFRV2();
	uint8_t mode();
	uint8_t setMode(int i); // todo: remove mode from lib
	void updateMotorSpeed();
	void updateMotorSpeed(int spdl, int spdr);
	int echoTrace();
	void pidWork(float dcdiff);
	int GetSensor(unsigned char index);
	int GetPos();
	int GetSensorValue(int index);
	float calcPid(float input);
	void PidValue(float *p, float *i, float *d);
	void SetPid(float p, float i, float d);
	void buzz(int freq, int delay);

	void SetThreshold(int index, int val);
	int GetThreshold(int index);


	void doEye(int left, int right);
	float doDistance();
	float doBattery();
	int GetButton1();
	int GetButton2();
	void doCarMove(int fw, int lr,float dcdiff);
	void move(int fw, int lr);
	void stop();
	void SetDCSpeed(int idx, int speed,float dcdiff);
	void motorSpeed(int idx, int speed);
	void HoverLight(int pix, int r, int g, int b);
	void RingRGB(int pix, int r, int g, int b);
	void FrontRGB(int pix, int r, int g, int b);
	void avoid_mode(float dcdiff);

protected:
	int AD[5] = { A3,A2,A1,A0,A6 };

};
#endif
