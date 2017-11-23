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

public:
	MiniLFRV2();
	void init();
	void loop();
	// motor related
	void speedSet(int spdl, int spdr);
	void speedSet(int spdl, int spdr, int delay);
	void stop();
	void motorDiffSet(float diff);
	float motorDiffGet();
	// peripheral
	void buttonGet(int btn);
	void eyeLedSet(int left, int right);
	int distance();
	float batteryVoltage();
	// music 
	void buzz(int freq, int delay);
	void playNote(int note);
	void playMusic(int music);
	void playMusic(uint8_t * music);
	// RGB
	void rgbBrightness(int value);
	void hoverRgb(int pix, int r, int g, int b);
	void headRgb(int pix, int r, int g, int b);
	// Sensor
	int getSensor(int index);
	void setSensorThreshold(int index, int value);
	int getSensorThreshold(int index);
	// LED matrix
	void matrixShow(char * data);
	void matrixShowTime(int hour, int minute);
	void matrixShowString(char * str);
	// Linefollow related
	void updatePid(float p, float i, float d);
	void pidLoop();
	int thresholdCalibrate();
	int motorCalibrate();

};
#endif
