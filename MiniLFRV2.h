#ifndef MiniLFRV2_h
#define MiniLFRV2_h

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
	float error, errorLast, erroInte;
  int outlineCnt;
  int getTrace();
  float calcPid(float input);
   
public:
  float Kp = 90; // 80
  float Ki = 0.15; // 0.15
  float Kd = 1200; //1200
  
	MiniLFRV2();
	void init();
  void loadSetup();
  void syncSetup();
	void loop();
	// motor related
	void speedSet(int spdl, int spdr);
	void speedSet(int spdl, int spdr, int t);
	void stop();
	void motorDiffSet(float diff);
	float motorDiffGet();
	// peripheral
	int buttonGet(int btn);
	void eyeLedSet(int left, int right);
	float distance();
	float batteryVoltage();
  uint32_t infraReceive();
  void infraSend(int hex);
	// music 
	void buzz(int freq, int duration);
	void buzz(int freq, int duration, int delayms);
	void playNote(int note, int clap);
	void playMusic(const char * notes);
	// RGB
	void setRgbBrightness(int value);
	void hoverRgbShow(int pix, int r, int g, int b);
	void headRgbShow(int pix, int r, int g, int b);
	// Sensor
	int getSensor(int index);
	void setSensorThreshold(int index, int value);
	int getSensorThreshold(int index);
	// LED matrix
	void matrixShow(uint8_t * data);
	void matrixShowTime(int hour, int minute);
	void matrixShowString(uint8_t * str);
	// Linefollow related
	void updatePid(float p, float i, float d);
  void startLineFollow();
	int pidLoop();
protected:
  int AD[5] = { A3,A2,A1,A0,A6 };
};
#endif
