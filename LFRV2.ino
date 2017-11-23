#include <math.h>
#include <Wire.h>
#include "MiniLFRV2.h"

enum mode {
  CODING,
  LINEFOLLOW,
  OBJECTAVOID
};

MiniLFRV2 mini;
void setup(){
  Serial.begin(115200);
  mini.init();
  
}

void loop(){
  
}


