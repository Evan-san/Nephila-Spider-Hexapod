#include <Adafruit_PWMServoDriver.h>
#include "leg.h"
#include "move.h"
#include <Ps3Controller.h>
#include "Control.h"


void setup() {
  Serial.begin(115200);
  board1.begin();
  board1.setPWMFreq(60);
  board2.begin();
  board2.setPWMFreq(60);

  for (int i = 0; i < 18; i++) {
    setServoAngle(i, defaultangle[i]);
    delay(500);
  }
delay(1000);
leg1(90,10,10);
leg2(90,10,10);
leg3(90,10,10);
leg4(90,10,10);
leg5(90,10,10);
leg6(90,10,10);

//delay(1000);

//leg1(90,115,140);
//leg2(90,115,140);
//leg3(90,115,140);
//leg4(90,115,140);
//leg5(90,115,140);
//leg6(90,115,140);

  delay(1000);


  Ps3.attachOnConnect(onPs3Connect);
  Ps3.attachOnDisconnect(onPs3Disconnect);
  Ps3.begin(); 
  printEsp32Mac();
  Serial.println("Waiting for PS3 controller...");
}

char lastMode = 'x'; 

void loop() {

  updateModeFromPs3();


  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w' || c == 's' || c == 'n' || c == 'x'|| c == 'a'|| c == 'd') {
      mode = c;
    }
  }

 
  if (mode != lastMode) {
    if (mode == 'x' || mode == 'n') {
      standNeutral();
    }
    lastMode = mode;
  }

  if (mode == 'w') {
    walkForwardStep();
  } else if (mode == 's') {
    walkBackwardStep();
  }else if (mode == 'd') {
    turnRight(1);
  }else if (mode == 'a') {
    turnLeft(1);
  }


}
