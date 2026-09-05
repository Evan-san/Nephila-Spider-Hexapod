
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  150 
#define SERVOMAX  600 

#define SERVO_FREQ 50 
int posisi =90;

int Angle;

uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();


  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}


void loop() {
  // Drive each servo one at a time using setPWM()

Serial.println(servonum);
 

  Angle=map(posisi,0,180,SERVOMIN,SERVOMAX);


    pwm.setPWM(servonum, 0, Angle );
  
 
  delay(1000);
  servonum++;
  if (servonum > 8) servonum = 0; // Testing the first 8 servo channels

}
