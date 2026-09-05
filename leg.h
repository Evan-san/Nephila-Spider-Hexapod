Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);       // called this way, it uses the default address 0x40   
Adafruit_PWMServoDriver board2 = Adafruit_PWMServoDriver(0x41);
#define SERVOMIN  125                                                 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625                                                 // this is the 'maximum' pulse length count (out of 4096)

int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
  {  ang = constrain(ang, 0, 180);                    // clamp first — offsets like mid-5, mid+9 etc.
                                                       // can push values outside 0-180, which used to
                                                       // send out-of-range pulses and make servos jerk
     int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     //Serial.print("Angle: ");Serial.print(ang);
     //Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
  };
int servoChannel[18] = {
  0,  1,  2,   // Kaki 1: Coxa, Femur, Tibia
  3,  4,  5,   // Kaki 2
  6,  7,  8,   // Kaki 3
  9, 10, 11,   // Kaki 4
  12, 13, 14,  // Kaki 5
  15, 16, 17   // Kaki 6 → butuh board PCA kedua jika pakai 18 channel
};

int defaultangle[18]={
  90,10,180,
  90,10,180,
  90,10,180,
  90,10,180,
  90,10,180,
  90,10,180,
};

void setServoAngle(int index, int angle) {
  int channel = servoChannel[index];
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  // Jika hanya punya 1 modul PCA9685, channel harus 0-15
  if (channel >= 0 && channel <= 8) {
    board1.setPWM(channel, 0, pulse);
  }
  // Tambahan: jika pakai 2 modul PCA, perlu panggil driver kedua
  else if (channel >= 9) {
    board2.setPWM(channel - 9, 0, pulse);
  }
}


void leg1(int atas,int mid,int bawah){
  board1.setPWM(0, 0, angleToPulse(atas));
  board1.setPWM(1, 0, angleToPulse(mid-5));
  board1.setPWM(2, 0, angleToPulse(bawah-2));
}
void leg2(int atas,int mid,int bawah){
  board1.setPWM(3, 0, angleToPulse(atas-1));
  board1.setPWM(4, 0, angleToPulse(mid+5));
  board1.setPWM(5, 0, angleToPulse(bawah-15));
}
void leg3(int atas,int mid,int bawah){
  board1.setPWM(6, 0, angleToPulse(atas));
  board1.setPWM(7, 0, angleToPulse(mid-5));
  board1.setPWM(8, 0, angleToPulse(bawah-5));
}
void leg4(int atas,int mid,int bawah){
  board2.setPWM(0, 0, angleToPulse(atas));
  board2.setPWM(1, 0, angleToPulse(mid+9));
  board2.setPWM(2, 0, angleToPulse(bawah+3));
}
void leg5(int atas,int mid,int bawah){
  board2.setPWM(3, 0, angleToPulse(atas));
  board2.setPWM(4, 0, angleToPulse(mid));
  board2.setPWM(5, 0, angleToPulse(bawah-11));
}
void leg6(int atas,int mid,int bawah){
  board2.setPWM(6, 0, angleToPulse(atas));
  board2.setPWM(7, 0, angleToPulse(mid-7));
  board2.setPWM(8, 0, angleToPulse(bawah-2));
}