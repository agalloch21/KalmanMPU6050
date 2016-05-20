#include <KalmanMPU6050.h>

KalmanMPU6050 accelemeter;
void setup() {

  Serial.begin(9600);
  
  // put your setup code here, to run once:
  accelemeter.setup();

}

void loop() {
  // put your main code here, to run repeatedly:
  accelemeter.update();
  Serial.print(accelemeter.getRoll()); Serial.print("\t");Serial.println(accelemeter.getPitch());

  if(Serial.available() > 0){
    String str = Serial.readString();
    if(str == "calib")
      accelemeter.calibration();
    if(str == "reset")
      accelemeter.reset();
      
  }

}
