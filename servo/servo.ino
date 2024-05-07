#include <Servo.h>
String angles, servo1_angle_s, servo2_angle_s, servo3_angle_s;
float servo1_angle, servo2_angle, servo3_angle, ser_read_angle1_ms, ser_read_angle2_ms, ser_read_angle3_ms;
int servo1_angle_ms, servo2_angle_ms, servo3_angle_ms, s1_def_pos, s2_def_pos, s3_def_pos, init_pos;
Servo servo1;
Servo servo2;
Servo servo3;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  // Serial connection
  //delay(1000);
  //Serial.println("Arduino!");

  //Servo attach
  servo1.attach(11);
  servo2.attach(10);
  servo3.attach(9);
  
  //Servo default position in ms
  s1_def_pos = 830;
  s2_def_pos = 1010;
  s3_def_pos = 860;
  init_pos = 500;
  servo1.writeMicroseconds(s1_def_pos+init_pos);
  servo2.writeMicroseconds(s2_def_pos+init_pos);
  servo3.writeMicroseconds(s3_def_pos+init_pos);
}

void loop() {
  digitalWrite(13, millis() / 800 %2);
  while (Serial.available() > 0) {
    char rec = Serial.read();
    angles += rec;
    if(rec == '\n'){
      //Delete \n\b\r
      angles.remove(0,3);
      angles.remove(19,2);

      //Read angle, change to ms
      servo1_angle_s = angles.substring(0,5);
      servo1_angle = servo1_angle_s.toFloat();
      if(servo1_angle == 0) {
        servo1_angle = 47.96;
      }
      ser_read_angle1_ms = map(servo1_angle,0,180,500,2500);
      servo1_angle_ms = s1_def_pos+ser_read_angle1_ms-init_pos;
      
      if(servo1_angle_ms>2400) {
        servo1_angle_ms = 2400;
        }
      if(servo1_angle_ms<500) {
        servo1_angle_ms = 500;
        }
      
      servo2_angle_s = angles.substring(6,11);
      servo2_angle = servo2_angle_s.toFloat();
      if(servo2_angle == 0) {
        servo2_angle = 47.96;
      }
      ser_read_angle2_ms = map(servo2_angle,0,180,500,2500);
      servo2_angle_ms = s2_def_pos+ser_read_angle2_ms-init_pos;
      if(servo2_angle_ms>2400) {
        servo2_angle_ms = 2400;
        }
      if(servo2_angle_ms<500) {
        servo2_angle_ms = 500;
        }
      
      servo3_angle_s = angles.substring(12,17);
      servo3_angle = servo3_angle_s.toFloat();
      if(servo3_angle == 0) {
        servo3_angle = 47.96;
      }
      ser_read_angle3_ms = map(servo3_angle,0,180,500,2500);
      servo3_angle_ms = s3_def_pos+ser_read_angle3_ms-init_pos;
      if(servo3_angle_ms>2400) {
        servo3_angle_ms = 2400;
      }
      if(servo3_angle_ms<500) {
        servo3_angle_ms = 500;
      }

      //Write servo
      servo1.writeMicroseconds(servo1_angle_ms);
      servo2.writeMicroseconds(servo2_angle_ms);
      servo3.writeMicroseconds(servo3_angle_ms);
      
      //Serial.print("Message recieved ");
      Serial.println(servo1_angle_ms);
      Serial.println(servo2_angle_ms);
      Serial.println(servo3_angle_ms);
      Serial.println(ser_read_angle1_ms);
      Serial.println(ser_read_angle2_ms);
      Serial.println(ser_read_angle3_ms);
      Serial.println(angles);
      
      angles ="";
    }
  }
  
  

}
