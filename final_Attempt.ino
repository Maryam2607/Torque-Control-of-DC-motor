#include <MovingAverageFilter.h>
MovingAverageFilter movingAverageFilter(20);
int sensor;
int sensor1;
int PWM,I,output,Reference=509,output1;
double error, prev_error=0;
double integral = 0, derivative = 0;
int time=0, previousTime=0;
float dt;
int kp=1;
int kd=1;
int ki=0;
int kt=-1,Torque;
void PIDloop();
void setup() {
  // put your setup code here, to run once:
pinMode(A1,INPUT);
pinMode(9,OUTPUT);
pinMode(4,OUTPUT);
pinMode(6,OUTPUT);
Serial.begin(250000);
}
void loop() {
  // put your main code here, to run repeatedly:
    // put your main code here, to run repeatedly:
  digitalWrite(9,LOW);
  digitalWrite(4,HIGH);
  analogWrite(6,PWM);
 output=(analogRead(A1));
 output1 = movingAverageFilter.process(output); 
 //Serial.println("without filter");
 //Serial.println(output);
// Serial.println("with filter");
// Serial.println(output1);
error=Reference-output1;
 //Serial.println("error");
//Serial.println(error);
previousTime = time;                                           // Save the time of the previous cycle
 time = millis();
  float dt = time - previousTime;  
   //if (abs(error) <0)
    //  PIDloop();
      //else
     // PWM=100;}
      //void PIDloop() {
  integral = integral + (error * dt);
 derivative = (error - prev_error) / dt;
 PWM = (kp * error) + (ki * integral) + (kd * derivative);
 prev_error = error;
 I=PWM*output1;
 Torque=kt*I;
  Serial.println(Torque);
  delay(100);}
//}
// sensor1=sensor-510;
//I=sensor/0.125;
//Serial.println(I);

// Serial.println("without_filter");
 // Serial.println(I);
  //Serial.println("with_filter");
   // Serial.println(output);
    //delay(100);
//if(sensor1>100)
//{PWM=75;}
//else 
//PWM=200;
 // sensor=Serial.print(analogRead(A1));
  //Serial.println();

//}
