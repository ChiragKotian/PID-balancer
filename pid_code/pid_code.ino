#include <NewPing.h>
#include <Wire.h>
#include <Servo.h>

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
///////////////////////Inputs/outputs///////////////////////
int Analog_in = A0;
int i=0;
Servo myservo;  // create servo object to control a servo, later attatched to D9
///////////////////////////////////////////////////////


////////////////////////Variables///////////////////////
int Read = 0;
int distance = 0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////


///////////////////PID constants///////////////////////
float kp=8; //Mine was 8
float ki=20; //Mine was 0.2
float kd=3100; //Mine was 3100
float distance_setpoint = 28;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////



void setup() {
  //analogReference(EXTERNAL);
  Serial.begin(9600);  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(135); //Put the servco at angle 125, so the balance is in the middle
    
  time = millis();
}

void loop() {
 
  if (millis() > time+period)
  {
    time = millis();    
    distance = sonar.ping_cm(); 
    Serial.println(distance);
    if (distance >=200){distance = 200;}
    distance_error = distance_setpoint - distance;   
    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd*((distance_error - distance_previous_error)/period);
      
    if(-3 < distance_error && distance_error < 3)
    {
      PID_i = PID_i + (ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    Serial.println(PID_total);
    PID_total = map(PID_total, -150, 150, 0, 150);
  
    if(PID_total < 45){PID_total = 45;}
    if(PID_total > 155) {PID_total = 155; } 
  for(i=-5;i<=0;i=i+5){
    myservo.write(PID_total+30+i);  }
    for(i=5;i<=0;i=i-5){
    myservo.write(PID_total+30+i);  }
    
    distance_previous_error = distance_error;
  }
}
