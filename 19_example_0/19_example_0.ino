// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#include <Servo.h>
#define PIN_SERVO 10

int a, b; // unit: mm
Servo myservo;
void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);
  
// initialize serial port
  Serial.begin(57600);

  a = 70;
  b = 325;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  //float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  float dist_cali = 75.0/59.0 * raw_dist + 725.0/59.0;
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
//  if(dist_cali >= 255){
//    myservo.writeMicroseconds(1720);
//  } else if(dist_cali <255){
//    myservo.writeMicroseconds(1310);
//  }
//  
}
