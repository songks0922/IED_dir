// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#include <Servo.h>
#define PIN_SERVO 10
#define DIST_ALPHA  0.3

float dist_ema, alpha;

#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)

#define _POS_START 2000
#define _DUTY_NEU 1580
#define _POS_END 1000

#define _SERVO_SPEED 80 // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval

unsigned long last_sampling_time; // unit: ms
int duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec

//추헌준 도전과제 참고코드
const float coE[] = {-0.0000095, 0.0056762, 0.0855494, 68.9527052};
//

int duty_target, duty_curr;

Servo myservo;

void setup() {
// initialize GPIO pins

  myservo.attach(PIN_SERVO);
  
// initialize serial port
  Serial.begin(57600);
  
  duty_curr = _DUTY_NEU;
  
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;

  // initialize variables for servo update.
  pause_time = 1;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
  last_sampling_time = 0;
  alpha = DIST_ALPHA;

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}


void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  float raw_dist = ir_distance();
  
  //추헌준 도전과제 참고 코드
  float x = ir_distance();
//  raw_dist= coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  //
  dist_ema = (1-alpha) * dist_ema + alpha * raw_dist;
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_ema:");
  Serial.println(dist_ema);

  if(dist_ema > 255){
    duty_target = _POS_START;
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    
  } else if(dist_ema < 255){
    duty_target = _POS_END;
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
     
  }

  myservo.writeMicroseconds(duty_curr);
//



    
  if(toggle_interval_cnt >= toggle_interval) {
    toggle_interval_cnt = 0;
    if(duty_target == _POS_START) duty_target = _POS_END;
    else duty_target = _POS_START;
  }
  else {
    toggle_interval_cnt++;
  }
  

  last_sampling_time += INTERVAL;
  
}
