#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9  //[1692] LED 9번핀에 연결
#define PIN_SERVO 10 // [3228] 서보10핀에 연결
#define PIN_IR A0  // [3133] 적외선 센서 signal -> A0핀

// Framework setting
#define _DIST_TARGET 255  //[0028] 목표 위치가 25.5cm임을 선언
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.34   // [1628] ema 필터의 측정 보정치

// Servo range
#define _DUTY_MIN 1050  //[0028] servo duty값 최소를 1000으로 고정 
#define _DUTY_NEU 1450        
#define _DUTY_MAX 1850  //[3145] servo duty값 최대를 2000으로 고정

// Servo speed control
#define _SERVO_ANGLE 30        // [3131] servo 각도 설정
#define _SERVO_SPEED 180        // [3141] servo 속도 설정

// Event periods
#define _INTERVAL_DIST 20 
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 0.2

#define DELAY_MICROS  1500 

//////////////////////
// global variables //
//////////////////////
float dist_min, dist_max, dist_raw ; // [3228]
// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; // location to send the ball
float dist_cail, dist_ema = 0;

float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3; 

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial, time_curr; 
bool event_dist, event_servo, event_serial;

//추헌준 도전과제 참고코드
const float coE[] = {-0.0000114, 0.0066235, -0.0041811, 72.6701333};
//

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

// ================
float under_noise_filter(void){
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}
float filtered_ir_distance(void){
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  dist_ema = _DIST_ALPHA*lowestReading + (1-_DIST_ALPHA)*dist_ema;
  return lowestReading;
}
//===================================================

float control_to_ms(float c){
  return _DUTY_NEU - (200/17 * c - 200/17);
}

void setup() {
// initialize GPIO pins for LED and attach servo 
 pinMode(PIN_LED,OUTPUT);
 digitalWrite(PIN_LED, 1);
 myservo.attach(PIN_SERVO); 
 // [3228]

// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX; 
  // [3129]

// move servo to neutral position
 myservo.writeMicroseconds(_DUTY_NEU); // [3228]
 duty_curr = _DUTY_NEU;


// initialize serial port
Serial.begin(57600); //[3128] 시리얼 포트 초기화

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000); //[3128]
  event_dist = event_serial = event_servo = false;
}
  
void loop() {
/////////////////////
// Event generator // [3133] 이벤트 실행 간격 구현 
/////////////////////
  unsigned long time_curr = millis();
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
      event_dist = false; // [3133]
  // get a distance reading from the distance sensor
      dist_raw = ir_distance();
      filtered_dist = filtered_ir_distance();
      dist_cail = coE[0] * pow(filtered_dist, 3) + coE[1] * pow(filtered_dist, 2) + coE[2] * filtered_dist + coE[3];
  // PID control logic
    error_curr = _DIST_TARGET - dist_cail;
    pterm = error_curr * _KP;
    control = pterm;

  // duty_target = f(duty_neutral, control)
    duty_target = control_to_ms(control);

  // [3133] keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 
  
    last_sampling_time_dist = millis(); // [3133] 마지막 dist event 처리 시각 기록
  }
  
  if(event_servo) {
    event_servo = false; // [3133]
    // adjust duty_curr toward duty_target by duty_chg_per_interval

    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    
    // update servo position
    myservo.writeMicroseconds(duty_curr);

    last_sampling_time_servo = millis(); // [3133] 마지막 servo event 처리 시각 기록

  }
  
  if(event_serial) {
    event_serial = false; // [3133]
    Serial.print("dist_ir:");
    Serial.print(dist_cail);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    last_sampling_time_serial = millis(); // [3133] 마지막 serial event 처리 시각 기록

  }
}
