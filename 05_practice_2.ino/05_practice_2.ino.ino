#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while(!Serial){
    ;
  }
}

void loop() {
  digitalWrite(PIN_LED, 0); //1초동안 켜기
  delay(1000);
  
  turnOffOn(5);
  
  digitalWrite(PIN_LED, 1); //LED를 끄고 그 상태 유지
  while(1){
    ;   
  }
}

void turnOffOn(int t){  
  
  for(int i = 0; i < t; i++){
    digitalWrite(PIN_LED, 1); //꺼짐
    delay(100);
    digitalWrite(PIN_LED, 0); //켜짐
    delay(100);
  }

}
