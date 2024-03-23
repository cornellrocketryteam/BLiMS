#define PULL_SWITCH A1

void setup() {
  Serial.begin(115200);
  pinMode(PULL_SWITCH, INPUT_PULLUP);
}

void loop() {
  //Serial.println(digitalRead(PULL_SWITCH));
  if(digitalRead(PULL_SWITCH) == HIGH){
    Serial.println("High");
  } else {
    Serial.println("Low/Pulled");
  }
  delay(1000);

}
