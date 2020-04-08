int fanPin = 3;

void setup() {
  Serial.begin(9600);
  pinMode(fanPin, OUTPUT);
}

void loop() {
  String message = "\0";
  if(Serial.available() > 0)
  {
    message = Serial.readStringUntil('\n');
    Serial.println(message);
  }
  analogWrite(fanPin, message.toInt());
}
