void setup() {
  // put your setup code here, to run once:
pinMode(19, OUTPUT);
pinMode(22, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(19, 128);
   delay(random(1000,5000));
  analogWrite(19, 0);
  analogWrite(22, 128);
  delay(random(1000,5000));
  analogWrite(22, 0);
}
