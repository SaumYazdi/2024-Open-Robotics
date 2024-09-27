
void setup() {
  Serial1.begin(115200);
}

float bytesToFloat(uint8_t *bytes) {
  static_assert(sizeof(float) == 4, "Float size shuold be 4 bytes.");
  float f;
  memcpy(&f, bytes, 4);
  return f;
}

const int NUM_BYTES = 8;
int byteIndex = 0;
char data[4];
void loop() {
  if (Serial1.available() > 0) {
    char recv = Serial1.read();
    data[byteIndex] = recv;

    if (++byteIndex >= NUM_BYTES) {
      byteIndex = 0;
      
      uint8_t distBytes[4] = {data[0], data[1], data[2], data[3]};
      float dist = bytesToFloat(distBytes);

      uint8_t angleBytes[4] = {data[4], data[5], data[6], data[7]};
      float angle = bytesToFloat(angleBytes);

      String text = "dist: " + String(dist, 3) + " angle: " + String(angle, 3);
      Serial.println(text);
    }
  }
}
