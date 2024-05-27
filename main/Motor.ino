#include "Wire.h"
#include "PowerfulBLDCdriver.h"
PowerfulBLDCdriver motor1;

void setup() {
  Serial.begin(115200);
  Wire.setSCL(9);
  Wire.setSDA(8);
  Wire.begin();
  motor1.begin(74, &Wire);
  motor1.setCurrentLimitFOC(131072);
  motor1.setSpeedPidConstants(5e-3, 1e-5, 2e-3); // valid for FOC only
  motor1.configureOperatingModeAndSensor(15, 1);
  motor1.configureCommandMode(15);
  motor1.setCalibrationOptions(300, 2097152, 50000, 500000);
  motor1.startCalibration();
  while (motor1.isCalibrationFinished() == false) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ELECANGLEOFFSET:");
  Serial.println(motor1.getCalibrationELECANGLEOFFSET());
  Serial.print("SINCOSCENTRE:");
  Serial.println(motor1.getCalibrationSINCOSCENTRE());

  motor1.configureOperatingModeAndSensor(3, 1);
  motor1.configureCommandMode(12);
  delay(500);
}

void loop() {
	motor1.setSpeed(5000000);
    delay(2000);

    motor1.updateQuickDataReadout();
    Serial.print("pos:");
    Serial.print(motor1.getPositionQDR());
    Serial.print(" spd:");
    Serial.print(motor1.getSpeedQDR());
    Serial.print(" err1:");
    Serial.print(motor1.getERROR1QDR());
    Serial.print(" err2:");
    Serial.println(motor1.getERROR2QDR());

    motor1.setSpeed(-5000000);
    delay(2000);

    motor1.updateQuickDataReadout();
    Serial.print("pos:");
    Serial.print(motor1.getPositionQDR());
    Serial.print(" spd:");
    Serial.print(motor1.getSpeedQDR());
    Serial.print(" err1:");
    Serial.print(motor1.getERROR1QDR());
    Serial.print(" err2:");
    Serial.println(motor1.getERROR2QDR());
}