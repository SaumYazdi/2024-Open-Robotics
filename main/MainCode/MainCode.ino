#include "Wire.h"
#include "PowerfulBLDCdriver.h"

PowerfulBLDCdriver motor1;
PowerfulBLDCdriver motor2;
PowerfulBLDCdriver motor3;
PowerfulBLDCdriver motor4;

void setup() {
  Wire.setSCL(9);
  Wire.setSDA(8);
  Serial.begin(115200); // initialise serial
  Wire.begin(); // initialise i2c0, make sure to look up the i2c pins of your microcontroller.
  Wire.setClock(1000000); // set i2c speed to 1MHz
  motor1.begin(25, &Wire); // motor1 has i2c address 25 and is using i2c0.
  motor1.setCurrentLimitFOC(65536*2); // set current limit to 1 amp (only works in FOC mode)
  motor1.setIdPidConstants(1500, 200); 
  motor1.setIqPidConstants(1500, 200);
  motor1.setSpeedPidConstants(4e-2, 4e-4, 3e-2); // Constants valid for FOC and Robomaster M2006 P36 motor only, see tuning constants document for more details
  motor2.begin(27, &Wire); // motor1 has i2c address 25 and is using i2c0.
  motor2.setCurrentLimitFOC(65536*2); // set current limit to 1 amp (only works in FOC mode)
  motor2.setIdPidConstants(1500, 200); 
  motor2.setIqPidConstants(1500, 200);
  motor2.setSpeedPidConstants(4e-2, 4e-4, 3e-2); // Constants valid for FOC and Robomaster M2006 P36 motor only, see tuning constants document for more details
  motor3.begin(28, &Wire); // motor1 has i2c address 25 and is using i2c0.
  motor3.setCurrentLimitFOC(65536*2); // set current limit to 1 amp (only works in FOC mode)
  motor3.setIdPidConstants(1500, 200); 
  motor3.setIqPidConstants(1500, 200);
  motor3.setSpeedPidConstants(4e-2, 4e-4, 3e-2); // Constants valid for FOC and Robomaster M2006 P36 motor only, see tuning constants document for more details
  motor4.begin(26, &Wire); // motor1 has i2c address 25 and is using i2c0.
  motor4.setCurrentLimitFOC(65536*2); // set current limit to 1 amp (only works in FOC mode)
  motor4.setIdPidConstants(1500, 200); 
  motor4.setIqPidConstants(1500, 200);
  motor4.setSpeedPidConstants(4e-2, 4e-4, 3e-2); // Constants valid for FOC and Robomaster M2006 P36 motor only, see tuning constants document for more details
  motor1.setELECANGLEOFFSET(1510395136); // set the ELECANGLEOFFSET calibration value. Each motor needs its own calibration value.
  motor1.setSINCOSCENTRE(1251); // set the SINCOSCENTRE calibration value. Each motor needs its own calibration value.
  motor1.configureOperatingModeAndSensor(3, 1); // configure FOC mode and sin/cos encoder
  motor1.configureCommandMode(12); // configure speed command mode
  motor2.setELECANGLEOFFSET(1480679936); // set the ELECANGLEOFFSET calibration value. Each motor needs its own calibration value.
  motor2.setSINCOSCENTRE(1251); // set the SINCOSCENTRE calibration value. Each motor needs its own calibration value.
  motor2.configureOperatingModeAndSensor(3, 1); // configure FOC mode and sin/cos encoder
  motor2.configureCommandMode(12); // configure speed command mode
  motor3.setELECANGLEOFFSET(1315084800); // set the ELECANGLEOFFSET calibration value. Each motor needs its own calibration value.
  motor3.setSINCOSCENTRE(1224); // set the SINCOSCENTRE calibration value. Each motor needs its own calibration value.
  motor3.configureOperatingModeAndSensor(3, 1); // configure FOC mode and sin/cos encoder
  motor3.configureCommandMode(12); // configure speed command mode
  motor4.setELECANGLEOFFSET(1697132032); // set the ELECANGLEOFFSET calibration value. Each motor needs its own calibration value.
  motor4.setSINCOSCENTRE(1235); // set the SINCOSCENTRE calibration value. Each motor needs its own calibration value.
  motor4.configureOperatingModeAndSensor(3, 1); // configure FOC mode and sin/cos encoder
  motor4.configureCommandMode(12); // configure speed command mode

  delay(500);
}

void setMotorSpeeds(float angle) {
  // Convert angle to radians
  float rad = (angle - 45) * (PI / 180.0);

  // Calculate motor speeds
  float speedY1 = -cos(rad);
  float speedX2 = -sin(rad);
  float speedY3 = cos(rad);
  float speedX4 = sin(rad);

  // Scale speeds to motor speed range (adjust based on your motor's max speed)
  float maxSpeed = 90000000;
  float scaledSpeedY1 = speedY1 * maxSpeed;
  float scaledSpeedX2 = speedX2 * maxSpeed;
  float scaledSpeedY3 = speedY3 * maxSpeed;
  float scaledSpeedX4 = speedX4 * maxSpeed;

  // Set the motor speeds
  motor1.setSpeed(scaledSpeedY1);
  motor2.setSpeed(scaledSpeedX2);
  motor3.setSpeed(scaledSpeedY3);
  motor4.setSpeed(scaledSpeedX4);
}

void loop() {
  // Set the desired angle here
  float angle = 0; // Change this value to set a different angle
  setMotorSpeeds(angle);
  
  delay(1000); // Adjust delay as needed
}
