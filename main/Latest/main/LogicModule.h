#ifndef LogicModule_h
#define LogicModule_h

#include "Wire.h"
#include "PowerfulBLDCdriver.h"
#include <math.h>
#include <vector>
#include <cmath>
#include <limits>
#include <Adafruit_BNO08x.h>

class LogicModule {
  public:
    LogicModule();
    void setReports(sh2_SensorId_t reportType, long report_interval);
    float anglePolynomial(float x);
    float distancePolynomial(float x);
    float calculateFinalDirection();
    void moveRobot(float direction, float rotation, int targetSpeed);
    bool readBall();
    void readIMU();
    void readTOFs();
    double simulateToF(double tx, double ty, double heading_deg, double sensor_offset, double sensor_angle_deg);
    void simulate(float tempX, float tempY, float direction);
    void odometry(float direction);
    void stop();
    void calibrate();
    void logic();
    void update();

  private:
    PowerfulBLDCdriver motor1;
    PowerfulBLDCdriver motor2;
    PowerfulBLDCdriver motor3;
    PowerfulBLDCdriver motor4;
    PowerfulBLDCdriver motor5;
    float distance = 0;
    float angle = 0;
    float heading = 0;
    float robotX = 0.0;
    float robotY = 0.0;
    float step = 100.0;
    int kickoffTicks = 0;
    int lostTicks = 0;
    int simDistances[8] = {0,0,0,0,0,0,0,0};
    int distances[8];
};

#endif