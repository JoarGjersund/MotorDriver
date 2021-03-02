#include <Arduino.h>
#ifdef ESP32
#include <analogWrite.h>
#endif
class MotorDriver {
    public:
    float phase = 0;
    int amplitude = 90;
    int offset = 90;
    MotorDriver(int PIN_ENABLE, int PIN_PHASE, int max_angle, int min_angle);
    int update();
    void setTargetPosition(int target);
    int getCurrentPosition();
    bool setCalibrationConstant(int newConstant);
    void setMinimumStepSize(int newMinimumStepSize);
    int getTargetPositionRounded();
    bool isMoving();
    int targetPosition;
    int speed = 255;
    void recalibrate(int direction = 0, int speed = 50, int calibrationconstant_new = 0, int percent_delay = 100);
    int angle_max = 180;
    int angle_min = 0;
    
    private:
    int currentPosition = 0;
    unsigned long timeWhenLastUpdated = 0;
    unsigned long timeWhenLastCalibrated = 0;
    int PIN_ENABLE;
    int PIN_PHASE;
    int direction = 0;
    int calibrationConstant=130; // slower motor -> Higher number. ms needed to go from 0 to 180 at maximum speed.
    int minimumStepSize=8; // minimum step size in degrees. higher step size increase torque and accuracy. should be defined by how often  m1.update() is called.

    bool calibrationInProgress = false;
    unsigned long calibrationDelayTime = 0;
    unsigned long calibrationTimeStart = 0;

};