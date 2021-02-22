#include <Arduino.h>
#ifdef ESP32
#include <analogWrite.h>
#endif
class MotorDriver {
    public:
    MotorDriver(int PIN_ENABLE, int PIN_PHASE);
    int update();
    void setTargetPosition(int target);
    int getCurrentPosition();
    int targetPosition;
    int speed = 255;
    void recalibrate(int direction = 0, int speed = 255, int calibrationconstant_new = 0, int percent_delay = 100);

    private:
    int currentPosition = 0;
    unsigned long timeWhenLastUpdated = 0;
    unsigned long timeWhenLastCalibrated = 0;
    int PIN_ENABLE;
    int PIN_PHASE;
    int direction = 0;
    int calibrationConstant=130; // slower motor -> Higher number. ms needed to go from 0 to 180 at maximum speed.
    int minimumStepSize=2; // minimum step size in degrees. higher step size increase torque and accuracy.
    int angle_max = 180;
    int angle_min = 0;

};