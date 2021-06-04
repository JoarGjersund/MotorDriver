#include <Arduino.h>
#ifdef ESP32
#include <analogWrite.h>
#endif
class MotorDriver {
    public:
    MotorDriver(int pin_enable, int pin_phase); 
    
    double gearfactor = 5; //430.0 / 360.0;// gearfactor: encoder_position = output_angle*gearfactor 
    void setFrequency(float frequency); // Frequency in revolutions per second. Normalized to amplitude [ A*sin(f*t/A) ]
    void setAmplitude(float amplitude); // current motor angle must be smaller than new amplitude
    void setPhaseOffset(float offset); // set offset in radians. will be added to current offset.
    void setMidline(int midline); // around where should the motor move. aka Y-axis offset.
    void stop(unsigned long delaytime_ms=0); // 
    void pause(bool doPause);
    bool update(volatile int encoder_position); // returns true if motor is moving.
    bool isAtPeakTop(); // for debounce purpose: calling this will clear flag until after peak bottom is reached
    bool isAtPeakBottom(); // for debounce purpose: calling this will clear flag until after peak top is reached
    void goTo(bool enable, float targetAngle, float acceleration); // set desired position and go.
    void sync(double input_angle); // parrot another motor. Must be called before each update().
    bool motor_move = false;
    double _angle = 0;

    private:
    int pin_en;
    int pin_ph;
    float _frequency = 12*M_PI;
    float _amplitude = 10;
    unsigned long _stop_t0 = 0;
    unsigned long _stall_t0 = 0;
    bool _pause = false;
    double _angle_prev = 0;
    float _angle_stallStart = 0;
    int16_t torque = 250;
    unsigned int stall_cutoff_ms = 50;
    bool isStalling = false;
    int encoder_position_prev = 0;
    double _phase_offset = 0.0;
    bool isClimbing=false;
    float newAmplitude=0.0f;
    float newFrequency=0.0f;
    float newOffset=0.0f;
    bool goAndStop=false;
    float goAndStop_targetAngle=0;
    double goAndStop_acceleration=0.1;
    double goAndStop_speed=0.0;
    int goAndStop_directionUp=0; // -1 down, +1 up, 0 nowhere.    
    bool parrot=false;
    double* parrot_angle;
    int _midline = 0;
    bool newMidline=false;

};