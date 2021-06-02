#include <Arduino.h>
#ifdef ESP32
#include <analogWrite.h>
#endif
class MotorDriver {
    public:
    MotorDriver(int pin_enable, int pin_phase); // gearfactor: encoder_position = output_angle*gearfactor 
    
    double gearfactor = 430.0f / 360.0f;
    void setFrequency(float frequency); // Frequency in revolutions per second.
    void setAmplitude(float amplitude); // current motor angle must be smaller than new amplitude
    void stop(unsigned int delaytime_ms=0); // 
    void pause(bool doPause);
    bool update(int encoder_position); // returns true if motor is moving.
    bool isAtPeakTop(); // for debounce purpose: calling this will clear flag until after peak bottom is reached
    bool isAtPeakBottom(); // for debounce purpose: calling this will clear flag until after peak top is reached
    void init();
    private:
    int pin_en;
    int pin_ph;
    float _frequency = 0.1;
    float _amplitude = 10;
    unsigned long _stop_t0 = 0;
    unsigned long _stall_t0 = 0;
    bool _pause = false;
    float _angle = 0;
    float _angle_prev = 0;
    int16_t torque = 170;
    unsigned int stall_cutoff_ms = 500;
    bool isStalling = false;
    int encoder_position_prev = 0;
    double _phase_offset = 0.0;
    double setOffset();
    bool isClimbing=false;
    

};