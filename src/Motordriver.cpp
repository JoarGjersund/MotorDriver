#include <Motordriver.h>


MotorDriver::MotorDriver( int pin_enable, int pin_phase){

    pin_en=pin_enable;
    pin_ph=pin_phase;




}
void MotorDriver::init() {

    pinMode(pin_en, OUTPUT);
    pinMode(pin_ph, OUTPUT);
    
}
void MotorDriver::setFrequency(float frequency) {
    frequency*=2*M_PI;
    double newOffset=(_frequency-frequency)*millis()/1000+_phase_offset; // to make a smooth transition to new frequency we need to shift phase.
    _phase_offset=newOffset;

    _frequency=frequency;

}

void MotorDriver::setAmplitude(float amplitude) {

    double ratio = _angle / amplitude;
    if (abs(ratio) > 1)
        _amplitude=_angle;
    else
        _amplitude=amplitude;
    
    setOffset(); // to change amplitude smothly we need to shift phase so both angles match up.
}

void MotorDriver::stop(unsigned int delaytime_ms) {
    _stop_t0=millis()+delaytime_ms;

}
void MotorDriver::pause(bool doPause) {
    _pause=doPause;

}

double MotorDriver::setOffset() {
    _phase_offset = asin(_angle/_amplitude) - _frequency * millis() / 1000; 
}

bool MotorDriver::update(int encoder_position) {
    if (millis() < _stop_t0 || _pause){
        setOffset();
        return false;
    }
    _angle_prev=_angle;
    _angle = _amplitude*sin(_frequency*millis()/1000+_phase_offset); // asin(ft+ph)

    bool motor_move = true;
    int16_t offset =  gearfactor*_angle - encoder_position;
    int16_t output_voltage = 0;

    // ramp up voltage until target is reached. ramp down voltage if target is reached.
    if (offset > 0) output_voltage=torque;
    else if (offset < 0) output_voltage=-torque;
    else output_voltage=0;


    // if max voltage and motor is still not moving assume stalling and cut voltage until moving the other way.
    if (!isStalling && encoder_position == encoder_position_prev && abs(output_voltage) == torque)
    {
        _angle_prev=_angle;
        isStalling=true;
        _stall_t0=millis()+stall_cutoff_ms;

    }
    else if (encoder_position != encoder_position_prev) {
        isStalling=false;
    }
    else if (isStalling && millis() > _stall_t0){
        
        output_voltage=0;
        if (abs(_angle) <= abs(_angle_prev)){ 
            isStalling=false;
        }
    }

    // output direction and voltage to motor
    digitalWrite(pin_ph, (output_voltage > 0));
    analogWrite(pin_en, abs(output_voltage));

    // return true if motor has moved since last update
    if (encoder_position == encoder_position_prev) motor_move=false;
    else
        motor_move=true;

    encoder_position_prev = encoder_position;

    return motor_move;

}

bool MotorDriver::isAtPeakTop() {
    if (isClimbing && _angle-_angle_prev < 0)
    {
        isClimbing=false;
        return true;
    }
    return false;
}

bool MotorDriver::isAtPeakBottom() {

    if (!isClimbing && _angle-_angle_prev > 0) {
        isClimbing=true;
        return true;
    }
    return false;

}



