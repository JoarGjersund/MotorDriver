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
    if (2*M_PI*frequency == _frequency) return;
    frequency*=2*M_PI;
    double newOffset=(_frequency-frequency)*millis()/1000.0+_phase_offset; // to make a smooth transition to new frequency we need to shift phase.
    _phase_offset=newOffset;

    _frequency=frequency;

}

void MotorDriver::setAmplitude(float amplitude) {
    if (amplitude==newAmplitude || amplitude==_amplitude) return;
    newAmplitude=amplitude;
}

void MotorDriver::stop(unsigned long delaytime_ms) {
    if (millis() <= _stop_t0) return;
    _stop_t0=millis()+delaytime_ms;
    _phase_offset-=_frequency*delaytime_ms/1000.0;

}
void MotorDriver::pause(bool doPause) {
    _pause=doPause;

}



bool MotorDriver::update(int encoder_position) {
    if (millis() < _stop_t0 || _pause) {
        analogWrite(pin_en, 0);
        return false;
    }

    _angle_prev=_angle;
    // if we want to  change amplitude we do it when angle is 0 to avoid having to deal with phase change.
    if (newAmplitude!=0 && round(_angle) == 0){
        _amplitude=newAmplitude;
        newAmplitude=0;
    }
    _angle = _amplitude*sin(_frequency*millis()/1000.0+_phase_offset); // asin(ft+ph)

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



