#include <Motordriver.h>


MotorDriver::MotorDriver( int pin_enable, int pin_phase){

    pin_en=pin_enable;
    pin_ph=pin_phase;
    pinMode(pin_en, OUTPUT);
    pinMode(pin_ph, OUTPUT);



}

void MotorDriver::setFrequency(float frequency) {
    if (2.0f*M_PI*frequency == _frequency) return;
    newFrequency=2.0f*M_PI*frequency;

}

void MotorDriver::setAmplitude(float amplitude) {
    if (amplitude==_amplitude) return;
    newAmplitude=amplitude;
}

void MotorDriver::setPhaseOffset(float offset) {
    if (newOffset!=0) return;
    newOffset=offset;
}

void MotorDriver::stop(unsigned long delaytime_ms) {
    if (millis() <= _stop_t0) return;
    _stop_t0=millis()+delaytime_ms;
    _phase_offset-=_frequency*delaytime_ms/(_amplitude*1000.0);

}
void MotorDriver::pause(bool doPause) {
    _pause=doPause;

}

void MotorDriver::goTo(bool enable, float targetAngle, float acceleration) {
    goAndStop=enable;
    goAndStop_targetAngle=targetAngle;
    goAndStop_acceleration=acceleration;

}



bool MotorDriver::update(volatile int encoder_position) {
    if (millis() < _stop_t0 || _pause) {
        analogWrite(pin_en, 0);
        return false;
    }
    if (goAndStop) {
        
        if ( abs(goAndStop_targetAngle*gearfactor-encoder_position) < 2){
            analogWrite(pin_en, 0);
            goAndStop_speed=0;

        } else if (goAndStop_targetAngle*gearfactor > encoder_position){
            goAndStop_speed+=goAndStop_acceleration;
            if (goAndStop_speed>torque) goAndStop_speed=torque;
            analogWrite(pin_en, goAndStop_speed);
            digitalWrite(pin_ph, HIGH);

        } else if (goAndStop_targetAngle*gearfactor < encoder_position){
            goAndStop_speed+=goAndStop_acceleration;
            if (goAndStop_speed>torque) goAndStop_speed=torque;
            analogWrite(pin_en, goAndStop_speed);
            digitalWrite(pin_ph, LOW);

        }
        return true;
    }

    
    // if we want to  change amplitude we do it when angle is 0 to avoid having to deal with phase change.
    if (newAmplitude!=0 && floor(_angle*10)/10 == 0){
        _phase_offset+=_frequency*(millis()/1000.0)*(1/_amplitude - 1/newAmplitude);
        _amplitude=newAmplitude;
        newAmplitude=0;
        
        
    }
    // we change frequency when want to do it at the paks. once angle is decreasing and we are at positive side of the axis we have just passed the top.
    if (floor(_angle*10)/10 == _amplitude && newFrequency!=0) {
        _frequency=newFrequency;
        newFrequency=0;
        _phase_offset=M_PI/2.0-_frequency*millis()/(_amplitude*1000.0);
    }
    // once angle is increasing and we are at negative side of the axis we have just passed the bottom.
    if (floor(_angle*10)/10 == -_amplitude && newFrequency!=0) {
        _frequency=newFrequency;
        newFrequency=0;
        _phase_offset=-M_PI/2.0-_frequency*millis()/(_amplitude*1000.0);
    }
    // if new offset. set offset.
    if (newOffset!=0) {
        _phase_offset+=newOffset;
        newOffset=0;
    }
    _angle_prev=_angle;
    _angle = _amplitude*sin(_frequency*millis()/(_amplitude*1000.0)+_phase_offset); // asin(ft+ph)
    

    bool motor_move = true;
    int16_t offset =  gearfactor*_angle - encoder_position;
    int16_t output_voltage = 0;

    // ramp up voltage until target is reached. ramp down voltage if target is reached.
    
    if (offset > 0) output_voltage=torque;
    else if (offset < 0) output_voltage=-torque;
    else output_voltage=0;

    // if max voltage and motor is still not moving assume stalling and cut voltage until moving the other way.
    if (!isStalling && encoder_position == encoder_position_prev && output_voltage!=0)
    {
        _angle_stallStart=_angle;
        isStalling=true;
        _stall_t0=millis();

    }
    else if (encoder_position != encoder_position_prev) {
        isStalling=false;
    }
    else if (isStalling && millis()-_stall_t0 > stall_cutoff_ms){
        
        output_voltage=0;
        if (abs(_angle) <= abs(_angle_stallStart)){ 
            isStalling=false;
        }
    }
    output_voltage=round(output_voltage*(0.2*(millis()-_stall_t0)/stall_cutoff_ms +0.8));
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
    if (isClimbing && _angle-_angle_prev < 0 )
    {
        return true;
    }
    if (_angle-_angle_prev < 0) isClimbing=false;
    else isClimbing=true;

    return false;
}

bool MotorDriver::isAtPeakBottom() {

    if (!isClimbing && _angle-_angle_prev > 0 ){
        return true;
    }
    if (_angle-_angle_prev < 0) isClimbing=false;
    else isClimbing=true;

    return false;

}



