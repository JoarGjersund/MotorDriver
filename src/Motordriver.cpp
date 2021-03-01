#include <Motordriver.h>


MotorDriver::MotorDriver(int pin_enable, int pin_phase){

    PIN_ENABLE=pin_enable;
    PIN_PHASE=pin_phase;

    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_PHASE, OUTPUT);


}
bool MotorDriver::setCalibrationConstant(int newConstant){
    calibrationConstant=newConstant;
    return true;
}

int MotorDriver::getCurrentPosition(){
    return currentPosition;
}

int MotorDriver::getTargetPositionRounded(){

    return round(targetPosition/minimumStepSize)*minimumStepSize;

}

bool MotorDriver::isMoving(){

    return abs(currentPosition-getTargetPositionRounded()) >=  minimumStepSize;

}




void MotorDriver::recalibrate(int calibrationdirection, int calibrationspeed, int calibrationconstant_new, int percent_delay){

    digitalWrite(PIN_PHASE, calibrationdirection);
    analogWrite(PIN_ENABLE, calibrationspeed);

    if (calibrationdirection==0) currentPosition=angle_min;
    else currentPosition=angle_max;

    if (calibrationconstant_new>0) delay(percent_delay*(1/100)*calibrationconstant_new*(255/speed));
    else delay(percent_delay*(1/100)*calibrationConstant*(255/speed));

    timeWhenLastCalibrated = millis();
}



void MotorDriver::setTargetPosition(int target){
    targetPosition=target;

}







int MotorDriver::update(){


    static bool init = false;
    if (!init){
        timeWhenLastUpdated=millis();
        init=true;
    }

    int timeSinceLastUpdated=millis()-timeWhenLastUpdated;
    


    int degreesMovedSinceLastUpdate=timeSinceLastUpdated*180/(calibrationConstant);

    if (degreesMovedSinceLastUpdate>0) timeWhenLastUpdated=millis();

    currentPosition+=degreesMovedSinceLastUpdate*direction;

    int targetPositionRounded=round(targetPosition/minimumStepSize)*minimumStepSize;

        // decide direction to move.
    if ( targetPositionRounded > currentPosition && currentPosition < angle_max){
        direction=1;
        digitalWrite(PIN_PHASE, 255); 
    } else if ( targetPositionRounded < currentPosition && currentPosition > angle_min){
        direction=-1;
        digitalWrite(PIN_PHASE, 0);
    }

    if (abs(currentPosition-targetPositionRounded) <  minimumStepSize || currentPosition > angle_max || currentPosition < angle_min )
    {
        direction=0;
        analogWrite(PIN_ENABLE, 0);
    }else{
        analogWrite(PIN_ENABLE, speed);
    }

    


    if (millis()-timeWhenLastCalibrated >5000){
        if (currentPosition>=angle_max) recalibrate(0, 255, calibrationConstant, 5);
        else if (currentPosition<=angle_min) recalibrate(255, 255, calibrationConstant, 5);    
    } else if (millis()-timeWhenLastCalibrated >10000){
        if (currentPosition>(2*angle_max-angle_min)/3) recalibrate(0, 255, calibrationConstant, 10);
        else if (currentPosition<(angle_max-angle_min)/3) recalibrate(255, 255, calibrationConstant, 10);        
    } else if (millis()-timeWhenLastCalibrated >60000){
        if (currentPosition>(5*angle_max-angle_min)/6) recalibrate(0, 255, calibrationConstant, 10);
        else if (currentPosition<5*(angle_max-angle_min)/12) recalibrate(255, 255, calibrationConstant, 10);        
    } else if (millis()-timeWhenLastCalibrated >120000){
        if (currentPosition>(angle_max-angle_min)/2) recalibrate(0, 255, calibrationConstant, 10);
        else recalibrate(255, 255, calibrationConstant, 10);        
    }
    

    return currentPosition;



    }




