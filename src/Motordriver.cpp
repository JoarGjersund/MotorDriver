#include <Motordriver.h>


MotorDriver::MotorDriver(int pin_enable, int pin_phase, int min_angle = 0, int max_angle = 180){

    PIN_ENABLE=pin_enable;
    PIN_PHASE=pin_phase;
    angle_max=max_angle;
    angle_min=min_angle;
    offset=(angle_max-angle_min)/2+angle_min;

    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_PHASE, OUTPUT);


}
bool MotorDriver::setCalibrationConstant(int newConstant){
    calibrationConstant=newConstant;
    return true;
}

void MotorDriver::setMinimumStepSize(int newMinimumStepSize){
    minimumStepSize=newMinimumStepSize;
    return;
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




void MotorDriver::recalibrate(int calibrationdirection, int calibrationspeed, int calibrationconstant_new, float delay_factor){


    Serial.println("calibrating...");
    digitalWrite(PIN_PHASE, calibrationdirection);
    analogWrite(PIN_ENABLE, calibrationspeed);
    
    

    if (calibrationdirection==0) currentPosition=angle_min;
    else currentPosition=angle_max;

    if (calibrationconstant_new>0) calibrationConstant = calibrationconstant_new;
    
    calibrationDelayTime= round(delay_factor*calibrationConstant*(255.0/calibrationspeed))*1000;
    calibrationTimeStart = micros();
    calibrationInProgress=true;

}



void MotorDriver::setTargetPosition(int target){
    if (target > angle_max) target=angle_max;
    if (target < angle_min) target=angle_min;
    targetPosition=target;
    #ifdef DEBUG
    Serial.println(String(targetPosition));
    #endif

}







int MotorDriver::update(){





    if (calibrationInProgress){

        if (micros()-calibrationTimeStart < calibrationDelayTime){
            #if DEBUG
            Serial.println(String(micros()-calibrationTimeStart-calibrationDelayTime));
            #endif
            
        }else {
            calibrationInProgress=false;
            direction=0;
            analogWrite(PIN_ENABLE, 0);
            timeWhenLastCalibrated = millis();
            init=false;

        }

        return currentPosition;
    }





    if (!init){
        timeWhenLastUpdated=millis();
        init=true;
    }

    unsigned long timeSinceLastUpdated=millis()-timeWhenLastUpdated;
    


    int degreesMovedSinceLastUpdate=timeSinceLastUpdated*180.0/(calibrationConstant);

    if (degreesMovedSinceLastUpdate>0) timeWhenLastUpdated=millis();

    currentPosition+=degreesMovedSinceLastUpdate*direction*acceleration;

    int targetPositionRounded=round(targetPosition/minimumStepSize)*minimumStepSize;

        // decide direction to move.
    if ( targetPositionRounded > currentPosition && currentPosition < angle_max){
        if (direction !=1) acceleration=0;
        else if (acceleration < 1)acceleration+=acceleration_factor;
        direction=1;
        digitalWrite(PIN_PHASE, 255); 
    } else if ( targetPositionRounded < currentPosition && currentPosition > angle_min){
        if (direction !=-1) acceleration=0;
        else if (acceleration < 1)acceleration+=acceleration_factor;

        direction=-1;
        digitalWrite(PIN_PHASE, 0);
    }

    if (abs(currentPosition-targetPositionRounded) <  minimumStepSize || currentPosition > angle_max || currentPosition < angle_min )
    {
        
        direction=0;
        analogWrite(PIN_ENABLE, 0);  

    }else{
        if (currentPosition > angle_max-degreesMovedSinceLastUpdate*2 || currentPosition < angle_min+degreesMovedSinceLastUpdate*2 ){
            analogWrite(PIN_ENABLE, speed/2); // in case we go to far we do not want to break the motor.
        }else {
            analogWrite(PIN_ENABLE, speed);
        }
        
    }


    if (millis()-timeWhenLastCalibrated >5000){

        
        Serial.println("should calibrate...");
        if (currentPosition>=angle_max-minimumStepSize*2 && direction != -1) 
        {
            recalibrate(255, 50, calibrationConstant, 0.1);
            return currentPosition;

        }
        else if (currentPosition<=angle_min+minimumStepSize*2 && direction != 1){
            recalibrate(0, 50, calibrationConstant, 0.1);
            return currentPosition;

        } 

    }else {

        if (currentPosition>=angle_max-minimumStepSize && direction != -1) 
        {
            recalibrate(255, 50, calibrationConstant, 0.05);
            return currentPosition;

        }
        else if (currentPosition<=angle_min+minimumStepSize && direction != 1){
            recalibrate(0, 50, calibrationConstant, 0.05);
            return currentPosition;

        } 

    }

    return currentPosition;



    }




