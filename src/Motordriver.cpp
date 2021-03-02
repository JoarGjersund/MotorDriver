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




void MotorDriver::recalibrate(int calibrationdirection, int calibrationspeed, int calibrationconstant_new, int percent_delay){


    Serial.println("calibrating...");
    digitalWrite(PIN_PHASE, calibrationdirection);
    analogWrite(PIN_ENABLE, calibrationspeed);
    

    if (calibrationdirection==0) currentPosition=angle_min;
    else currentPosition=angle_max;

    if (calibrationconstant_new>0) calibrationConstant = calibrationconstant_new;
    
    calibrationDelayTime= round(percent_delay*(1/100.0)*calibrationConstant*(255.0/calibrationspeed));
    calibrationTimeStart = millis();
    calibrationInProgress=true;

}



void MotorDriver::setTargetPosition(int target){
    if (target > angle_max) target=angle_max;
    if (target < angle_min) target=angle_min;
    targetPosition=target;

}







int MotorDriver::update(){


    if (calibrationInProgress){

        if (millis()-calibrationTimeStart < calibrationDelayTime){

            Serial.println(String(millis()-calibrationTimeStart-calibrationDelayTime)); 
            
        }else {
            calibrationInProgress=false;
            direction=0;
            analogWrite(PIN_ENABLE, 0);
            timeWhenLastCalibrated = millis();

        }

        return currentPosition;
    }


    static bool init = false;
    if (!init){
        timeWhenLastUpdated=millis();
        init=true;
    }

    unsigned long timeSinceLastUpdated=millis()-timeWhenLastUpdated;
    


    int degreesMovedSinceLastUpdate=timeSinceLastUpdated*180.0/(calibrationConstant);

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

        Serial.println("should calibrate..."+String(currentPosition));
        if (currentPosition>=angle_max-minimumStepSize) recalibrate(0, 5, calibrationConstant, 1);
        else if (currentPosition<=angle_min+minimumStepSize) recalibrate(255, 5, calibrationConstant, 1);    

    } else if (millis()-timeWhenLastCalibrated >10000){

        if (currentPosition>(2*angle_max-angle_min)/3-minimumStepSize) recalibrate(0, 10, calibrationConstant, 6);
        else if (currentPosition<(angle_max-angle_min)/3+minimumStepSize) recalibrate(255, 10, calibrationConstant, 6);       

    } else if (millis()-timeWhenLastCalibrated >60000){

        if (currentPosition>(5*angle_max-angle_min-minimumStepSize)/6) recalibrate(0, 20, calibrationConstant, 8);
        else if (currentPosition<5*(angle_max-angle_min)/12+minimumStepSize) recalibrate(255, 20, calibrationConstant, 8);  

    } else if (millis()-timeWhenLastCalibrated >120000){

        if (currentPosition>(angle_max-angle_min)/2-minimumStepSize) recalibrate(0, 35, calibrationConstant, 10);
        else recalibrate(255, 255, calibrationConstant, 10);     

    }
    

    return currentPosition;



    }




