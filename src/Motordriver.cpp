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



void MotorDriver::calibrateIfNeeded(){


    if (millis()-timeWhenLastCalibrated >10000){
        
        if (currentPosition>=angle_max-minimumStepSize*8 && direction != -1 && previousCalibrationDir != 1) 
        {
            recalibrate(255, speed/4, calibrationConstant, 0.1);
            previousCalibrationDir = 1;
            return;

        }
        else if (currentPosition<=angle_min+minimumStepSize*8 && direction != 1 && previousCalibrationDir != -1){
            recalibrate(0, speed/4, calibrationConstant, 0.1);
            previousCalibrationDir = -1;
            return;

        } 

    }else {
        

        if (currentPosition>=angle_max-minimumStepSize*4 && direction != -1 && previousCalibrationDir != 1) 
        {
            recalibrate(255, speed/4, calibrationConstant, 0.05);
            previousCalibrationDir = 1;
            return;
            

        }
        else if (currentPosition<=angle_min+minimumStepSize*4 && direction != 1 && previousCalibrationDir != -1){
            recalibrate(0, speed/4, calibrationConstant, 0.05);
            previousCalibrationDir = -1;
            return;
            

        } 

    }

}


bool MotorDriver::calibrationHappeningNow(){


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

        return true;
    }
    return false;

}



int MotorDriver::update(){


    if (calibrationHappeningNow()){
        return currentPosition;
    }
    if (!init){
        timeWhenLastUpdated=micros();
        init=true;
    }

    unsigned long timeSinceLastUpdated=micros()-timeWhenLastUpdated;
    
    float degreesMovedSinceLastUpdate=timeSinceLastUpdated*180.0/(calibrationConstant*1000.0)*currentSpeed/speed;

    if (degreesMovedSinceLastUpdate>0) timeWhenLastUpdated=micros();

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
        if ( abs(currentPosition-targetPositionRounded) < minimumStepSize*2 || ( currentPosition > angle_max-minimumStepSize*4 && direction == 1 ) || (currentPosition < angle_min+minimumStepSize*4 && direction == -1 ) ){
            currentSpeed=speed/6;

        }else {
            currentSpeed=speed;
            
        }
        analogWrite(PIN_ENABLE, currentSpeed);
    }

    calibrateIfNeeded();



    return currentPosition;



    }




