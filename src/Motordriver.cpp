#include <Motordriver.h>


MotorDriver::MotorDriver(int pin_enable, int pin_phase, int min_angle = 0, int max_angle = 180){

    PIN_ENABLE=pin_enable;
    PIN_PHASE=pin_phase;
    angle_max=max_angle;
    angle_min=min_angle;
    offset=(angle_max-angle_min)/2+angle_min;

    amplitude=(angle_max-angle_min)/2;


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


    if (millis()-timeWhenLastCalibrated >5000){

        if (millis()-counter_calibration1 > 1000){
            counter_calibration2++;
            if (counter_calibration2>angle_max) counter_calibration2 = 1;
            counter_calibration1 = millis();
        }
        

        
        if (currentPosition>=angle_max-counter_calibration2 && direction != -1) 
        {
            float timeToReach = 2*abs(angle_max-currentPosition)/angle_max;
            recalibrate(255, speed/6, calibrationConstant, timeToReach);
            return;

        }
        else if (currentPosition<=angle_min+counter_calibration2 && direction != 1){
            float timeToReach = 2*abs(angle_min-currentPosition)/angle_max;
            recalibrate(0, speed/6, calibrationConstant, timeToReach);
            return;

        } 

    } else if (millis()-timeWhenLastCalibrated >2000){
        

        if (currentPosition>=angle_max-minimumStepSize*2 && direction != -1) 
        {
            recalibrate(255, speed/6, calibrationConstant, 0.05);
            return;
            

        }
        else if (currentPosition<=angle_min+minimumStepSize*2 && direction != 1){
            recalibrate(0, speed/6, calibrationConstant, 0.05);
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
            counter_calibration2 = 1; // reset offset from calibration threshold angle 
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
    
    float degreesMovedSinceLastUpdate=timeSinceLastUpdated*180.0/(calibrationConstant*1000.0);

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
        currentSpeed=0;
        direction=0;

        analogWrite(PIN_ENABLE, currentSpeed);  

    }else{

        if ( (currentPosition > angle_max-minimumStepSize && direction==1 ) || (currentPosition < angle_min+minimumStepSize && direction==-1) )
        {
            
             currentSpeed=speed/(minimumStepSize);
        }
        else{
            currentSpeed = speed;
        }

        analogWrite(PIN_ENABLE, currentSpeed);
    }

    calibrateIfNeeded();



    return currentPosition;



    }




