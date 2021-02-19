#include <Motordriver.h>



MotorDriver::MotorDriver(int PIN_ENABLE, int PIN_PHASE){

    PIN_ENABLE=PIN_ENABLE;
    PIN_PHASE=PIN_PHASE;


}
int MotorDriver::getCurrentPosition(){
    return currentPosition;
}


void MotorDriver::setTargetPosition(int target){
    targetPosition=target;
}
void MotorDriver::setSpeed(int speed){
    speed=speed;
}

void MotorDriver::update(){

    int timeSinceLastUpdated=millis()-timeWhenLastUpdated;
    int distanceToNewPosition = abs(newPosition-currentPosition);
    int timeToReachNewPosition = calibrationConstant*distanceToNewPosition/speed;

    if (timeSinceLastUpdated >= timeToReachNewPosition){
        currentPosition=targetPosition;
        analogWrite(PIN_ENABLE, 0); // Turn off motor
        return;
    }


    if (targetPostion != currentPosition){

    // decide direction to move.
    if (targetPosition > currentPosition) digitalWrite(PIN_PHASE, 255); else digitalWrite(PIN_PHASE, 0);
    // set speed:
    analogWrite(PIN_ENABLE, speed);
    // run until new position is reached:
    timeWhenLastUpdated=millis();
  }





    }

}
