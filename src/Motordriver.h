class MotorDriver {
    public:
    MotorDriver(int PIN_ENABLE, int PIN_PHASE);
    void update();
    void setTargetPosition(int target);
    int getCurrentPosition();
    int targetPosition;
    int calibrationConstant=200; // slower motor -> Higher number

    private:
    int currentPositon;
    int speed;
    int timeWhenLastUpdated;
    int PIN_ENABLE;
    int PIN_PHASE;

};