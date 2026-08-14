#ifndef STEPPER_CONTROLLER_H
#define STEPPER_CONTROLLER_H

#include <AccelStepper.h>


class StepperController {
public:
       StepperController(AccelStepper& stepperMotor, int stepsPerPos, int homingPin, int homingOffsetSteps, bool invertDirection);
    void initialize();
    void moveToDigit(int targetDigit);
    void moveToDigitBackward(int targetDigit);

    // returns true if the motor is still moving
    bool run();
    
    bool runHoming();

private:
    enum HomingState {
        UNHOMED,
        CLEARING,
        HOMING,
        OFFSETTING,
        HOMED
    };
    AccelStepper& motor_;
    int targetDigit_;  // 0-9, the digit we are moving toward
    int stepsPerPosition_;
    int homingPin_;
    int homingOffsetSteps_;
    bool invertDirection_;
    HomingState homingState_ = UNHOMED;
    void resetStepCountIfStationary();
};

#endif // STEPPER_CONTROLLER_H