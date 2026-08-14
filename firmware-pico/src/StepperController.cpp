#include "StepperController.h"
#include "Configuration.h"

#define MIN_DIGIT              0
#define MAX_DIGIT              9
#define DIGITS_PER_WHEEL       10

StepperController::StepperController(AccelStepper& stepperMotor, int stepsPerPos, int homingPin, int homingOffsetSteps, bool invertDirection)
    : motor_(stepperMotor), targetDigit_(0), stepsPerPosition_(stepsPerPos), homingPin_(homingPin), homingOffsetSteps_(homingOffsetSteps), invertDirection_(invertDirection) {
    motor_.setMaxSpeed(MOTOR_MAX_SPEED);
    motor_.setAcceleration(MOTOR_ACCELERATION);
    motor_.setPinsInverted(invertDirection_);
}

void StepperController::initialize() {
    pinMode(homingPin_, INPUT_PULLUP); // Active low with pullup
    homingState_ = UNHOMED;
}

// Homing state machine
bool StepperController::runHoming() {
    switch (homingState_) {
        case UNHOMED:
            // Start clearing
            motor_.move(stepsPerPosition_ * DIGITS_PER_WHEEL * 2); // command two full revolutions forward
            homingState_ = CLEARING;
            break;
        case CLEARING:
            // Move until endstop is de-asserted
            if (digitalRead(homingPin_) == LOW) {
                motor_.run();
            } else {
                homingState_ = HOMING;
            }
            break;
        case HOMING:
            // Move until endstop is asserted
            if (digitalRead(homingPin_) == HIGH) {
                motor_.run();
            } else {
                motor_.move(homingOffsetSteps_); // move past the endstop
                homingState_ = OFFSETTING;
            }
            break;
        case OFFSETTING:
            if (motor_.distanceToGo() != 0) {
                motor_.run();
            } else {
                targetDigit_ = 0;
                motor_.setCurrentPosition(0);
                homingState_ = HOMED;
            }
            break;
        case HOMED:
            return true;
    }
    return false;
}

void StepperController::moveToDigit(int targetDigit) {
    // sanity check to avoig big ugly moves if something goes terribly wrong
    if (targetDigit < MIN_DIGIT || targetDigit > MAX_DIGIT) return;

    // drop out if already at target
    if (targetDigit == targetDigit_) return;

    // reset step count if stationary to avoid step count overflow over long runtimes
    resetStepCountIfStationary();

    // use current target digit to figure how how far to move (forward only)
    int forwardDigits = (targetDigit - targetDigit_);
    if (forwardDigits < 0) forwardDigits += DIGITS_PER_WHEEL;

    // calculate new step target and command move
    long newStepTarget = motor_.targetPosition() + (forwardDigits * stepsPerPosition_);
    motor_.moveTo(newStepTarget);

    // remember where we are going
    targetDigit_ = targetDigit;
}

void StepperController::moveToDigitBackward(int targetDigit) {
    // sanity check to avoig big ugly moves if something goes terribly wrong
    if (targetDigit < MIN_DIGIT || targetDigit > MAX_DIGIT) return;

    // drop out if already at target
    if (targetDigit == targetDigit_) return;

    // reset step count if stationary to avoid step count overflow over long runtimes
    resetStepCountIfStationary();

    // use current target digit to figure how far to move (backward only)
    int backwardDigits = (targetDigit_ - targetDigit);
    if (backwardDigits < 0) backwardDigits += DIGITS_PER_WHEEL;

    // calculate new step target and command move
    long newStepTarget = motor_.targetPosition() - (backwardDigits * stepsPerPosition_);
    motor_.moveTo(newStepTarget);

    // remember where we are going
    targetDigit_ = targetDigit;
}

bool StepperController::run() {
    motor_.run();
    return motor_.distanceToGo() != 0;
}

void StepperController::resetStepCountIfStationary() {
    // if the motor isn't moving, this is a good time to reset the current
    // and target positions to zero to avoid step count overflow over long runtimes
    if (motor_.distanceToGo() == 0) {
        motor_.setCurrentPosition(0);
    }
}