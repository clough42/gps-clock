#include "MechanicalDisplay.h"

MechanicalDisplay::MechanicalDisplay(StepperController& hoursTens,
                                                StepperController& hoursOnes,
                                                StepperController& minutesTens,
                                                StepperController& minutesOnes,
                                                StepperController& secondsTens,
                                                StepperController& secondsOnes,
                                                int enablePin,
                                                int debugPin)
        : hoursTens_(hoursTens), hoursOnes_(hoursOnes),
            minutesTens_(minutesTens), minutesOnes_(minutesOnes), secondsTens_(secondsTens), secondsOnes_(secondsOnes), enablePin_(enablePin), debugPin_(debugPin) {
}

void MechanicalDisplay::initialize() {
    // Initialize shared motor enable pin (active low)
    pinMode(enablePin_, OUTPUT);
    digitalWrite(enablePin_, LOW);  // Enable all motors

    // Initialize debug pin for timing measurement
    pinMode(debugPin_, OUTPUT);
    digitalWrite(debugPin_, LOW);  // Start low

    // Initialize all stepper controllers
    hoursTens_.initialize();
    hoursOnes_.initialize();
    minutesTens_.initialize();
    minutesOnes_.initialize();
    secondsTens_.initialize();
    secondsOnes_.initialize();

    // home the motors
    homeAllMotors();
}

void MechanicalDisplay::homeAllMotors() {
    bool allHomed = false;
    do {
        allHomed = true;
        allHomed &= hoursTens_.runHoming();
        allHomed &= hoursOnes_.runHoming();
        allHomed &= minutesTens_.runHoming();
        allHomed &= minutesOnes_.runHoming();
        allHomed &= secondsTens_.runHoming();
        allHomed &= secondsOnes_.runHoming();
        rp2040.wdt_reset(); // let the watchdog timer know we're still alive
    } while (!allHomed);
}

// This method is called from the main core when we have new time data to display.
void MechanicalDisplay::updateTime(const TimeData& timeData) {
    // Only update if time is valid
    if (timeData.validTime) {
        // shove the digits into the FIFO for the other core
        rp2040.fifo.push(
            timeData.localHours * 10000 +
            timeData.localMinutes * 100 +
            timeData.localSeconds);
    }
}

// This method is called from the second core's main loop to service the motors.
bool MechanicalDisplay::runMotors() {
    // if we have time data from the other core
    if(rp2040.fifo.available()) {
        // Decode and update the motors
        uint32_t timeDataInt = rp2040.fifo.pop();
        hoursTens_.moveToDigitBackward(timeDataInt   / 100000 % 10);
        hoursOnes_.moveToDigitBackward(timeDataInt   / 10000 % 10);
        minutesTens_.moveToDigitBackward(timeDataInt / 1000 % 10);
        minutesOnes_.moveToDigitBackward(timeDataInt / 100 % 10);
        secondsTens_.moveToDigitBackward(timeDataInt / 10 % 10);
        secondsOnes_.moveToDigitBackward(timeDataInt % 10);
    }

    digitalWrite(debugPin_, HIGH);  // Start timing measurement
    
    bool anyMotorsRunning = false;
    anyMotorsRunning |= hoursTens_.run();
    anyMotorsRunning |= hoursOnes_.run();
    anyMotorsRunning |= minutesTens_.run();
    anyMotorsRunning |= minutesOnes_.run();
    anyMotorsRunning |= secondsTens_.run();
    anyMotorsRunning |= secondsOnes_.run();
    
    digitalWrite(debugPin_, LOW);   // End timing measurement

    return anyMotorsRunning;
}

