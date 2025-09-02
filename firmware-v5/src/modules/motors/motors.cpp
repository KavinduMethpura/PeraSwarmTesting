#include "motors.h"  // Include the motor control class definition

// Create a single global instance of SW_Motors.
// This allows you to call motors.begin(), motors.write(), etc.
// from anywhere in your program without needing to pass references.
SW_Motors motors;
