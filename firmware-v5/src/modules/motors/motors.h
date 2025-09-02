#pragma once
// Prevents multiple inclusion of this header during compilation

#include "./src/robot_motors.h"
// Brings in the full SW_Motors class definition

extern SW_Motors motors;
// Declares that there is a global SW_Motors object called 'motors'
// The actual object is defined elsewhere (e.g., in motors.cpp as `SW_Motors motors;`)
