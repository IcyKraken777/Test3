#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LF = motor(PORT8, ratio6_1, true);
motor LM = motor(PORT9, ratio6_1, true);
motor RF = motor(PORT10, ratio6_1, true);
motor RM = motor(PORT3, ratio6_1, false);
motor LB = motor(PORT7, ratio6_1, false);
motor RB = motor(PORT18, ratio6_1, false);
//motor Intake = motor(PORT9, ratio6_1, false);
motor SecondStage = motor(PORT19, ratio6_1, false);
motor Roller = motor(PORT15, ratio6_1, false);
digital_out Wings = digital_out(Brain.ThreeWirePort.H);
digital_out Tilt = digital_out(Brain.ThreeWirePort.A);
digital_out Clamp = digital_out(Brain.ThreeWirePort.B);
inertial Gyro = inertial(PORT1);
//Naming convention: 
// Important variables
const double wheelDiam = 2.75;
const double wheelToMotorRatio = 48.0/36;

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}