#include "robot.h"

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();

    /**
     * TODO: Set pin 13 HIGH when navigating and LOW when destination is reached.
     * Need to set as OUTPUT here.
     */
}

void Robot::EnterIdleState(void)
{
    chassis.Stop();

    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;
}

/**
 * The main loop for your robot. Process both synchronous events (motor control),
 * and asynchronous events (distance readings, etc.).
*/
void Robot::RobotLoop(void) 
{
     /**
     * Run the chassis loop, which handles low-level control.
     */
    Twist velocity;
    if(chassis.ChassisLoop(velocity))
    {
        // We do FK regardless of state
        UpdatePose(velocity);
        chassis.SetMotorEfforts(220,-220);
        
        /**
         * Here, we break with tradition and only call these functions if we're in the 
         * DRIVE_TO_POINT state. CheckReachedDestination() is expensive, so we don't want
         * to do all the maths when we don't need to.
         * 
         * While we're at it, we'll toss DriveToPoint() in, as well.
         */ 
        if(robotState == ROBOT_DRIVE_TO_POINT)
        {
            DriveToPoint();
            if(CheckReachedDestination()) HandleDestination();
        }
    }
}
