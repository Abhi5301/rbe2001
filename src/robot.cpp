#include "robot.h"


void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();

    servoPin5.attach();

    servoPin5.setTargetPos(1500);
    SetDestination(dests_pose[dests_i]);
    robotState = ROBOT_TASK;
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
        // if(currPose.theta < 62.82){
        //     chassis.SetMotorEfforts(-80,80);
        // } else {
        //     chassis.SetMotorEfforts(0,0);
        // }
        /*if(robotState == ROBOT_IDLE) {
            if(digitalRead(14) == LOW){
                delay(2000);
                SetDestination(dests_pose[dests_i]);
                robotState = ROBOT_DRIVE_TO_POINT;
            }
        } */
        /**
         * Here, we break with tradition and only call these functions if we're in the 
         * DRIVE_TO_POINT state. CheckReachedDestination() is expensive, so we don't want
         * to do all the maths when we don't need to.
         * 
         * While we're at it, we'll toss DriveToPoint() in, as well.
         */ 
        if(robotState == ROBOT_TASK)
        {
            servo5.update();
            servo6.update();
            handle_blueMotor_stuff();
            

        } else if(robotState == ROBOT_DRIVE_TO_POINT) {
            DriveToPoint();
            if(CheckReachedDestination()){ HandleDestination();};
        }
    }
}
