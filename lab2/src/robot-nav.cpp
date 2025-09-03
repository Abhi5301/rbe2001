/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"



void Robot::UpdatePose(const Twist& twist)
{
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */
    currPose.theta = currPose.theta + twist.omega * 0.5; //incrementing by half the timestep to use the average omega for the x and y calculations
    currPose.x = currPose.x + twist.u * cos(currPose.theta);
    currPose.y = currPose.y + twist.v * sin(currPose.theta); 
    currPose.theta = currPose.theta + twist.omega * 0.5; // incrementing by the other half

    /*
        TODO:it
        use the current u forward delta in cm in the robot reference frame and the corresponding difference in angle
        we take this and brreak down into forwards and sideways differences in the robots position in the robots reference frame
        take these forward and sideways deltats and do the offcoordinate transformation from robot space to global space
        recompute location from global space offsets (math above is wrong)
    */


#ifdef __NAV_DEBUG__
    TeleplotPrint("x", currPose.x);
    TeleplotPrint("y", currPose.y);
    TeleplotPrint("theta", currPose.theta);
#endif

}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose& dest)
{
    /**
     * TODO: Turn on LED, as well.
     */
    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

bool Robot::CheckReachedDestination(void)
{
    bool retVal = false;
    /**
     * TODO: Add code to check if you've reached destination here.
     */

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        /**
         * TODO: Add your IK algorithm here. 
         */

#ifdef __NAV_DEBUG__
        // Print useful stuff here.
#endif

        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */
    }
}

void Robot::HandleDestination(void)
{
    /**
     * TODO: Stop and change state. Turn off LED.
     */
}