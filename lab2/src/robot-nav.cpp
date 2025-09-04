/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"

void Robot::UpdatePose(const Twist& twist)
{

    currPose.theta += twist.omega * 0.5; //incrementing by half the timestep to use the average omega for the x and y calculations
    currPose.x += twist.u * cos(currPose.theta);
    currPose.y += twist.u * sin(currPose.theta); 
    currPose.theta += twist.omega * 0.5; // incrementing by the other half

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
    double error_r = sqrt((destPose.x - currPose.x)*(destPose.x - currPose.x) + (destPose.y - currPose.y)*(destPose.y - currPose.y));
    if (error_r < 5){
        retVal = true;
    }

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        double error_r = sqrt((destPose.x - currPose.x)*(destPose.x - currPose.x) + (destPose.y - currPose.y)*(destPose.y - currPose.y));
        double error_theta = atan2((destPose.y - currPose.y), (destPose.x - currPose.x));

        double left_effort = 0;
        double right_effort = 0;

        double r_kp = 2;
        double theta_kp = 1;

        left_effort = error_r * r_kp;// + error_theta * theta_kp;
        right_effort = error_r * r_kp;// - error_theta * theta_kp;

        /**
         * TODO: Add your IK algorithm here. 
         */

#ifdef __NAV_DEBUG__
        // Print useful stuff here.
        TeleplotPrint("error_r", currPose.x);
        TeleplotPrint("error_theta", currPose.x);
#endif

        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */
        //chassis.SetMotorEfforts(left_effort, right_effort);
    }
}

void Robot::HandleDestination(void)
{
    dests_i++;
    SetDestination(dests_pose[dests_i]);  
}