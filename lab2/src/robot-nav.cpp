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

    if(currPose.theta > M_PI){
        currPose.theta -= 2*M_PI;
        circle++;
    }
    if(currPose.theta < -M_PI){
        currPose.theta += 2*M_PI;
    }

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

    if (error_r < 3){
        retVal = true;
    }

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        double error_r = sqrt((destPose.x - currPose.x)*(destPose.x - currPose.x) + (destPose.y - currPose.y)*(destPose.y - currPose.y));
        double error_theta = atan2((destPose.y - currPose.y), (destPose.x - currPose.x))-currPose.theta;

        if (error_theta > M_PI){
            error_theta -= 2 * M_PI;
        }
        if (error_theta < -M_PI){
            error_theta += 2 * M_PI;
        }

        double left_effort = 0;
        double right_effort = 0;

        double r_kp = 7; //9; // <- this is a reasonable value, set to 0 for turning tuning
        double theta_kp = 300;

        if(error_r > 50){
            error_r = 50;
        }
        if(abs(error_theta) > 50){
            if(error_theta > 0){
                error_theta = 50;
            } else {
                error_theta = -50;
            }
        }

        left_effort += error_r * r_kp*abs(cos(error_theta));
        right_effort += error_r * r_kp*abs(cos(error_theta));

        /*
        if(abs(error_theta)<0.4){
            left_effort += error_r * r_kp*abs(cos(error_theta));
            right_effort += error_r * r_kp*abs(cos(error_theta));
        } else {
            left_effort += error_r * r_kp*abs(cos(error_theta))*abs(cos(error_theta-0.4))*0.2;
            right_effort += error_r * r_kp*abs(cos(error_theta))*abs(cos(error_theta-0.4))*0.2;
        }
        */

        left_effort -= error_theta * theta_kp;
        right_effort += error_theta * theta_kp;

        if (abs(left_effort) > 150){
            if(left_effort > 0){
                left_effort = 150;
            } else {
                left_effort = -150;
            }
        }

        if (abs(right_effort) > 150){
            if(right_effort > 0){
                right_effort = 150;
            } else {
                right_effort = -150;
            }
        }

        

        /**
         * TODO: Add your IK algorithm here. 
         */

#ifdef __NAV_DEBUG__
        // Print useful stuff here.
        TeleplotPrint("x", currPose.x);
        TeleplotPrint("y", currPose.y);
        TeleplotPrint("error_r", error_r);
        TeleplotPrint("error_theta", error_theta);
#endif

        chassis.SetMotorEfforts(left_effort, right_effort);
        /*
        left_effort = 40;
        right_effort = 70;

        if(circle == 1 && currPose.theta > 0){
            EnterIdleState();
        } else{
            chassis.SetMotorEfforts(left_effort, right_effort);
        }
        
        TeleplotPrint("x", currPose.x);
        TeleplotPrint("y", currPose.y);
        TeleplotPrint("theta", currPose.theta);
        */
    }
}

void Robot::HandleDestination(void)
{
    EnterIdleState();
    if(dests_i == dests_size) {
        robotState = ROBOT_IDLE;
    } else {
        dests_i++;
        SetDestination(dests_pose[dests_i]);
    }  
}