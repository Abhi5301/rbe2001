/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"

void Robot::UpdatePose(const Twist& twist)
{
    //Second order calculation

    //Incrementing by half the timestep to use the average omega for the x and y calculations
    currPose.theta += twist.omega * 0.5;
    //Uses the forward distance travelled (u) and transforms it into the lab reference frame using theta
    currPose.x += twist.u * cos(currPose.theta);
    currPose.y += twist.u * sin(currPose.theta); 
    // Incrementing by the other half of the timestep to complete the second order calculation
    currPose.theta += twist.omega * 0.5;

    //Handles the heading discontinuity and wraps the theta to between 
    if(currPose.theta > M_PI){
        currPose.theta -= 2*M_PI;
        circle++;
    }
    if(currPose.theta < -M_PI){
        currPose.theta += 2*M_PI;
    }


#ifdef __NAV_DEBUG__
    
    TeleplotPrint("secOrdX", currPose.x);
    TeleplotPrint("secOrdY", currPose.y);
    TeleplotPrint("secOrdTheta", currPose.theta);

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
    double deltaX = destPose.x - currPose.x;
    double deltaY = destPose.y - currPose.y;
    double error_r = sqrt(square(deltaX) + square(deltaY));

    if (error_r < 3){
        return true;
    }

    return false;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        double deltaX = destPose.x - currPose.x;
        double deltaY = destPose.y - currPose.y;
        double error_r = sqrt(square(deltaX) + square(deltaY));
        double error_theta = atan2((deltaY), (deltaX)) - currPose.theta;

        //Handles the heading discontinuity and wraps the theta to between ±π; 
        if (error_theta > PI){
            error_theta -= 2 * PI;
        }
        if (error_theta < -PI){
            error_theta += 2 * PI;
        }

        double left_effort = 0;
        double right_effort = 0;

        double r_kp = 20;
        double theta_kp = 2000;

        //Forward component of the proportional controller
        left_effort += error_r * r_kp*abs(cos(error_theta));
        right_effort += error_r * r_kp*abs(cos(error_theta)); 
        /*
            Note: The abs(cos(error_theta)) is multiplied in to 
            help weight the robot to drive less if it is facing 
            far away from its target. This works because if the 
            theta error is high, it will return a small out from 
            cos (like 0.1) which will scale the romi's forward 
            velocity until it aligns better with its goal.
        
        */

        //Turning component of the proportional controller
        left_effort -= error_theta * theta_kp;
        right_effort += error_theta * theta_kp;


        //Caps the max left and right motor efforts to limit 
        //the romi speed to reduce spliage and increase reliability
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


#ifdef __NAV_DEBUG__
        
        TeleplotPrint("x", currPose.x);
        TeleplotPrint("y", currPose.y);
        TeleplotPrint("error_r", error_r);
        TeleplotPrint("error_theta", error_theta);
        
#endif
        //Outputs the efforts determined by the controller to the motors
        chassis.SetMotorEfforts(left_effort, right_effort);
    }
}

void Robot::HandleDestination(void)
{
    if(dests_i == dests_size) {
        EnterIdleState();
    } else {
        SetDestination(dests_pose[dests_i]);
        dests_i++;
    }  
}