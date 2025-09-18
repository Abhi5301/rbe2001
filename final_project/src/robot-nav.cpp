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

    //First order calculation
    currPose1.theta += twist.omega;
    currPose1.x += twist.u * cos(currPose1.theta);
    currPose1.y += twist.u * sin(currPose1.theta);

    if(currPose1.theta > M_PI){
        currPose1.theta -= 2*M_PI;
    }
    if(currPose1.theta < -M_PI){
        currPose1.theta += 2*M_PI;
    }

    //Instantanous Circle Calculation

    if(twist.omega == 0){
        currPoseCir.x += twist.u * cos(currPoseCir.theta);
        currPoseCir.y += twist.u * sin(currPoseCir.theta);
    } else {
        float R = (twist.u)/(twist.omega);  
        float startTheta = currPoseCir.theta;
        currPoseCir.theta += twist.omega;
        currPoseCir.x += R * (sin(currPoseCir.theta) - sin(startTheta));
        currPoseCir.y += R * (cos(startTheta) - cos(currPoseCir.theta));

        if(currPoseCir.theta > M_PI){
            currPoseCir.theta -= 2*M_PI;
        }
        if(currPoseCir.theta < -M_PI){
            currPoseCir.theta += 2*M_PI;
        }
    }
    


#ifdef __NAV_DEBUG__
    
    TeleplotPrint("secOrdX", currPose.x);
    TeleplotPrint("secOrdY", currPose.y);
    TeleplotPrint("secOrdTheta", currPose.theta);

    TeleplotPrint("firOrdX", currPose1.x);
    TeleplotPrint("firOrdY", currPose1.y);
    TeleplotPrint("firOrdTheta", currPose1.theta);

    TeleplotPrint("cirX", currPoseCir.x);
    TeleplotPrint("cirY", currPoseCir.y);
    TeleplotPrint("cirTheta", currPoseCir.theta);
    
    TeleplotPrint("secOrdX - firOrderX", currPose.x - currPose1.x);
    TeleplotPrint("secOrdY - firOrderY", currPose.y - currPose1.y);
    TeleplotPrint("secOrdTheta - firOrderTheta", currPose.theta - currPose1.theta);

    TeleplotPrint("secOrdX - cirX", currPose.x - currPoseCir.x);
    TeleplotPrint("secOrdY - cirY", currPose.y - currPoseCir.y);
    TeleplotPrint("secOrdTheta - cirTheta", currPose.theta - currPoseCir.theta);

    TeleplotPrint("firOrdX - cirX", currPose1.x - currPoseCir.x);
    TeleplotPrint("firOrdY - cirY", currPose1.y - currPoseCir.y);
    TeleplotPrint("firOrdTheta - cirTheta", currPose1.theta - currPoseCir.theta);
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

    double deltaX = destPose.x - currPose.x;
    double deltaY = destPose.y - currPose.y;
    double error_r = sqrt(square(deltaX) + square(deltaY));

    if (error_r < 1){
        retVal = true;
    }

    return retVal;
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
        /*
        TeleplotPrint("x", currPose.x);
        TeleplotPrint("y", currPose.y);
        TeleplotPrint("error_r", error_r);
        TeleplotPrint("error_theta", error_theta);
        */
#endif
        //Outputs the efforts determined by the controller to the motors
        chassis.SetMotorEfforts(left_effort, right_effort);
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