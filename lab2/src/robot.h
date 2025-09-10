#pragma once

#include "chassis.h"

class Robot
{
protected:
    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE,
        ROBOT_DRIVE_TO_POINT,
    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    /* Define the chassis*/
    Chassis chassis;

    // For managing key presses
    String keyString;

    /**
     * For tracking current pose and the destination.
     */
    Pose currPose;
    Pose destPose;
    
public:
    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);

    const int dests_size = 4;
    const Pose dests_pose[5] = {Pose(50, 0, 0), Pose(40, 20, 0), Pose(10, 10, 0), Pose(10, -30, 0), Pose(50, 0, 0)};
    // const Pose dests_pose[1] = {Pose(50,-200,0)};
    int dests_i = 0;
    int circle = 0;
    

protected:
    /* State changes */    
    void EnterIdleState(void);

    // /* Navigation methods.*/
    void UpdatePose(const Twist& u);
    void SetDestination(const Pose& destination);
    void DriveToPoint(void);
    bool CheckReachedDestination(void);
    void HandleDestination(void);
};
