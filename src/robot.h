#pragma once

#include "chassis.h"
#include "servo32u4.h"
#include "BlueMotor.h"

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
        ROBOT_TASK,
    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    /* Define the chassis*/
    Chassis chassis;

    EventTimer timerTask;
    
    Servo32U4Pin5 servoPin5;
    Servo32U4Pin12 servoPin12;
    BlueMotor blueMotor;
    
    // For managing key presses
    String keyString;

    /**
     * For tracking current pose and the destination.
     */
    Pose currPose;
    Pose currPoseCir;
    Pose currPose1;
    Pose destPose;
    
public:
    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);

    const int dests_size = 5;
    const Pose dests_pose[6] = {Pose(50, -25, 0), Pose(-40, 10, 0), Pose(25, 20, 0), Pose(-15, -20, 0), Pose(-40, -10, 0), Pose(0, 0, 0)};
    // const Pose dests_pose[1] = {Pose(50,-200,0)};
    int dests_i = 0;
    int circle = 0;

    //Servo targets
    int servo5target;
    int servo12target;
    int bluemotortarget; 

    //Blue motor target
    long currentPos = 0;
    long oldPosition = 0;

    double ki = 0.05; //%5-%10 of Kp


    int task_i = 0;

    //servo5 - slider: 0, 2, 4, 6, 8, 10, 12, 14
    //servo6 - grabber: 1, 5, 9, 13
    //blue: 3, 7, 11, 15

    #define slideIn 2000
    #define slideOut 1200
    #define closeGrip 1950
    #define openGrip 1300
    #define ground 0
    #define shelf1 1700
    #define shelf2 2100
    #define shelf3 2500

    int servo5Pos[16] =     {slideOut,  slideOut,   slideIn,    slideIn,    slideOut,   slideOut,   slideIn,    slideIn,    slideOut,   slideOut,   slideIn,    slideIn,    slideOut,   slideOut,   slideIn,    slideIn};
    int servo12Pos[16] =    {openGrip,  closeGrip,  closeGrip,  closeGrip,  closeGrip,  openGrip,   openGrip,   openGrip,   openGrip,   closeGrip,  closeGrip,  closeGrip,  closeGrip,  openGrip,   openGrip,   openGrip};
    int bluePos[16] =       {ground,    ground,     ground,     shelf1,     shelf1,     shelf1,     shelf1,     shelf2,     shelf2,     shelf2,     shelf2,     shelf3,     shelf3,     shelf3,     shelf3,     ground};
    

protected:
    /* State changes */    
    void EnterIdleState(void);

    // /* Navigation methods.*/
    void UpdatePose(const Twist& u);
    void SetDestination(const Pose& destination);
    void DriveToPoint(void);
    bool CheckReachedDestination(void);
    void HandleDestination(void);
    bool checkReached(void);
    bool doNextTask(void);

};
