#include "robot.h"


void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();

    //Initializing motors
    servoPin5.attach();
    servoPin6.attach();
    blueMotor.setup();
    blueMotor.reset();

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

bool Robot::checkReached() {
    bool returnVal = true;
    if(abs(bluemotortarget - blueMotor.getPosition()) > 20){
        return false;
    }

    return returnVal;
}

bool Robot::doNextTask(){
    //servo5 - slider: 0, 2, 4, 6, 8, 10, 12, 14
    //servo6 - grabber: 1, 5, 9, 13
    //blue: 3, 7, 11, 15
    if(task_i == (0 || 2 || 4 || 6 || 8 || 10 || 12 || 14)){
        //its a servo5 - slider movement
        servo5target = taskPos[task_i];
        servoPin5.setTargetPos(servo5target);
        servoPin5.update();
        delay(500);
    }
    if (task_i == (1 || 5 || 9 || 13)){
        //its a servo 6 - grabber movement
        servo6target = taskPos[task_i];
        servoPin6.setTargetPos(servo6target);
        servoPin6.update();
        delay(500);
    }
    if (task_i == (3 || 7 || 11 || 15)){
        //its a blue motor move
        bluemotortarget = taskPos[task_i];
    }
    
    if (task_i == 16)
    {
        return true;
    }
    
    task_i++;
    return false;
    
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

     //TODO: loop to handle blue motor checking current blue motor pose
     //then doing a porportional loop based off of blue motors current 
     //position and also the bluemotortarget global variable which will 
     //house the blue motor's target position

    Twist velocity;
    if(chassis.ChassisLoop(velocity))
    {
        // We do FK regardless of state
        UpdatePose(velocity);
         
        if(robotState == ROBOT_TASK && timerTask.CheckExpired())
        {
            if(checkReached()){
                if(doNextTask()){   //to do out auton, set the target positions in the array in robot.h for the coresponding actuators
                    robotState == ROBOT_DRIVE_TO_POINT;
                }
            }
            timerTask.start(50);
            

        } else if(robotState == ROBOT_DRIVE_TO_POINT) {
            DriveToPoint();
            if(CheckReachedDestination()){ HandleDestination();};
        }
    }
    
}
