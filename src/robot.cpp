#include "robot.h"


void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();

    //Initializing motors
    servoPin5.attach();
    servoPin12.attach();
    blueMotor.setup();
    blueMotor.reset();

    servoPin5.setTargetPos(slideIn);
    servoPin12.setTargetPos(openGrip);
}

void Robot::EnterIdleState(void)
{
    chassis.Stop();

    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;
}

bool Robot::checkReached() {
    bool returnVal = false;
    if(abs(bluemotortarget - blueMotor.getPosition()) < 20){
        return true;
    }

    return returnVal;
}

bool Robot::doNextTask(){
    //servo5 - slider: 0, 2, 4, 6, 8, 10, 12, 14
    //servo6 - grabber: 1, 5, 9, 13
    //blue: 3, 7, 11, 15

    if (task_i == 16)
    {
        return true;
    }
    
    servo5target = servo5Pos[task_i];
    servo12target = servo12Pos[task_i];
    bluemotortarget = bluePos[task_i];

    servoPin5.setTargetPos(servo5target);
    servoPin12.setTargetPos(servo12target);

    
    
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
        if(digitalRead(14) == HIGH){
        robotState = ROBOT_TASK;
        timerTask.start(2000);
        }
        
        // We do FK regardless of state
        UpdatePose(velocity);
         
        if(robotState == ROBOT_TASK && timerTask.CheckExpired())
        {
            if(checkReached()){
                if(doNextTask()){   //to do out auton, set the target positions in the array in robot.h for the coresponding actuators
                    // EnterIdleState();
                    SetDestination(dests_pose[dests_i]);
                    robotState == ROBOT_DRIVE_TO_POINT;
                }
            }
            timerTask.start(2000);

        } else if(robotState == ROBOT_DRIVE_TO_POINT) {
            DriveToPoint();
            if(CheckReachedDestination()){ HandleDestination();};
        }

        //blue motor p control loop here
        double kp = 2;  //1.5
        double ki = 0.01; //%5-%10 of Kp
        
        int base = 0; //175

        currentPos = blueMotor.getPosition();
        int error = (bluemotortarget-currentPos);
        sumError += error; //Current error + previous error
        blueMotor.setEffort(((error*kp) + (sumError*ki)) + base);

        servoPin5.update();
        servoPin12.update();

        Serial.print("      target: ");
        Serial.print(bluemotortarget);
        Serial.print("      task: ");
        Serial.print(task_i); //ln
        Serial.print("      ticks: ");
        Serial.print(blueMotor.getPosition());
        Serial.print("      effort: ");
        Serial.println((error*kp) + (sumError*ki));
        Serial.print("      state: ");
        Serial.print(robotState);
    }
}