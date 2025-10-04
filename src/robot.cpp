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
    if(abs(bluemotortarget - blueMotor.getPosition()) < 30){
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
        chassis.SetMotorEfforts(100, -100);
        delay(850);
        chassis.SetMotorEfforts(-130, -80);
        delay(4600);
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
    if(digitalRead(14) == HIGH){
        robotState = ROBOT_TASK;
        timerTask.start(800);
    }

    if(robotState == ROBOT_TASK)
    {
        if(timerTask.CheckExpired()){
            if(checkReached()){
                if(doNextTask()){   //to do out auton, set the target positions in the array in robot.h for the coresponding actuators
                    EnterIdleState();
                    //HandleDestination();
                    // robotState = ROBOT_DRIVE_TO_POINT;
                    exit(0);
                }
            }
            timerTask.start(800);
        }
        //blue motor p control loop here
        double kp = 1.0;  //1.5
        int base = 170;
        
        currentPos = blueMotor.getPosition();
        int error = (bluemotortarget-currentPos);
        if(abs(error) < 200) sumError += error; //only do i term if if within 200 ticks of target
        else sumError = 0; //otherwise reset
        int effort = (error*kp) + (sumError*ki);
        if(effort > 0) effort += base;
        blueMotor.setEffort(effort);

        servoPin5.update();
        servoPin12.update();

        Serial.print("      target: ");
        Serial.print(bluemotortarget);
        Serial.print("      task: ");
        Serial.print(task_i);
        Serial.print("      ticks: ");
        Serial.print(blueMotor.getPosition());
        Serial.print("      effort: ");
        Serial.print((error*kp) + (sumError*ki));
        Serial.print("      state: ");
        Serial.println(robotState);

    } else if(robotState == ROBOT_DRIVE_TO_POINT) {
        if(chassis.ChassisLoop(velocity)){
            //We do fk regardless of state
            UpdatePose(velocity);

            DriveToPoint();
            if(CheckReachedDestination()){ HandleDestination();};
        }
    }
}