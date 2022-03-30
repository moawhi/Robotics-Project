// File:          z5184027_MTRN4110_PhaseA.cpp
// Date: 10/06/2021
// Description:
// Author: Minh Duong Nguyen
// Modifications:

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>

// All the webots classes are defined in the "webots" namespace
using namespace webots;


#define TIME_STEP 64
#define PI 3.14

#define MAX_SPEED 6.28*0.4
#define STEP_FORWARD 165/20  // in radian, linear distant/wheel radius
#define STEP_ROTATION PI/2*(56.6/2)/20  // in radian, angle of Rotation*axle radius/wheel radius
#define MOTION_PLAN_FILE_NAME "../../MotionPlan.txt"
#define MOTION_EXECUTION_FILE_NAME "../../MotionExecution.csv"

char WallDetect(DistanceSensor*left, DistanceSensor*right, DistanceSensor*front);
char headingUpdate(char heading, char step);
int* PositionUpdate(int* coordinate, char heading, char step);

double left_position = 0;
double right_position = 0;

int main(int argc, char **argv) {
  // Read in motion plan from "MotionPlan.txt"
  std::cout << "[z5184027_MTRN4110_PhaseA] Reading in motion plan from ../../MotionPlan.txt..." << std::endl;
  std::cout << "[z5184027_MTRN4110_PhaseA] Motion Plan: ";
  // Place content of text file in an array of charater
  int i = 0;
  char step[100];
  std::ifstream StepFile(MOTION_PLAN_FILE_NAME);
  while (!StepFile.eof()) {
      StepFile >> step[i];
      i = i + 1;
  }
  StepFile.close();
  // Print out motion plan
  int j = 0;
  while(j < i) {
       std::cout << step[j];
       j = j + 1;
  }
  int StepLength = j;
  
  // Print out initial position
  std::cout<< "\n" <<std::endl;
  std::cout << "[z5184027_MTRN4110_PhaseA] Motion Plan read in! " << std::endl;
  std::cout << "[z5184027_MTRN4110_PhaseA] Executing motion plan... " << std::endl;
  
  std::ofstream MyCSVFile;
  MyCSVFile.open(MOTION_EXECUTION_FILE_NAME);
  MyCSVFile << "Step" << "," << "Row" << "," << "Column" 
  << "," << "Heading" << "," << "Left Wall" << "," <<"Front Wall"
  << "," <<"Right Wall" << std::endl;
  
  // create the Robot instance.
  Robot *robot = new Robot();
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  // Initialize Motors
  Motor* leftMotor = robot->getMotor("left wheel motor");
  Motor* rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setVelocity(MAX_SPEED);
  rightMotor->setVelocity(MAX_SPEED);
  leftMotor->setPosition(0);
  rightMotor->setPosition(0);
  //  Initialize distance sensors
  DistanceSensor *ds[3];
  char dsNames[3][10] = {
    "dsl","dsr","dsf"
  };
  for (int i = 0; i < 3; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(timeStep);
  }
   
  int StepNo = 3;
  j = 0;
  double pastTime = robot->getTime();
  double waitTime = 0;
  
  int init_row = step[0]-48;
  int init_col = step[1]-48;
  char heading = step[2];
  int pos[2] = {init_row,init_col};
  int* position = pos;
  int end_flag = 0;
  char right_wall, left_wall, front_wall;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1 && end_flag == 0) {
    double currentTime = robot->getTime();
    if (currentTime - pastTime >= waitTime) {
      // Print out Pos Infor to console
      std::cout << "[z5184027_MTRN4110_PhaseA] Step: " ;
      if (j <10) {
        printf("00%d",j);  
      }else if (j >= 10 && j <100) {
        printf("0%d",j);
      } else {
        printf("%d",j);
      }
      std::cout << " row: " << *(pos) << ","
      << " column: " << *(pos+1) << ","
      << " heading: " << heading << ",";
      // Print out Pos Infor to .csv file
      MyCSVFile << j << "," << *(pos) << "," <<  *(pos+1)
      << "," << heading << "," ;
 
      // Dectect Wall
      if (ds[1]->getValue() < 1000) {
        right_wall = 'Y';
      } else {
        right_wall = 'N';  
      }
      if (ds[0]->getValue() < 1000) {
        left_wall = 'Y';
      } else {
        left_wall = 'N';  
      }
      if (ds[2]->getValue() < 1000) {
        front_wall = 'Y';
      } else {
        front_wall = 'N';  
      }    
      std::cout << " left wall: " << left_wall << ","
          << " front wall: " << front_wall << ","
          << " right wall: "  << right_wall
          << std::endl;

      MyCSVFile << left_wall << "," <<  front_wall << "," << right_wall << std::endl;
      
      // Motor Control
      if (StepNo < StepLength && end_flag == 0) {
        if (step[StepNo] =='F') {
            left_position += STEP_FORWARD; 
            right_position += STEP_FORWARD; 
            rightMotor->setPosition(right_position);
            leftMotor->setPosition(left_position);
          
            waitTime = 4;        
        } else if (step[StepNo] =='L'){
            left_position -= (STEP_ROTATION); 
            right_position += (STEP_ROTATION); 
            rightMotor->setPosition(right_position); 
            leftMotor->setPosition(left_position);
             
            waitTime = 4;
        } else if (step[StepNo] == 'R') {
            left_position += (STEP_ROTATION); 
            right_position -= (STEP_ROTATION); 
            rightMotor->setPosition(right_position);
            leftMotor->setPosition(left_position);
          
            waitTime = 4;        
        } else {
            end_flag = 1;
            std::cout<<"[z5184027_MTRN4010_PhaseA] Motion plan executed!" << std::endl;        
        }
      }
      heading = headingUpdate(heading, step[StepNo]);
      position = PositionUpdate(position, heading, step[StepNo]);
      
      pastTime = robot->getTime();
      StepNo++;
      j++;
    }
  }   
  // Enter here exit cleanup code.
  printf("[z5184027_MTRN4010_PhaseA] Motion plan executed!");
  delete robot;
  return 0;
}

// Update heading of the robot base on the step_rotation
char headingUpdate(char heading, char step) {
  if (heading == 'N' && step == 'L') {
    heading = 'W';
  } else if (heading == 'N' && step == 'R') {
    heading = 'E';
  } else if (heading == 'E' && step == 'L') {
    heading = 'N';
  } else if (heading == 'E' && step == 'R') {
    heading = 'S';
  } else if (heading == 'S' && step == 'L') {
    heading = 'E';
  } else if (heading == 'S' && step == 'R') {
    heading = 'W';
  } else if (heading == 'W' && step == 'L') {
    heading = 'S';
  } else if (heading == 'W' && step == 'R') {
    heading = 'N';
  }
  return heading;  
}
// Update Coordinate base on the step_forward
int* PositionUpdate(int* coordinate, char heading, char step) {
  if (heading == 'N' && step == 'F'){
    *coordinate -= 1;
  } else if (heading == 'E' && step == 'F'){
    *(coordinate+1) += 1;
  } else if (heading == 'S' && step == 'F'){
    *coordinate += 1;
  } else if (heading == 'W' && step == 'F'){
    *(coordinate+1) -= 1;
  }
  return coordinate;
}