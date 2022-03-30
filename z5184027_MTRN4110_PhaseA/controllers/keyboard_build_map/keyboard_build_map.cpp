// File:          z5184027_MTRN4110_PhaseA.cpp
// Date: 10/06/2021
// Description:
// Author: Minh Duong Nguyen
// Modifications:

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
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

int MapUpdate(int* coordinate, char heading, char left_wall, char right_wall, char front_wall, char back_wall);
char headingUpdate(char heading, char step);
int* PositionUpdate(int* coordinate, char heading, char step);
char KeyConvert(int key);
int Display_Map(int* coordinate, char heading);

bool Hwall[6][9] = {0};
bool Vwall[5][10] = {0};
double left_position = 0;
double right_position = 0;

int main(int argc, char **argv) {
  
  
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
  DistanceSensor *ds[4];
  char dsNames[4][10] = {
    "dsl","dsr","dsf","dsb"
  };
  for (int i = 0; i < 4; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(timeStep);
  }
  // Keyboard
  Keyboard keyboard;
  keyboard.enable(TIME_STEP);
   
  int StepNo = 0;
  char step[1000];
  int j = 0;
  double pastTime = robot->getTime();
  double waitTime = 0;
  
  char heading = 'S';
  int pos[2] = {0,0};
  int* position = pos;
  int end_flag = 0;
  char right_wall, left_wall, front_wall, back_wall;
  
  // for (int i = 0; i < 5; i++) {
     // int j = 0;
     // std::cout << Hwall[i][j];      
     // for ( j = 0; j < 9; j++) {
        // std::cout << Vwall[i][j];
     // }
     // printf("\n");
  // }
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1 ) {
    double currentTime = robot->getTime();
    int key = keyboard.getKey();
    if (currentTime - pastTime >= waitTime && key != -1 && end_flag == 0) {
        
      //std::cout << key << std::endl;
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
      if (ds[3]->getValue() < 1000) {
        back_wall = 'Y';
      } else {
        back_wall = 'N';
      }
      // for (int i = 0; i< 4; i++) {
        // std::cout << ds[i] << std::endl;
      // }
      
      std::cout << " left wall: " << left_wall << ","
          << " front wall: " << front_wall << ","
          << " right wall: "  << right_wall << " back wall: " << back_wall
          << std::endl;
      
      Display_Map(position, heading);
      MapUpdate(position, heading, left_wall, right_wall, front_wall, back_wall);
      // Dectect Wall
      step[StepNo] = KeyConvert(key);
       
      std::cout << step[StepNo] << std::endl;
      std::cout << pos[0] << std::endl;
      std::cout << pos[1] << std::endl;
      
      //std::cout << Hwall[0][1] << std::endl;
      // Motor Control
      if (end_flag == 0) {
        if (step[StepNo] =='F') {
            left_position += STEP_FORWARD; 
            right_position += STEP_FORWARD;
            rightMotor->setPosition(right_position);
            leftMotor->setPosition(left_position);
            waitTime = 3.5;                
        } else if (step[StepNo] =='L'){
            left_position -= (STEP_ROTATION); 
            right_position += (STEP_ROTATION);
            rightMotor->setPosition(right_position);
            leftMotor->setPosition(left_position);
            waitTime = 1.5; 
        } else if (step[StepNo] == 'R') {
            left_position += (STEP_ROTATION); 
            right_position -= (STEP_ROTATION);
            rightMotor->setPosition(right_position);
            leftMotor->setPosition(left_position);
            waitTime = 1.5;        
        } else if (step[StepNo] == 'B') {
            left_position -= STEP_FORWARD; 
            right_position -= STEP_FORWARD;
            rightMotor->setPosition(right_position);
            leftMotor->setPosition(left_position);
            waitTime = 3.7; 
        } else if (step[StepNo] == 'C') {
            end_flag = 1;
        
            std::cout<<"End Game!" << std::endl;        
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
// Key convert to Step
char KeyConvert(int key) {
  char step;
  if (key == 315) {
    step = 'F'; 
  } else if (key == 314) {
    step = 'L';
  } else if (key == 316) {
    step = 'R';
  } else if (key == 317) {
    step = 'B';
  } else if (key == 67) {
    step = 'C';
  }
  return step;
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
  } else if (heading == 'E' && step == 'B'){
    *(coordinate+1) -= 1;
  } else if (heading == 'S' && step == 'B'){
    *coordinate -= 1;
  } else if (heading == 'W' && step == 'B'){
    *(coordinate+1) += 1;
  } else if (heading == 'N' && step == 'B'){
    *coordinate += 1;
  }
  return coordinate;
}
// Map Update
int MapUpdate(int* coordinate, char heading, char left_wall, char right_wall, char front_wall, char back_wall) {
  if (heading == 'N' && left_wall == 'Y') {
    Vwall[*coordinate][*(coordinate+1)] = 1;
  }  
  if (heading == 'N' && right_wall == 'Y') {
    Vwall[*coordinate][*(coordinate+1)+1] = 1;
  }  
  if (heading == 'N' && front_wall == 'Y') {
    Hwall[*coordinate][*(coordinate+1)-1] = 1;
  } 
  if (heading == 'N' && back_wall == 'Y') {
    Hwall[*coordinate+1][*(coordinate+1)] = 1;
  }
  if (heading == 'E' && left_wall == 'Y') {
    Hwall[*coordinate][*(coordinate+1)] = 1;
  } 
  if (heading == 'E' && right_wall == 'Y') {
    Hwall[*coordinate+1][*(coordinate+1)] = 1;
  } 
  if (heading == 'E' && front_wall == 'Y') {
    Vwall[*coordinate][*(coordinate+1)+1] = 1;
  }
  if (heading == 'E' && back_wall == 'Y') {
    Vwall[*coordinate][*(coordinate+1)] = 1;
  }
  if (heading == 'S' && left_wall == 'Y') {
    Vwall[*coordinate][*(coordinate+1)+1] = 1;
  } 
  if (heading == 'S' && right_wall == 'Y') {
    Vwall[*coordinate][*(coordinate+1)] = 1;
  } 
  if (heading == 'S' && front_wall == 'Y') {
    Hwall[*coordinate+1][*(coordinate+1)] = 1;
  }
  if (heading == 'S' && back_wall == 'Y') {
    Hwall[*coordinate][*(coordinate+1)] = 1;
  } 
  if (heading == 'W' && left_wall == 'Y') {
    Hwall[*coordinate+1][*(coordinate+1)] = 1;
  } 
  if (heading == 'W' && right_wall == 'Y') {
    Hwall[*coordinate][*(coordinate+1)] = 1;
  } 
  if (heading == 'W' && front_wall == 'Y') {
    Vwall[*coordinate][*(coordinate+1)] = 1;
  }
  if (heading == 'W' && back_wall == 'Y') {
    Vwall[*coordinate][*(coordinate+1)+1] = 1;
  }
  return 0;
}
int Display_Map(int* coordinate, char heading) {
  int target_cell_row = 2;
  int target_cell_col = 4;
  int WallRow = 0;
  while (WallRow < 6) {
    // Print out Horizontal Wall
    printf("[z5184027_MTRN4110_PhaseB]  ");
    int HWallCol = 0;
    while (HWallCol < 9) {
      if (Hwall[WallRow][HWallCol] == 1) {
        printf("--- ");
      } else {
        printf("    ");
      }
      HWallCol++;
    }
    printf("\n"); 
    // Print out Vertical Wall
    if (WallRow < 5) {
      printf("[z5184027_MTRN4110_PhaseB] ");
      int VWallCol = 0;
      while (VWallCol < 10) {
        if (Vwall[WallRow][VWallCol] == 1) {
          printf("|   ");
        } else {
          printf("    ");
        }
        if (VWallCol < 9) {
          // //Print Robot Location OR Cell Value
          if ( *coordinate == WallRow && VWallCol == *(coordinate+1) -1 ) {
            if (heading == 'N') {
              printf("  ^ ");
            } else if (heading == 'E') {
              printf("  > ");
            } else if (heading == 'S') {
              printf("  v ");
            } else if (heading == 'W'){
              printf("  < ");
            } 
          } else if (target_cell_row == WallRow && target_cell_col - 1 == VWallCol) {
              printf("  x ");   
          }     
        }
        VWallCol++;  
      }
      printf("\n"); 
    }
    WallRow++;
  } 
  return 0;
} 