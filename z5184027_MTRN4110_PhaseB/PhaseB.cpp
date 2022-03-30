// File: z5184027_MTRN4110_PhaseB.cpp
// Date: 07/07/2021
// Description:
// Author: Minh Duong Nguyen
// Modifications:


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <queue>


#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"
#define MAP_FILE_NAME "../../Map.txt"

using namespace std;

bool Hwall[6][9] = {0};
bool Vwall[5][10] = {0};
int CellValue[5][9] = {0};

// class Wall {  
//   bool Hwall[6][9] = {0};
//   bool Vwall[5][10] = {0};
// };

// class Position {
//   int row = 0;
//   int col = 0;
//   Position(){}
//   RobotPos(int _row, int _col) : row(_row), col(_col) {};
// };

// class Path {

// };

// int North(int row, int col) {
//   return !Hwall[row][col]; 
// }
// printf(North);
// int East(int row, int col) {
//   return !Hwall[row][col]; 
// }
// int South(int row, int col) {
//   return !Hwall[row][col]; 
// }
// int West(int row, int col) {
//   return !Hwall[row][col]; 
// }

// }
int Map_Scan();
// void Display_Map(int CellValue[5][9], int RobotRow, int RobotCol, int RobotHeading);

int main(int argc, char **argv) {
  // Task 3.1: Read in Map infor from Map.txt and Display Map
  printf("[z5184027_MTRN4110_PhaseB] Reading in map from ../../Map.txt...\n");
  Map_Scan();

  // Initialize Cell Matrix
  int i = 0;
  while (i < 5) {
    int j = 0;
    while (j < 9) {
      CellValue[i][j] = -1;
      j++;
    }
    i++;
  }
  i = 0;
  while (i < 5) {
    int j = 0;
    while (j < 9) {
      printf("%d", Hwall[i][j]);
      j++;
    }
    i++;
  }
  // Target cell
  CellValue[2][4] = 0;
  return 0;
}

int Map_Scan() {

  // Read in Map.txt
  ifstream Map;
  Map.open(MAP_FILE_NAME);
  
  char MapLine[38] = {}; 
 
  int row = 0;
  int col = 0;
  int Wall_Row = 0;
  int robot_initial_row = 0;
  int robot_initial_col = 0;
  char robot_initial_heading = ' ';
  
  while ( Wall_Row < 11) {
    // Print out the Map
    Map.getline(MapLine, 38);  
    printf("[z5184027_MTRN4110_PhaseB] %s\n", MapLine);
    
    int Wall_Col = 0;
    // Find Horizontal Wall
    if (MapLine[0] == ' ') {
      Wall_Col = 1;
      while (Wall_Col < 38) {
        if (MapLine[Wall_Col] == '-') {
          Hwall[row][col == (Wall_Col -1) / 4] = {1};
        }
        Wall_Col += 4; 
      }
    } else {
      // Find Vertical Wall      
      while (Wall_Col < 38) {
        Vwall[row][col = Wall_Col/4] = {1};
        Wall_Col += 4;
      }
      // Find Robot Initial location
      Wall_Col = 2;
      while ( Wall_Col < 38 ) {
        if (MapLine[Wall_Col] == '^') {
          robot_initial_row = row/2;
          robot_initial_col = (Wall_Col -1) / 4;
          robot_initial_heading = 'N';
        } else if (MapLine[Wall_Col] == '>') {
          robot_initial_row = row/2;
          robot_initial_col = (Wall_Col -1) / 4;
          robot_initial_heading = 'E';
        } else if (MapLine[Wall_Col] == 'v') {
          robot_initial_row = row/2;
          robot_initial_col = (Wall_Col -1) / 4;
          robot_initial_heading = 'S';
        } else if (MapLine[Wall_Col] == '<') {
          robot_initial_row = row/2;
          robot_initial_col = (Wall_Col -1) / 4;
          robot_initial_heading = 'W';
        }    
        Wall_Col += 4; 
      }
    }  
    Wall_Row++;
    row++;
  }
  printf("[z5184027_MTRN4110_PhaseB] Map Read in!\n");
  // cout << robot_initial_row << endl;
  // cout << robot_initial_col << endl;
  // cout << robot_initial_heading << endl;
  Map.close();
  return 0;
} 