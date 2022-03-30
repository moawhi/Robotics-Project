// File: z5184027_MTRN4110_PhaseB.cpp
// Date: 07/07/2021
// Description:
// Author: Minh Duong Nguyen
// Modifications:

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <queue>




#define MAP_FILE_NAME "../../Map.txt"
#define OUTPUT_FILE_NAME "../../Output.txt"
#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"

using namespace std;

bool Hwall[6][9] = {0};
bool Vwall[5][10] = {0};
int CellValue[5][9] = {0};
int robot_initial_row = -1;
int robot_initial_col = -1;
int target_cell_row = -1;
int target_cell_col = -1;
char robot_initial_heading = ' ';


class Pose {
  public:
  int row;
  int col;
  char heading;
  Pose() {};
  Pose(int _row, int _col, char _heading) : row(_row), col(_col), heading(_heading) {};
};
class Path {
  public:
  vector<Pose> Step;
  string PathPlan;
  Pose position;
  Path(Pose _position) {
    position=_position;
  };
  Path(Pose _position, Path& PrePath, const char * step_member, char heading) {
    position = _position;
    Step = PrePath.Step;
    PathPlan = PrePath.PathPlan;
    PathPlan.append(step_member);
    position.heading = heading;  
  }
};

// Reference : https://stackoverflow.com/questions/2202731/is-there-support-in-c-stl-for-sorting-objects-by-attribute
bool compare ( Path p1, Path p2 ) {
  return p1.PathPlan.length() < p2.PathPlan.length();  
}

Pose NextPose(Pose position, char heading) {
  int brake = 0;
  if (heading == 'N' && brake == 0) {
    return Pose(position.row-1, position.col, position.heading);
  } else if (heading == 'E') {
    return Pose(position.row, position.col+1, position.heading);
  } else if (heading == 'S') {
    return Pose(position.row+1, position.col, position.heading);
  } else if (heading == 'W') {
    return Pose(position.row, position.col-1, position.heading);
  } else {
    return Pose(0,0, position.heading);
    brake = 1;
  }
}



int Map_Scan();
int FloodFill();
int Display_Map(int Cell[5][9]);
void Display_Path(Path path); 
bool CanGo (int row, int col, char heading);
Pose NextPose(Pose position, char heading);
void Find_Path(int cellvalue[5][9], Path path_member);
char headingUpdate(char heading, char step);
void OutputCompletePath();
void ShortestPath();
//const int sort( PathList.begin(),PathList.end(), PathLengthShortest());

vector <Path> PathList;


int main(int argc, char **argv) {
  //freopen(stdout, "w", stdout);
  // ofstream outputfile;
  // outputfile.open(OUTPUT_FILE_NAME);
  int i = 0;
  while ( i < 2) {
    if (i == 1) {
      freopen(OUTPUT_FILE_NAME, "w", stdout);
      Map_Scan();
      OutputCompletePath();
      ShortestPath();
    } else {
      // Task 3.1: Read in a map from a text file and display it in the console
      Map_Scan();

      // Task 3.2: Find all the shortest paths for the robot from its initial location 
      // to the target position
      FloodFill();
      Find_Path(CellValue, Path(Pose(robot_initial_row, robot_initial_col, robot_initial_heading)));
      OutputCompletePath();

      // Task 3.3 + 3.4: Find the shortest path with the least turns and generate a motion sequence
      // Write the found path to a text file
      ShortestPath();
    }
    i++;
  }  
  return 0;
}

int Map_Scan() {
  // Read in Map.txt
  printf("[z5184027_MTRN4110_PhaseB] Reading in map from ../../Map.txt...\n");
  
  ifstream Map;
  Map.open(MAP_FILE_NAME);
  
  char MapLine[38] = {}; 
 
  int row = 0;
  int Wall_Row = 0;
  
  
  while ( Wall_Row < 11) {
    // Print out the Map
    
    Map.getline(MapLine, 38);  
    printf("[z5184027_MTRN4110_PhaseB] %s\n", MapLine); 
    
    int Wall_Col = 0;
    // Find Horizontal Wall
    if (MapLine[0] == ' ') {
      Wall_Col = 2;
      while (Wall_Col < 38) {
        if (MapLine[Wall_Col] == '-') {
          Hwall[row/2][(Wall_Col - 2) / 4] = 1;
        }
        Wall_Col += 4; 
      }
    } else {
      Wall_Col = 0;
      // Find Vertical Wall      
      while (Wall_Col < 38) {
        if (MapLine[Wall_Col] == '|') {
          Vwall[row/2][Wall_Col/4] = 1;
        }
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
        } else if (MapLine[Wall_Col] == 'x') {
          target_cell_row = row/2;
          target_cell_col = (Wall_Col -1)/4;
        }   
        Wall_Col += 4; 
      }
    }  
    Wall_Row++;
    row++;
  }
  printf("[z5184027_MTRN4110_PhaseB] Map read in!\n");
  Map.close();
  return 0;
}
// Reference: Tutorial Notes Week 4 Pseudo Code
int FloodFill() {
  // Initialize Cell Matrix
  int i = 0;
  while (i < 5) {
    int j = 0;
    while (j < 9) {
      CellValue[i][j] = 45;
      j++;
    }    
    i++;
  }
  // Target Cell
  CellValue[target_cell_row][target_cell_col] = 0;
  
  // FloodFill algorithm
  int CurrentExploredValue = 0;
  int MazeValueChanged = 1;
  while (MazeValueChanged != 0) {
    MazeValueChanged = 0;
    //std::cout << CellValue[row][col] << endl;
    for (int row = 0; row < 5; row++) {
      for (int col = 0; col < 9; col++) {
        if (CellValue[row][col] == CurrentExploredValue) {
          //cout << CellValue[row][col] << endl;
          // North
          if(Hwall[row][col] == 0) {
            if(row != 0 && CellValue[row-1][col] == 45) {
              CellValue[row-1][col] = CellValue[row][col] + 1;
              MazeValueChanged = 1;   
            }    
          }
          // South
          if (Hwall[row+1][col] == 0) {
            if(CellValue[row+1][col] == 45) {
              CellValue[row+1][col] = CellValue[row][col] + 1;
              MazeValueChanged = 1;   
            } 
          }
          // West
          if (Vwall[row][col] == 0) {
            if(col != 0 && CellValue[row][col-1] == 45) {
              CellValue[row][col-1] = CellValue[row][col] + 1;
              MazeValueChanged = 1;   
            } 
          }
          // East
          if (Vwall[row][col+1] == 0) {
            if(CellValue[row][col+1] == 45) {
              CellValue[row][col+1] = CellValue[row][col] + 1;
              MazeValueChanged = 1;   
            } 
          }    
        }
      }
    }
    CurrentExploredValue++;
  }
  return 0;  
}

int Display_Map(int Cell[5][9]) {
  
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
          printf("|");
        } else {
          printf(" ");
        }
        if (VWallCol < 9) {
          // Print Robot Location OR Cell Value
          if (robot_initial_row == WallRow && robot_initial_col == VWallCol) {
            if (robot_initial_heading == 'N') {
              printf(" ^ ");
            } else if (robot_initial_heading == 'E') {
              printf(" > ");
            } else if (robot_initial_heading == 'S') {
              printf(" v ");
            } else if (robot_initial_heading == 'W'){
              printf(" < ");
            } 
          } else if (target_cell_row == WallRow && target_cell_col == VWallCol) {
              printf(" 0 ");   
          } else {
              if (Cell[WallRow][VWallCol] == 0) {
              printf("   ");
            } else if (Cell[WallRow][VWallCol] < 10 && Cell[WallRow][VWallCol] > 0) {
              printf(" %d ",Cell[WallRow][VWallCol]);
            } else if (Cell[WallRow][VWallCol] < 100) {
              printf(" %d", Cell[WallRow][VWallCol]);
            }     
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

void Display_Path(Path path){
  int cell[5][9] = {0};
  int i = 0;
  while (i < 5) {
    int j = 0;
    while (j < 9) {
      cell[i][j] == 1000;
      j++;
    }
    i++;
  }
  int path_number = path.Step.size();
  for (Pose pos : path.Step) {
    cell[pos.row][pos.col] = --path_number;
    //printf(" %d ", cell[pos.row][pos.col]);
  }
  Display_Map(cell);
}

void Find_Path( int cellvalue[5][9], Path path_member) {
  path_member.Step.push_back(path_member.position);
  int row = path_member.position.row;
  int col = path_member.position.col;
  int cell_value = cellvalue[row][col]; 
  // cout<< path_member.position.heading << endl;
  //printf("%d", cell_value);
  if (cell_value == 0) {
    PathList.push_back(path_member);
    return;
  }
  // cout<< cell_value << endl;
  // Go Forward
  if (CanGo(path_member.position.row, path_member.position.col, path_member.position.heading)){
    
    Pose next_member= NextPose(path_member.position, path_member.position.heading);
    // Find the cell with 1 unit cell value less than the current cell
    if (cellvalue[next_member.row][next_member.col] == cell_value -1) {
      Find_Path(cellvalue, Path(next_member, path_member, "F", path_member.position.heading));
    }  
  }
  // Turn Left and Go Forward
  char turn_left = headingUpdate(path_member.position.heading, 'L');
  
  if (CanGo(path_member.position.row, path_member.position.col, turn_left)){    
    
    Pose next_member = NextPose(path_member.position, turn_left);
    // Find the cell with 1 unit cell value less than the current cell
    if (cellvalue[next_member.row][next_member.col] == cell_value -1) {
      Find_Path(cellvalue,Path(next_member, path_member, "LF", turn_left));
    }
  }
  // Turn Right and Go Forward
  char turn_right = headingUpdate(path_member.position.heading, 'R');
  
  if (CanGo(path_member.position.row, path_member.position.col, turn_right)) {
    
    Pose next_member = NextPose(path_member.position, turn_right);
    // Find the cell with 1 unit cell value less than the current cell
    if (cellvalue[next_member.row][next_member.col] == cell_value -1) {
      Find_Path(cellvalue,Path(next_member, path_member, "RF", turn_right));
    }
  }
}

bool CanGo (int row, int col, char heading) {
  if (Hwall[row][col] == 0 && heading == 'N') {
    // North 
    return 1;
  } else if (Vwall[row][col+1] == 0 && heading == 'E') {
    // East
    return 1;
  } else if (Hwall[row+1][col] == 0 && heading == 'S') {
    // South
    return 1;
  } else if (Vwall[row][col] == 0 && heading == 'W') {
    // West
    return 1;
  } else {
    return 0;
  }
}

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

void OutputCompletePath() {
  printf("[z5184027_MTRN4110_PhaseB] Finding shortest paths...\n");
  int path_number = 0;
  for (Path path : PathList) {
    path_number++;
    printf("[z5184027_MTRN4110_PhaseB] Path - %d: \n",path_number);
    Display_Path(path);
  }
  printf("[z5184027_MTRN4110_PhaseB] %d shortest paths found!\n", path_number);
}
void ShortestPath() {
  printf("[z5184027_MTRN4110_PhaseB] Finding shortest path with least turns...\n");
  sort(PathList.begin(), PathList.end(), compare);
  Path ShortestPaths = PathList.front();
  Display_Path(ShortestPaths);
  printf("[z5184027_MTRN4110_PhaseB] Shortest path with least turns found!\n");
  printf("[z5184027_MTRN4110_PhaseB] Path Plan (%d steps): ", (int) ShortestPaths.PathPlan.length());
  printf("%d%d%c%s\n", robot_initial_row, robot_initial_col, robot_initial_heading, ShortestPaths.PathPlan.c_str());
  printf("[z5184027_MTRN4110_PhaseB] Writing path plan to ../../PathPlan.txt...\n");
  ofstream pathPlantxt;
  pathPlantxt.open(PATH_PLAN_FILE_NAME);
  pathPlantxt << robot_initial_row;
  pathPlantxt << robot_initial_col;
  pathPlantxt << robot_initial_heading;
  pathPlantxt << ShortestPaths.PathPlan.c_str();
  pathPlantxt.close();
  printf("[z5184027_MTRN4110_PhaseB] Path plan written tp ../../PathPlan.txt...");
}