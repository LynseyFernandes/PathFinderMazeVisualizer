#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAZE_SIZE 8

int goal_maze[MAZE_SIZE][MAZE_SIZE];
int wall_maze[MAZE_SIZE][MAZE_SIZE][4]; // 0:Up, 1:Left, 2:Right, 3:Down

int GOAL_ROW = 3;
int GOAL_COLUMN = 3;

void initialize_maze();
void update_maze();
void print_maze();

int main() {
    initialize_maze();

    // test wall
    wall_maze[3][3][0] = 1; 
    wall_maze[2][3][3] = 1; 

    printf("--- Initial Maze ---\n");
    print_maze();

    update_maze();

    printf("\n--- Maze After Flood Fill ---\n");
    print_maze();

    return 0;
}

void initialize_maze() {
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            goal_maze[i][j] = abs(GOAL_ROW - i) + abs(GOAL_COLUMN - j);
        }
    }
    // Initialize boundary walls
    memset(wall_maze, 0, sizeof(wall_maze));
    for (int i = 0; i < MAZE_SIZE; i++) {
        wall_maze[0][i][0] = 1;
        wall_maze[MAZE_SIZE - 1][i][3] = 1;
        wall_maze[i][0][1] = 1;
        wall_maze[i][MAZE_SIZE - 1][2] = 1;
    }
}

void update_maze() {
    int changed;
    do {
        changed = 0;
        for (int r = 0; r < MAZE_SIZE; r++) {
            for (int c = 0; c < MAZE_SIZE; c++) {
                if (r == GOAL_ROW && c == GOAL_COLUMN) continue;

                int min_neighbor = 255;
                if (wall_maze[r][c][0] == 0 && r > 0) min_neighbor = (goal_maze[r - 1][c] < min_neighbor) ? goal_maze[r - 1][c] : min_neighbor;
                if (wall_maze[r][c][1] == 0 && c > 0) min_neighbor = (goal_maze[r][c - 1] < min_neighbor) ? goal_maze[r][c - 1] : min_neighbor;
                if (wall_maze[r][c][2] == 0 && c < MAZE_SIZE - 1) min_neighbor = (goal_maze[r][c + 1] < min_neighbor) ? goal_maze[r][c + 1] : min_neighbor;
                if (wall_maze[r][c][3] == 0 && r < MAZE_SIZE - 1) min_neighbor = (goal_maze[r + 1][c] < min_neighbor) ? goal_maze[r + 1][c] : min_neighbor;

                if (min_neighbor != 255 && goal_maze[r][c] != min_neighbor + 1) {
                    goal_maze[r][c] = min_neighbor + 1;
                    changed = 1;
                }
            }
        }
    } while (changed);
}

void print_maze() {
    for (int i = 0; i < MAZE_SIZE; i++) {
     
        for (int j = 0; j < MAZE_SIZE; j++) {
            printf("+%s", wall_maze[i][j][0] ? "---" : "   ");
        }
        printf("+\n");

      
        for (int j = 0; j < MAZE_SIZE; j++) {
            printf("%s%3d", wall_maze[i][j][1] ? "|" : " ", goal_maze[i][j]);
        }
        printf("|\n");
    }
   
    for (int j = 0; j < MAZE_SIZE; j++) {
        printf("+---");
    }
    printf("+\n");
}
