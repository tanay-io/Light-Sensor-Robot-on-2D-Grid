#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>


const int GRID_SIZE = 10;
const int MAX_MOVEMENTS = 60;  
const int NUM_OBSTACLES = 10;
const int MAX_PATH_LENGTH = 100;
const int MAX_VISITED_LENGTH = 200;

class Grid;
class Light;
class Obstacle;

struct Position {
    int x;
    int y;
    
    Position(int x_coord = 0, int y_coord = 0) : x(x_coord), y(y_coord) {}
    
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }

    // Manhattan distance calculation
    int manhattanDistanceTo(const Position& other) const {
        return std::abs(x - other.x) + std::abs(y - other.y);
    }
};

class Obstacle {
private:
    int x;
    int y;
    
public:
    Obstacle() : x(0), y(0) {}
    
    Obstacle(int x_coord, int y_coord) : x(x_coord), y(y_coord) {}
    
    int getX() const { return x; }
    
    int getY() const { return y; }
    
    void setPosition(int x_coord, int y_coord) {
        x = x_coord;
        y = y_coord;
    }
    
    void printPosition() const {
        std::cout << "Obstacle Position: (" << x << ", " << y << ")" << std::endl;
    }
};

class Robot;

class Light {
private:
    int x;
    int y;
    bool hasBeenCapturedOnce;
    
public:
    Light() : hasBeenCapturedOnce(false) {
        x = rand() % GRID_SIZE;
        y = rand() % GRID_SIZE;
    }
    
    int getX() const { return x; }
    
    int getY() const { return y; }
    
    void moveRandomly() {
        x = rand() % GRID_SIZE;
        y = rand() % GRID_SIZE;
        hasBeenCapturedOnce = true;
    }
    
    bool wasCapturePossible() const {
        return hasBeenCapturedOnce;
    }
    
    void printPosition() const {
        std::cout << "Light Position: (" << x << ", " << y << ")" << std::endl;
    }
};

class Robot {
private:
    int x;
    int y;
    Position path[MAX_PATH_LENGTH];
    Position visited[MAX_VISITED_LENGTH];
    int pathLength;
    int visitedLength;
    bool firstCaptureComplete;
    
public:
    Robot() : pathLength(1), visitedLength(1), firstCaptureComplete(false) {
        x = rand() % GRID_SIZE;
        y = rand() % GRID_SIZE;
        path[0] = Position(x, y);
        visited[0] = Position(x, y);
    }
    
    int getX() const { return x; }
    
    int getY() const { return y; }
    
    float distanceTo(const Light& light) const {
        int dx = x - light.getX();
        int dy = y - light.getY();
        return std::sqrt(dx * dx + dy * dy);
    }
    
    void moveTowardsShortestPath(const Light& light, Obstacle* obstacles) {
        Position validMoves[4];
        int validMoveCount = 0;
        int dx[] = {0, 0, -1, 1};
        int dy[] = {-1, 1, 0, 0};
        
        for (int i = 0; i < 4; ++i) {
            int newX = x + dx[i];
            int newY = y + dy[i];
            
            if (isValidMove(newX, newY, obstacles)) {
                validMoves[validMoveCount++] = Position(newX, newY);
            }
        }
        
        if (validMoveCount == 0) {
            backtrack();
            return;
        }
        
        Position lightPos(light.getX(), light.getY());
        Position bestMove = validMoves[0];
        int shortestDistance = bestMove.manhattanDistanceTo(lightPos);
        
        for (int i = 1; i < validMoveCount; ++i) {
            int currDistance = validMoves[i].manhattanDistanceTo(lightPos);
            if (currDistance < shortestDistance) {
                bestMove = validMoves[i];
                shortestDistance = currDistance;
            }
        }
        
        x = bestMove.x;
        y = bestMove.y;
        
        // Update path
        if (pathLength < MAX_PATH_LENGTH) {
            path[pathLength++] = Position(x, y);
        } else {
            // Shift array if max length reached
            for (int i = 0; i < MAX_PATH_LENGTH - 1; ++i) {
                path[i] = path[i + 1];
            }
            path[MAX_PATH_LENGTH - 1] = Position(x, y);
        }
        
        // Update visited
        if (visitedLength < MAX_VISITED_LENGTH) {
            visited[visitedLength++] = Position(x, y);
        } else {
            // Shift array if max length reached
            for (int i = 0; i < MAX_VISITED_LENGTH - 1; ++i) {
                visited[i] = visited[i + 1];
            }
            visited[MAX_VISITED_LENGTH - 1] = Position(x, y);
        }
    }
    
    bool isValidMove(int newX, int newY, Obstacle* obstacles) const {
        if (newX < 0 || newX >= GRID_SIZE || newY < 0 || newY >= GRID_SIZE) {
            return false;
        }
        
        for (int i = 0; i < NUM_OBSTACLES; ++i) {
            if (obstacles[i].getX() == newX && obstacles[i].getY() == newY) {
                return false;
            }
        }
        
        int visitCount = 0;
        for (int i = 0; i < visitedLength; ++i) {
            if (visited[i].x == newX && visited[i].y == newY) {
                visitCount++;
            }
        }
        
        return visitCount < 2;
    }
    
    void backtrack() {
        if (pathLength > 1) {
            pathLength--;
            
            Position prevPos = path[pathLength - 1];
            x = prevPos.x;
            y = prevPos.y;
        }
    }
    
    void printPosition() const {
        std::cout << "Robot Position: (" << x << ", " << y << ")" << std::endl;
    }
    
    const Position* getPath() const {
        return path;
    }
    
    int getPathLength() const {
        return pathLength;
    }
};

class Grid {
private:
    char grid[GRID_SIZE][GRID_SIZE];
    Robot robot;
    Light light;
    Obstacle obstacles[NUM_OBSTACLES];
    int movementCount;
    bool lightRepositioned;
    
public:
    Grid() : movementCount(0), lightRepositioned(false) {
        srand(time(nullptr));
        
        placeObstacles();
        
        clearGrid();
        
        repositionEntities();
        
        updateGrid();
    }
    
    void placeObstacles() {
        for (int i = 0; i < NUM_OBSTACLES; ++i) {
            int x, y;
            bool validPosition;
            do {
                x = rand() % GRID_SIZE;
                y = rand() % GRID_SIZE;
                
                validPosition = true;
                for (int j = 0; j < i; ++j) {
                    if (obstacles[j].getX() == x && obstacles[j].getY() == y) {
                        validPosition = false;
                        break;
                    }
                }
            } while (!validPosition);
            
            obstacles[i].setPosition(x, y);
        }
    }
    
    void repositionEntities() {
        bool robotOnObstacle, lightOnObstacle;
        do {
            robot = Robot();
            
            robotOnObstacle = false;
            for (int i = 0; i < NUM_OBSTACLES; ++i) {
                if (obstacles[i].getX() == robot.getX() && 
                    obstacles[i].getY() == robot.getY()) {
                    robotOnObstacle = true;
                    break;
                }
            }
        } while (robotOnObstacle);
        
        do {
            light = Light();
            
            lightOnObstacle = false;
            for (int i = 0; i < NUM_OBSTACLES; ++i) {
                if (obstacles[i].getX() == light.getX() && 
                    obstacles[i].getY() == light.getY()) {
                    lightOnObstacle = true;
                    break;
                }
            }
        } while (lightOnObstacle);
    }
    
    void clearGrid() {
        for (int i = 0; i < GRID_SIZE; ++i) {
            for (int j = 0; j < GRID_SIZE; ++j) {
                grid[i][j] = '.';
            }
        }
    }
    
    void updateGrid() {
        clearGrid();
        
        for (int i = 0; i < NUM_OBSTACLES; ++i) {
            grid[obstacles[i].getY()][obstacles[i].getX()] = '#';
        }
        
        grid[robot.getY()][robot.getX()] = 'R';
        
        grid[light.getY()][light.getX()] = 'L';
        
        const Position* path = robot.getPath();
        int pathLength = robot.getPathLength();
        for (int i = 0; i < pathLength; ++i) {
            if (grid[path[i].y][path[i].x] == '.') {
                grid[path[i].y][path[i].x] = '*';
            }
        }
    }
    
    void displayGrid() {
        std::cout << "\nGrid State:" << std::endl;
        for (int i = 0; i < GRID_SIZE; ++i) {
            for (int j = 0; j < GRID_SIZE; ++j) {
                std::cout << grid[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    
    void runSimulation() {
        std::cout << "Obstacles:" << std::endl;
        for (int i = 0; i < NUM_OBSTACLES; ++i) {
            obstacles[i].printPosition();
        }
        std::cout << std::endl;
        
        while (movementCount < MAX_MOVEMENTS) {
            std::cout << "Movement " << movementCount + 1 << ":" << std::endl;
            robot.printPosition();
            light.printPosition();
            displayGrid();
            
            if (robot.getX() == light.getX() && robot.getY() == light.getY()) {
                std::cout << "Light Found!" << std::endl;
                
                if (!lightRepositioned) {
                    light.moveRandomly();
                    lightRepositioned = true;
                    std::cout << "Light repositioned!" << std::endl;
                    continue;  
                }
                else {
                    break; 
                }
            }
            
            robot.moveTowardsShortestPath(light, obstacles);
            
            updateGrid();
            
            ++movementCount;
        }
        
        grid[light.getY()][light.getX()] = 'B';
        
        std::cout << "Final Simulation State:" << std::endl;
        displayGrid();
        
        if (robot.getX() != light.getX() || robot.getY() != light.getY()) {
            std::cout << "Robot failed to reach the light within " 
                      << MAX_MOVEMENTS << " movements." << std::endl;
        }
    }
};

int main() {
    std::cout << "Light Sensor Robot Simulation with Shortest Path and One Reposition" << std::endl;
    std::cout << "=================================================================" << std::endl;
    
    Grid simulation;
    simulation.runSimulation();
    
    return 0;
}
