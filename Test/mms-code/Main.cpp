#include <iostream>
#include <string>
#include <queue>
#include <utility> 
#include <vector>
#include <cmath>
#include <algorithm>
#include "API.h"

const int MAZE_SIZE = 16;

// Bitmasks for absolute directions
const int WALL_N = 1; // 0001
const int WALL_E = 2; // 0010
const int WALL_S = 4; // 0100
const int WALL_W = 8; // 1000

// Fixed: Enums are sequentially organized clockwise for circular angular math
enum Heading { 
    NORTH = 0, 
    NORTHEAST = 1, 
    EAST = 2, 
    SOUTHEAST = 3, 
    SOUTH = 4, 
    SOUTHWEST = 5, 
    WEST = 6, 
    NORTHWEST = 7 
};

enum MouseState { EXPLORING_TO_CENTER, RETURNING_TO_START, FAST_RUN };

struct Mouse {
    int x = 0;
    int y = 0;
    Heading heading = NORTH;
};

// A* Node structure optimized for 8-connected grid navigation
struct AStarNode {
    int x, y;
    int g; // Cumulative step + circular angle turn penalties
    int h; // Manhattan distance heuristic
    Heading heading;

    bool operator>(const AStarNode& other) const {
        return (g + h) > (other.g + other.h);
    }
};

int wallMap[MAZE_SIZE][MAZE_SIZE];  
int distMap[MAZE_SIZE][MAZE_SIZE];  
bool visitedMap[MAZE_SIZE][MAZE_SIZE]; 

std::vector<std::pair<int, int> > fastRunPath;
size_t pathIndex = 0;

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

// --- Orthogonal Movement Drivers ---
void moveMouseForward(Mouse& mouse) {
    API::moveForward(); 
    if (mouse.heading == NORTH)      mouse.y++;
    else if (mouse.heading == EAST)  mouse.x++;
    else if (mouse.heading == SOUTH) mouse.y--;
    else if (mouse.heading == WEST)  mouse.x--;
}

void turnMouseLeft(Mouse& mouse) {
    API::turnLeft();
    // Move 2 steps backward counter-clockwise on our 8-heading wheel (90 degrees)
    mouse.heading = static_cast<Heading>((mouse.heading + 6) % 8);
}

void turnMouseRight(Mouse& mouse) {
    API::turnRight();
    // Move 2 steps forward clockwise on our 8-heading wheel (90 degrees)
    mouse.heading = static_cast<Heading>((mouse.heading + 2) % 8);
}

// --- Secure Wall Mapping Hooks ---
void setWallAbsolute(int x, int y, Heading dir) {
    if (dir == NORTH) {
        wallMap[x][y] |= WALL_N;
        if (y + 1 < MAZE_SIZE) wallMap[x][y+1] |= WALL_S;
    }
    else if (dir == EAST) {
        wallMap[x][y] |= WALL_E;
        if (x + 1 < MAZE_SIZE) wallMap[x+1][y] |= WALL_W;
    }
    else if (dir == SOUTH) {
        wallMap[x][y] |= WALL_S;
        if (y - 1 >= 0) wallMap[x][y-1] |= WALL_N;
    }
    else if (dir == WEST) {
        wallMap[x][y] |= WALL_W;
        if (x - 1 >= 0) wallMap[x-1][y] |= WALL_E;
    }
}

void updateWalls(Mouse& mouse) {
    visitedMap[mouse.x][mouse.y] = true;

    Heading frontDir = mouse.heading;
    Heading leftDir  = static_cast<Heading>((mouse.heading + 6) % 8);
    Heading rightDir = static_cast<Heading>((mouse.heading + 2) % 8);

    if (API::wallFront()) setWallAbsolute(mouse.x, mouse.y, frontDir);
    if (API::wallLeft())  setWallAbsolute(mouse.x, mouse.y, leftDir);
    if (API::wallRight()) setWallAbsolute(mouse.x, mouse.y, rightDir);

    if (wallMap[mouse.x][mouse.y] & WALL_N) API::setWall(mouse.x, mouse.y, 'n');
    if (wallMap[mouse.x][mouse.y] & WALL_E) API::setWall(mouse.x, mouse.y, 'e');
    if (wallMap[mouse.x][mouse.y] & WALL_S) API::setWall(mouse.x, mouse.y, 's');
    if (wallMap[mouse.x][mouse.y] & WALL_W) API::setWall(mouse.x, mouse.y, 'w');
}

// --- Standard Flood-Fill Exploration Engine ---
void floodFill(bool routingToCenter) {
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) distMap[i][j] = 255;
    }

    std::queue<std::pair<int, int> > q;
    
    if (routingToCenter) {
        distMap[7][7] = 0; q.push(std::make_pair(7, 7));
        distMap[7][8] = 0; q.push(std::make_pair(7, 8));
        distMap[8][7] = 0; q.push(std::make_pair(8, 7));
        distMap[8][8] = 0; q.push(std::make_pair(8, 8));
    } else {
        distMap[0][0] = 0; 
        q.push(std::make_pair(0, 0));
    }

    while (!q.empty()) {
        std::pair<int, int> current = q.front();
        q.pop();

        int cx = current.first;
        int cy = current.second;
        int currentDist = distMap[cx][cy];

        if (!(wallMap[cx][cy] & WALL_N) && cy + 1 < MAZE_SIZE && distMap[cx][cy+1] == 255) {
            distMap[cx][cy+1] = currentDist + 1;
            q.push(std::make_pair(cx, cy+1));
        }
        if (!(wallMap[cx][cy] & WALL_E) && cx + 1 < MAZE_SIZE && distMap[cx+1][cy] == 255) {
            distMap[cx+1][cy] = currentDist + 1;
            q.push(std::make_pair(cx+1, cy));
        }
        if (!(wallMap[cx][cy] & WALL_S) && cy - 1 >= 0 && distMap[cx][cy-1] == 255) {
            distMap[cx][cy-1] = currentDist + 1;
            q.push(std::make_pair(cx, cy-1));
        }
        if (!(wallMap[cx][cy] & WALL_W) && cx - 1 >= 0 && distMap[cx-1][cy] == 255) {
            distMap[cx-1][cy] = currentDist + 1;
            q.push(std::make_pair(cx-1, cy));
        }
    }
}

int getHeuristic(int x, int y) {
    int targetX = (x < 8) ? 7 : 8;
    int targetY = (y < 8) ? 7 : 8;
    return (std::abs(targetX - x) + std::abs(targetY - y)) * 10; 
}

// --- Advanced V-Groove Safety Check for Diagonal Intersection Obstacles ---
bool isPathBlocked(int cx, int cy, int dirIndex) {
    if (dirIndex == 0) return (wallMap[cx][cy] & WALL_N) != 0; // NORTH
    if (dirIndex == 2) return (wallMap[cx][cy] & WALL_E) != 0; // EAST
    if (dirIndex == 4) return (wallMap[cx][cy] & WALL_S) != 0; // SOUTH
    if (dirIndex == 6) return (wallMap[cx][cy] & WALL_W) != 0; // WEST

    // Diagonal Rule: A corner cut requires BOTH intersecting walls to be missing
    if (dirIndex == 1) { // NORTHEAST
        if ((wallMap[cx][cy] & WALL_N) || (wallMap[cx][cy] & WALL_E)) return true;
        if (cx + 1 < MAZE_SIZE && (wallMap[cx+1][cy] & WALL_N)) return true;
        if (cy + 1 < MAZE_SIZE && (wallMap[cx][cy+1] & WALL_E)) return true;
        return false;
    }
    if (dirIndex == 3) { // SOUTHEAST
        if ((wallMap[cx][cy] & WALL_S) || (wallMap[cx][cy] & WALL_E)) return true;
        if (cx + 1 < MAZE_SIZE && (wallMap[cx+1][cy] & WALL_S)) return true;
        if (cy - 1 >= 0 && (wallMap[cx][cy-1] & WALL_E)) return true;
        return false;
    }
    if (dirIndex == 5) { // SOUTHWEST
        if ((wallMap[cx][cy] & WALL_S) || (wallMap[cx][cy] & WALL_W)) return true;
        if (cx - 1 >= 0 && (wallMap[cx-1][cy] & WALL_S)) return true;
        if (cy - 1 >= 0 && (wallMap[cx][cy-1] & WALL_W)) return true;
        return false;
    }
    if (dirIndex == 7) { // NORTHWEST
        if ((wallMap[cx][cy] & WALL_N) || (wallMap[cx][cy] & WALL_W)) return true;
        if (cx - 1 >= 0 && (wallMap[cx-1][cy] & WALL_N)) return true;
        if (cy + 1 < MAZE_SIZE && (wallMap[cx][cy+1] & WALL_W)) return true;
        return false;
    }
    return true;
}

// --- Balanced 8-Direction A* Finder with Angular Costing ---
std::vector<std::pair<int, int> > calculateAStarPath() {
    int closedG[MAZE_SIZE][MAZE_SIZE];
    std::pair<int, int> parentMap[MAZE_SIZE][MAZE_SIZE];
    
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            closedG[i][j] = 999999;
            parentMap[i][j] = std::make_pair(-1, -1);
        }
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode> > pq;
    
    closedG[0][0] = 0;
    AStarNode startNode = {0, 0, 0, getHeuristic(0, 0), NORTH};
    pq.push(startNode);

    std::pair<int, int> destinationCell = std::make_pair(-1, -1);

    while (!pq.empty()) {
        AStarNode curr = pq.top();
        pq.pop();

        if (curr.g > closedG[curr.x][curr.y]) continue;

        if ((curr.x == 7 || curr.x == 8) && (curr.y == 7 || curr.y == 8)) {
            destinationCell = std::make_pair(curr.x, curr.y);
            break;
        }

        Heading headings[8] = {NORTH, NORTHEAST, EAST, SOUTHEAST, SOUTH, SOUTHWEST, WEST, NORTHWEST};
        int dx[8] = {0,  1, 1,  1, 0, -1, -1, -1};
        int dy[8] = {1,  1, 0, -1, -1, -1,  0,  1};
        int baseCosts[8] = {10, 14, 10, 14, 10, 14, 10, 14}; // 14 simulates the sqrt(2) distance scale

        for (int i = 0; i < 8; i++) {
            if (!isPathBlocked(curr.x, curr.y, i)) {
                int nx = curr.x + dx[i];
                int ny = curr.y + dy[i];

                if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
                    
                    // Safe Constraint check: Must be a cell explored during flood phase
                    if (!visitedMap[nx][ny]) continue;

                    int stepCost = baseCosts[i];
                    
                    // Circular Turn Penalty system
                    int diff = std::abs(static_cast<int>(headings[i]) - static_cast<int>(curr.heading));
                    if (diff > 4) diff = 8 - diff; 

                    if (diff == 1)      stepCost += 15;  // 45-degree corner entry cost
                    else if (diff == 2) stepCost += 75;  // Heavy 90-degree pivot penalty (forces straights)
                    else if (diff >= 3) stepCost += 150; // Severe U-turn filtering

                    int nextG = curr.g + stepCost;

                    if (nextG < closedG[nx][ny]) {
                        closedG[nx][ny] = nextG;
                        parentMap[nx][ny] = std::make_pair(curr.x, curr.y);
                        AStarNode neighbor = {nx, ny, nextG, getHeuristic(nx, ny), headings[i]};
                        pq.push(neighbor);
                    }
                }
            }
        }
    }

    std::vector<std::pair<int, int> > path;
    if (destinationCell.first == -1) return path;

    std::pair<int, int> backtrackNode = destinationCell;
    while (backtrackNode.first != 0 || backtrackNode.second != 0) {
        path.push_back(backtrackNode);
        backtrackNode = parentMap[backtrackNode.first][backtrackNode.second];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

int main(int argc, char* argv[]) {
    log("Starting Secure 8-Directional Hybrid Flight Engine...");
    
    Mouse myMouse;
    MouseState currentState = EXPLORING_TO_CENTER;

    for(int i = 0; i < MAZE_SIZE; i++) {
        for(int j = 0; j < MAZE_SIZE; j++) {
            visitedMap[i][j] = false;
        }
    }

    for(int i=0; i<MAZE_SIZE; i++) {
        wallMap[i][0] |= WALL_S;
        wallMap[i][MAZE_SIZE-1] |= WALL_N;
        wallMap[0][i] |= WALL_W;
        wallMap[MAZE_SIZE-1][i] |= WALL_E;
    }

    while (true) {
        if (currentState != FAST_RUN) {
            API::setColor(myMouse.x, myMouse.y, 'G');
            updateWalls(myMouse);
        }

        if (currentState == EXPLORING_TO_CENTER && 
           (myMouse.x == 7 || myMouse.x == 8) && (myMouse.y == 7 || myMouse.y == 8)) {
            log("Center mapped. Re-routing back to Base...");
            currentState = RETURNING_TO_START;
            API::clearAllText();
            continue; 
        }

        if (currentState == RETURNING_TO_START && myMouse.x == 0 && myMouse.y == 0) {
            log("Base reached. Initializing 8-Connected A* Flight calculations...");
            currentState = FAST_RUN;
            API::clearAllText();
            continue;
        }

        if (currentState != FAST_RUN) {
            floodFill(currentState == EXPLORING_TO_CENTER);

            for(int x = 0; x < MAZE_SIZE; x++) {
                for(int y = 0; y < MAZE_SIZE; y++) {
                    if (distMap[x][y] != 255) {
                        API::setText(x, y, std::to_string(distMap[x][y]));
                    }
                }
            }

            int minDistance = 255;
            Heading bestDirection = myMouse.heading;

            if (!(wallMap[myMouse.x][myMouse.y] & WALL_N) && myMouse.y + 1 < MAZE_SIZE && distMap[myMouse.x][myMouse.y+1] < minDistance) {
                minDistance = distMap[myMouse.x][myMouse.y+1];
                bestDirection = NORTH;
            }
            if (!(wallMap[myMouse.x][myMouse.y] & WALL_E) && myMouse.x + 1 < MAZE_SIZE && distMap[myMouse.x+1][myMouse.y] < minDistance) {
                minDistance = distMap[myMouse.x+1][myMouse.y];
                bestDirection = EAST;
            }
            if (!(wallMap[myMouse.x][myMouse.y] & WALL_S) && myMouse.y - 1 >= 0 && distMap[myMouse.x][myMouse.y-1] < minDistance) {
                minDistance = distMap[myMouse.x][myMouse.y-1];
                bestDirection = SOUTH;
            }
            if (!(wallMap[myMouse.x][myMouse.y] & WALL_W) && myMouse.x - 1 >= 0 && distMap[myMouse.x-1][myMouse.y] < minDistance) {
                minDistance = distMap[myMouse.x-1][myMouse.y];
                bestDirection = WEST;
            }

            int turnStep = (bestDirection - myMouse.heading + 8) % 8;
            if (turnStep == 2)      turnMouseRight(myMouse);
            else if (turnStep == 4) { turnMouseRight(myMouse); turnMouseRight(myMouse); }
            else if (turnStep == 6) turnMouseLeft(myMouse);

            moveMouseForward(myMouse);
        } 
        else {
            if (fastRunPath.empty()) {
                fastRunPath = calculateAStarPath();
                
                for (size_t i = 0; i < fastRunPath.size(); i++) {
                    API::setColor(fastRunPath[i].first, fastRunPath[i].second, 'B');
                }
                log("Diagonal A* Path locked. Running simulator translation adaptors!");
            }

            if (pathIndex < fastRunPath.size()) {
                std::pair<int, int> nextCell = fastRunPath[pathIndex];
                
                // MMS Orthogonal Step Decomposer: If the true target cell is diagonal, 
                // fake it in the UI by resolving one axis first.
                if (nextCell.first != myMouse.x && nextCell.second != myMouse.y) {
                    nextCell.second = myMouse.y; 
                }

                Heading targetHeading = myMouse.heading;
                if (nextCell.second > myMouse.y)      targetHeading = NORTH;
                else if (nextCell.first > myMouse.x)  targetHeading = EAST;
                else if (nextCell.second < myMouse.y) targetHeading = SOUTH;
                else if (nextCell.first < myMouse.x)  targetHeading = WEST;

                int turnStep = (targetHeading - myMouse.heading + 8) % 8;
                if (turnStep == 2)      turnMouseRight(myMouse);
                else if (turnStep == 4) { turnMouseRight(myMouse); turnMouseRight(myMouse); }
                else if (turnStep == 6) turnMouseLeft(myMouse);

                moveMouseForward(myMouse);
                
                if (myMouse.x == fastRunPath[pathIndex].first && myMouse.y == fastRunPath[pathIndex].second) {
                    pathIndex++;
                }
            } else {
                log("FAST RUN COMPLETE: Center anchored smoothly via diagonal path profiles.");
                while (true) {}
            }
        }
    }
}