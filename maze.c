//
//  main.cpp
//  Maze
//
//  Created by Mohammad Jamali on 7/14/16.
//  Copyright © 2016 Mohammad Jamali. All rights reserved.
//

#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>

#define BLOCK   'X'
#define WAY     '.'
#define GOAL    'G'
#define START   'S'
#define MOUSE   '#'
#define TUNNEL  128

#define posY(x) x.first
#define posX(x) x.second
#define top(x)  x.at(x.size() - 1)
#define inBound(x, sizeY, sizeX) posY(x) < sizeY && posX(x) < sizeX &&\
                                 posY(x) >= 0  && posX(x) >= 0

using namespace std;

typedef pair<int, int> Point;
typedef vector<vector<char>> TDMatrix;

struct Step{
    int passedGate;
    TDMatrix gameMap;
    vector<Point> mouseLocations;
};

struct GateInfo {
    char symbol;
    Point selfLocation;
    Point destinationLocation;
};

typedef vector<GateInfo> GateVector;

inline bool isGoal(const Point& myLocation, const Point& endLocation,
                   int gates, int numberOfGateMustPass){
    if (posY(myLocation) == posY(endLocation) &&
        posX(myLocation) == posX(endLocation) &&
        gates >= numberOfGateMustPass) {
        return true;
    }
    
    return false;
}

int calculatePathLen(const TDMatrix& gameMap) {
    int len = 0;
    for (int i = 0; i < gameMap.size(); ++i) {
        for (int j = 0; j < gameMap.at(0).size(); ++j) {
            if(gameMap[i][j] == MOUSE){
                len++;
            }
        }
    }
    return len;
}

Point getTranslatedPoint(char gate, const GateVector& gates )
{
    for (int i = 0; i < gates.size(); ++i) {
        if (gates.at(i).symbol == gate) {
            return gates.at(i).destinationLocation;
        }
    }
    
    return Point(0, 0);
}

// ----- Check for out of bound and Block points before call this function
Step createStep(const TDMatrix& gameMap, vector<Point> mouseLocs,
                int passedGates, const GateVector& gates)
{
    TDMatrix temp(gameMap);
    char c = temp[posY(top(mouseLocs))][posX(top(mouseLocs))];
    
    while (c != GOAL && c != START && isalpha(c))
    {
        mouseLocs.push_back(getTranslatedPoint(c , gates));
        c = temp[posY(top(mouseLocs))][posX(top(mouseLocs))];
        
        ++passedGates;
    }
    
    temp[posY(top(mouseLocs))][posX(top(mouseLocs))] = MOUSE;
    
    Step newStep;
    newStep.gameMap = temp;
    newStep.mouseLocations = mouseLocs;
    newStep.passedGate = passedGates;
    
    return newStep;
}

bool isEqualMap(const TDMatrix& first, const TDMatrix& second)
{
    for (int i = 0; i < first.size(); ++i) {
        for (int j = 0; j < first.at(0).size(); ++j) {
            if(first.at(i).at(j) != second.at(i).at(j))
                return false;
        }
    }
    
    return true;
}

bool isDuplicated(const Step& gameStep, const vector<Step>& tempMemory)
{
    int posY, posX;
    for (int i = (int)(tempMemory.size()) - 1; i >= 0; --i)
    {
        // get last position of mouse in the map
        posY = posY(top(gameStep.mouseLocations));
        posX = posX(top(gameStep.mouseLocations));
        
        if(gameStep.gameMap[posY][posX] != tempMemory.at(i).gameMap[posY][posX])
            continue;
        
        if(isEqualMap(gameStep.gameMap, tempMemory.at(i).gameMap))
           return true;
    }
    
    return false;
}

void printMap(const TDMatrix& gameMap)
{
    cout << endl;
    for (int i = 0; i < gameMap.size(); ++i) {
        cout << "\t";
        for (int j = 0; j < gameMap.at(0).size(); ++j) {
            cout << gameMap.at(i).at(j);
        }
        cout << endl;
    }
    cout << endl << endl;
}

inline double euclideanDistance(Point first, Point second)
{
    return sqrt(powf(posY(first) - posY(second), 2) +
                powf(posX(first) - posX(second), 2));
}

vector<Point> getStepsByOrder(const Step& currentStep, int nextTarget, Point endPoint, const GateVector& gates)
{
    // define all possible movement
    const Point movement[4] =  {Point(-1, 0),   // Down
                                Point(1, 0),    // Up
                                Point(0, 1),    // Right
                                Point(0, -1)};  // Left
    
    size_t  mapX = currentStep.gameMap.at(0).size(),
            mapY = currentStep.gameMap.size();
    int     posY = posY(top(currentStep.mouseLocations));
    int     posX = posX(top(currentStep.mouseLocations));
    
    vector<pair<Point, int>> pointDistance;

    for (int i = 0; i < 4; ++i) {
        
        Point newPoint(posY - posY(movement[i]), posX - posX(movement[i]));
        
        if (inBound(newPoint, mapY, mapX))
        {
            if(currentStep.gameMap[posY(newPoint)][posX(newPoint)] != BLOCK)
            {
                if (nextTarget == GOAL) {
                    pointDistance.push_back(pair<Point, int>(newPoint, euclideanDistance(endPoint, newPoint)));
                } else {
                    double minLen = 999999.999999;
                    for (int l = 0; l < gates.size(); ++l) {
                        double min = euclideanDistance(gates.at(l).selfLocation, newPoint);
                        
                        if(min < minLen) {
                            minLen = min;
                        }
                    }
                    
                    pointDistance.push_back(pair<Point, int>(newPoint, minLen));
                }
            }
        }
    }
    
    sort(pointDistance.begin(), pointDistance.end(), [](pair<Point, int> a, pair<Point, int> b) {
        return b.second < a.second;
    });
    
    vector<Point> points;
    for (int i = 0; i < pointDistance.size(); ++i)
         points.push_back(pointDistance.at(i).first);
    
    return points;
}

void solve(const TDMatrix& gameMap, int numberOfGateMustPass, const GateVector& gates){
    Point startPoint(-1, -1), endPoint(-1, -1);
    size_t mapX = gameMap.at(0).size(),
            mapY = gameMap.size();
    vector<Step> tempMemory,
                solvedMemory; // all not goal tempMemory will move into solvedMemory
    
    Step minPath;
    int minPathLen = INT_MAX;
    
    // find start and end points
    for (int i = 0; i < mapY; ++i) {
        for (int j = 0; j < mapX; ++j) {
            if(gameMap[i][j] == START){
                posY(startPoint) = i;
                posX(startPoint) = j;
            }
            else if (gameMap[i][j] == GOAL){
                posY(endPoint) = i;
                posX(endPoint) = j;
            }
        }
    }
    
    int passedGates = 0, loops = 0;
    vector<Point> startVector;
    startVector.push_back(startPoint);
    tempMemory.push_back(createStep(gameMap, startVector, passedGates, gates));
    
    cout << " loop started ..." << endl;
    while (tempMemory.size() > 0) {
        ++loops;
        
        Step temp = top(tempMemory);
        solvedMemory.push_back(temp);
        tempMemory.pop_back();
        
        int pathLen = calculatePathLen(temp.gameMap);
        if(pathLen > minPathLen)
            continue;
    
        
        if (isGoal(top(temp.mouseLocations), endPoint, temp.passedGate, numberOfGateMustPass))
        {
            if (minPathLen > pathLen)
            {
                minPathLen = pathLen;
                minPath = temp;
                
                cout << "\tminPathLen :" << minPathLen << "\tNumber of itrations :"
                     << loops << endl << "\t=> ";
                for (int i = 0; i < temp.mouseLocations.size(); ++i)
                {
                    cout << "(" << posY(temp.mouseLocations[i]) + 1 << ", "
                         << posX(temp.mouseLocations[i]) + 1 << ") ";
                    cout << (i % 4 == 3 ? "\n\t   " : "");
                }
                
                printMap(minPath.gameMap);
            }
            continue;
        }
        
        vector<Point> stepOrders = getStepsByOrder(temp, temp.passedGate >= numberOfGateMustPass ? GOAL : TUNNEL,
                                     endPoint, gates);
        
        for (int i = 0; i < stepOrders.size(); ++i) {
            vector<Point> steps(temp.mouseLocations);
            steps.push_back(stepOrders[i]);
            
            Step newStep = createStep(temp.gameMap, steps, temp.passedGate, gates);
            if(!isDuplicated(newStep, solvedMemory)){
                tempMemory.push_back(newStep);
            }
        }
    }
    
    cout << " Done !" << endl << "\tFinal stack size : " << solvedMemory.size() + tempMemory.size() << endl
         << "\tNumber of itrations :" << loops << endl;
    
}

int main(int argc,  char * argv[]) {
    
    char* path = "/Users/mohammadjamali/Desktop/testBench.txt";
    
    cout << " Opening file ... " << endl;
    ifstream input(path, ifstream::in);
    
    unsigned numberOfProblems = 0, numberOfGates = 0, gateMustPass = 0;
    input >> numberOfProblems;
    
    // boardSize.first : column . boardSize.second : row
    
    for (int i = 0; i < numberOfProblems; ++i) {
        
        cout << " Creating map #" << i + 1 << " ... " << endl;
        
        Point boardSize(0, 0);
        input >> boardSize.first >> boardSize.second
              >> numberOfGates >> gateMustPass;
        
        TDMatrix map; // map(y, x)
        GateVector gates;
        
        map.resize(boardSize.first);
        for(int j = 0; j < boardSize.first; j++){
            vector<char> map_row;
            map_row.resize(boardSize.second);
            for(int k = 0; k < boardSize.second; k++){
                do{
                    map_row[k] = input.get();
                }while(iscntrl(map_row[k]));
                
                if (isalpha(map_row[k]) && map_row[k] != START &&
                    map_row[k] != BLOCK && map_row[k] != GOAL){
                    
                    GateInfo gateInfo;
                    gateInfo.symbol = map_row[k];
                    gateInfo.selfLocation = Point(j, k);
                    gateInfo.destinationLocation = Point(-1, -1);
                    
                    gates.push_back(gateInfo);
                }
            }
            
            map[j] = map_row;
        }
        
        for (int i = 0; i < numberOfGates; ++i) {
            char symbol;
            int locX, locY;
            input >> symbol >> locY >> locX;
            for (int j = 0; j < gates.size(); ++j) {
                if(gates.at(j).symbol == symbol)
                    gates.at(j).destinationLocation = Point(locY - 1, locX - 1);
            }
        }
        
        input.close();
        
        cout << " Done! this app is now on going..." << endl;
        
        solve(map, gateMustPass, gates);
    }
    
    
    return 0;
}
