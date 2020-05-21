//
//  main.cpp
//  Horse
//
//  Created by Mohammad Jamali on 7/19/16.
//  Copyright © 2016 Mohammad Jamali. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

typedef pair<int, int> Point;
typedef vector<vector<char>> Array;

#define posY(x) x.first
#define posX(x) x.second
#define top(x)  x.at(x.size() - 1)

#define HORSE 'H'
#define BLOCK 'X'

void print(char **map){
    for (int i = 0; i < 8; i++) {
        cout << map[i] << endl;
    }
    cout << endl;
}

void print(int **input){
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++)
            cout << (input[i][j] == INT_MAX ? 0 : input[i][j] ) << "\t";
        cout  << endl;
            
    }
    cout << endl;
}

char** copyMap(char input[8][8+1]){
    char** map = new char*[8];
    for (int i = 0; i < 8; i++) {
        map[i] = new char[9];
        for (int j = 0; j < 9; j++) {
            map[i][j] = input[i][j];
        }
    }
    return map;
}

char** copyMap(char** input){
    char** map = new char*[8];
    for (int i = 0; i < 8; i++) {
        map[i] = new char[9];
        for (int j = 0; j < 9; j++) {
            map[i][j] = input[i][j];
        }
    }
    return map;
}

void deleteMap(char** input)
{
    for (int i = 0; i < 8; i++)
        delete[] input[i];
    delete[] input;
}

inline int sign(int x)
{
    return x >= 0 ? 1 : -1;
}

vector<Point> getNextPointsByOrder(const Point differentialMovement[8], int** potentialDistance, char map[8][9]
                                   ,vector<Point> currentStep)
{
    vector<pair<Point, int>> moves;
    for (int move = 0; move < 8; ++move) { // for each differentialMovement
        
        int destPosY = posY(top(currentStep)) + posY(differentialMovement[move]),
            destPosX = posX(top(currentStep)) + posX(differentialMovement[move]),
            posY = posY(top(currentStep)),
            posX = posX(top(currentStep));
        bool isBlock = false;
        
        if(destPosX >= 0 && destPosY >= 0 && destPosX < 8 && destPosY < 8)
        {
            for (int y = 0; y < abs(posY(differentialMovement[move])); ++y) {
                posY += sign(posY(differentialMovement[move]));
                if(map[posY][posX] == BLOCK) {
                    isBlock = true;
                }
            }
            
            for (int x = 0; x < abs(posX(differentialMovement[move])); ++x) {
                posX += sign(posX(differentialMovement[move]));
                if(map[posY][posX] == BLOCK) {
                    isBlock = true;
                }
            }
            
            if (!isBlock) {
                moves.push_back(pair<Point, int>(Point(destPosY, destPosX),
                                                 potentialDistance[destPosY][destPosX] == 0 ? 999 :
                                                 potentialDistance[destPosY][destPosX]));
                continue;
            }
            else {
                posY = posY(top(currentStep));
                posX = posX(top(currentStep));
                isBlock = false;
            }
            
            for (int x = 0; x < abs(posX(differentialMovement[move])); ++x) {
                posX += sign(posX(differentialMovement[move]));
                if(map[posY][posX] == BLOCK) {
                    isBlock = true;
                }
            }
            
            for (int y = 0; y < abs(posY(differentialMovement[move])); ++y) {
                posY += sign(posY(differentialMovement[move]));
                if(map[posY][posX] == BLOCK) {
                    isBlock = true;
                }
            }
                                                 
            if (!isBlock) {
                moves.push_back(pair<Point, int>(Point(destPosY, destPosX),
                                                 potentialDistance[destPosY][destPosX] == 0 ? 999 :
                                                 potentialDistance[destPosY][destPosX]));
            }
        }
    }
    
    sort(moves.begin(), moves.end(), [](pair<Point, int> a, pair<Point, int> b) { return b.second < a.second; });
    
    vector<Point> result;
    for (int i = 0; i < moves.size(); ++i)
        result.push_back(moves.at(i).first);
    
    return result;
}

inline bool isGoal(Point goal, Point endPoint)
{
    return (posY(goal) == posY(endPoint)) && (posX(goal) == posX(endPoint));
}

void generatePotentialDistanceHandler(int depth, Point point, int** map, const Point differentialMovement[8]) {
    if(--depth > 0) {
        int destPosY, destPosX;
        for (int move = 0; move < 8; ++move) { // for each differentialMovement
            destPosY = posY(point) + posY(differentialMovement[move]);
            destPosX = posX(point) + posX(differentialMovement[move]);
            
            if(destPosX >= 0 && destPosY >= 0 && destPosX < 8 && destPosY < 8)
            {
                if(map[destPosY][destPosX] != INT_MAX && (map[destPosY][destPosX] == 0 ||
                                                          map[destPosY][destPosX] >= map[posY(point)][posX(point)]))
                    map[destPosY][destPosX] += map[posY(point)][posX(point)] + 1;
            }
        }
        
        for (int move = 0; move < 8; ++move) { // for each differentialMovement
            destPosY = posY(point) + posY(differentialMovement[move]);
            destPosX = posX(point) + posX(differentialMovement[move]);
            
            if(destPosX >= 0 && destPosY >= 0 && destPosX < 8 && destPosY < 8)
            {
                if(map[destPosY][destPosX] != INT_MAX && (map[destPosY][destPosX] == 0 ||
                                                          map[destPosY][destPosX] >= map[posY(point)][posX(point)]))
                    generatePotentialDistanceHandler(depth, Point(destPosY, destPosX), map, differentialMovement);
            }
        }
    }
}

int** generatePotentialDistance(int depth, Point endPoint, const char map[8][9], const Point differentialMovement[8]) {
    int** potentialDistance = new int*[8];
    
    for (int i = 0; i < 8; ++i) {
        potentialDistance[i] = new int[8];
        for (int j = 0; j < 8; ++j) {
            if (map[i][j] == 'X') {
                potentialDistance[i][j] = INT_MAX;
            }
            else {
                potentialDistance[i][j] = 0;
            }
        }
    }
    
    potentialDistance[posY(endPoint)][posX(endPoint)] = 1;
    
    generatePotentialDistanceHandler(depth, endPoint, potentialDistance, differentialMovement);
    
    return potentialDistance;
}

const bool duplicate(const vector<vector<Point>> memory, const vector<Point> steps){
    
    for (int i = memory.size() - 1; i >= 0 ; --i) {
        if(memory[i].size() == steps.size()) {
            bool isEqual = true;
            for (int j = 0; j < steps.size(); ++j) {
                if(posX(steps[j]) != posX(memory[i][j]) || posY(steps[j]) != posY(memory[i][j])) {
                    isEqual = false;
                    break;
                }
            }
            
            if(isEqual){
                return true;
            }
        }
    }
    
    return false;
}

vector<Point> rideHorse(char map[8][8+1], Point start, Point end)
{
    const Point differentialMovement[8] =  {
        Point( 1, 2),   Point(2, -1),
        Point(-1, 2),   Point(2,  1),
        Point( 1,-2),   Point(-2,-1),
        Point(-1,-2),   Point(-2, 1)};
    
    int minStepCount = INT_MAX;
    vector<Point> minSteps;
    int** potentialDistance = generatePotentialDistance(8, end, map, differentialMovement);
    
    cout << " potentialDistance :> " << endl;
    print(potentialDistance);
    
    vector<Point> firstStep;
    firstStep.push_back(start);
    
    vector<vector<Point>> memory, solved;
    memory.push_back(firstStep);
    
    int loop = 0, minLoop = 0;
    while (memory.size() > 0) {
        loop++;
        vector<Point> temp = top(memory);
        memory.pop_back();
        solved.push_back(temp);
        
        if(temp.size() > 30 || temp.size() >= minStepCount){
            continue;
        }
        
        if (isGoal(top(temp), end)) {
            if(minStepCount > temp.size()) {
                minStepCount = (int)temp.size();
                minSteps = temp;
                minLoop = loop;
            }
            continue;
        }
        
        vector<Point> steps = getNextPointsByOrder(differentialMovement, potentialDistance, map, temp);
        
        for (int i = 0; i < steps.size(); ++i) {
            vector<Point> nextSteps(temp);
            nextSteps.push_back(steps[i]);
            
            //if(!duplicate(solved, nextSteps))
                memory.push_back(nextSteps);
        }
    }
    
    cout << " #" << minLoop << " iterations when minimum found ..."  << endl;
    
    return minSteps;
}

int main(int argc, const char * argv[]) {
    
    char map[8][8+1] = {"........",
                        ".....X..",
                        "..X..X..",
                        "..X..X..",
                        "..XXXX..",
                        "..X..X..",
                        "..X.....",
                        "........"};
    
    char* path = "/Users/mohammadjamali/Desktop/horseTestBench.txt";
    
    cout << " Opening file ... " << endl;
    ifstream input(path, ifstream::in);
    
    unsigned numberOfProblems = 0;
    input >> numberOfProblems;
    
    for (int i = 0; i < numberOfProblems; ++i) {
        Point startPoint(-1, -1), endPoint(-1, -1);
        input   >> posY(startPoint) >> posX(startPoint)
                >> posY(endPoint) >> posX(endPoint);
        
        vector<Point> steps = rideHorse(map, startPoint, endPoint) ;
        cout << " Problem #" << i + 1 << " : " << steps.size() - 1 << " steps" << endl;
        
        char** mapTemp = copyMap(map);
        
        for (int i = 0; i < steps.size(); ++i) {
            mapTemp[posY(steps[i])][posX(steps[i])] = HORSE;
            cout << "(" << posY(steps[i]) << ", " << posX(steps[i]) << ") ";
        }
        cout << endl << endl;
        
        print(mapTemp);
        deleteMap(mapTemp);
    }

    return 0;
}
