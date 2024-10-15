/*=================================================================
 *
 * runtest.cpp
 *
 *=================================================================*/
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
//my stuff
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <memory>

#include "../include/planner.h"
#include "../include/utils.h"


#ifndef MAPS_DIR
#define MAPS_DIR "maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "output"
#endif

// double dist_heuristic(int x1, int y1, int x2, int y2) {
//     return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
// }

// struct state{ //state for A*
//     std::pair<int, int> coord;
//     bool is_open;
//     double g;
//     double h;
//     double f;
//     long time;
//     state * parent;

//     state(int x, int y, bool open, double gc, double hc, state * parent =nullptr){
//         coord = std::make_pair(x, y);
//         is_open = open;
//         g = gc;
//         h = hc;
//         f = g + h;
//         this->parent = parent;
//     }

//     state(int x, int y, int xg, int yg, double cost, state * parent =nullptr){
//         coord = std::make_pair(x, y);
//         is_open = true;;
//         h = dist_heuristic(x, y, xg, yg);
//         f = g + h;
//         this->parent = parent;
//         g= parent->g + cost;
//     }

//     state(int x, int y, double hc, double cost, state * parent =nullptr){
//         coord = std::make_pair(x, y);
//         is_open = true;;
//         h = hc;
//         this->parent = parent;
//         g= parent->g + cost;
//         f = g + h;
//     }

//     void printcoord(){
//         std::cout << "x: " << coord.first << " y: " << coord.second << " f: " << f << std::endl;
//     }

//     void close(){
//         is_open = false;
//     }
// };

// struct CompareState { //compare function for state
//     bool operator()(const state* s1, const state* s2) {
//         return s1->f > s2->f;
//     }
// };

// struct simplestate{ //simple state for backward djikstra
//     std::pair<int, int> coord;
//     int g;
//     simplestate(int x, int y){
//         coord = std::make_pair(x, y);
//         g=10000;
//     }

//     simplestate(int x, int y, int gc){
//         coord = std::make_pair(x, y);
//         g=gc;
//     }
// };

// struct CompareSimpleState { //compare function for simplestate
//     bool operator()(const simplestate& s1, const simplestate& s2) {
//         return s1.g > s2.g;
//     }
// };

// struct hash_pair { //hash function for pair
//     template <class T1, class T2>
//     std::size_t operator()(const std::pair<T1, T2>& p) const
//     {
//         auto hash1 = std::hash<T1>{}(p.first);
//         auto hash2 = std::hash<T2>{}(p.second);
//         return hash1 ^ (hash2<<1);
//     }

// };


// void printOpenList(std::priority_queue<state*, std::vector<state*>, CompareState> open_list) {
//     std::cout << "Open List: ";
//     while (!open_list.empty()) {
//         state* s = open_list.top();
//         std::cout << "(" << s->coord.first << ", " << s->coord.second << ") " << s->f << " | ";
//         open_list.pop();
//     }
//     std::cout << std::endl;
// }

// bool isInClosedList(std::unordered_set<std::pair<int, int>, hash_pair>& closed_list, int x, int y) {
//     return closed_list.find(std::make_pair(x, y)) != closed_list.end();
// }

// bool isInClosedList2(std::unordered_set<std::pair<int, int>, hash_pair>& closed_list, const simplestate* s) {
//     if (s == nullptr) return false;
//     return closed_list.find(s->coord) != closed_list.end();
// }

// bool isInHeuristic(std::unordered_map<std::pair<int, int>, int, hash_pair>& heuristic, const simplestate& s) {
//     return heuristic.find(s.coord) != heuristic.end();
// }

// bool isInHeuristic2(std::unordered_map<std::pair<int, int>, int, hash_pair>& heuristic, int x, int y) {
//     return heuristic.find(std::make_pair(x, y)) != heuristic.end();
// }

int main(int argc, char *argv[])
{
    // READ PROBLEM
    if (argc != 2)
    {
        std::cout << "runtest takes exactly one command line argument: the map file" << std::endl;
        return -1;
    }

    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + argv[1];
    std::cout << "Reading problem definition from: " << mapFilePath << std::endl;

    std::ifstream myfile;
    myfile.open(mapFilePath);
    if (!myfile.is_open()) {
        std::cout << "Failed to open the file:" << mapFilePath << std::endl;
        return -1;
    }

    // read map size
    char letter;
    std::string line;
    int x_size, y_size;

    myfile >> letter;
    if (letter != 'N')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    
    myfile >> x_size >> letter >> y_size;
    std:: cout << "map size: " << x_size << letter << y_size << std::endl;

    // read collision threshold
    int collision_thresh;
    myfile >> letter;
    if (letter != 'C')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    
    myfile >> collision_thresh;
    std:: cout << "collision threshold: " << collision_thresh << std::endl;

    // read robot position
    int robotposeX, robotposeY;
    myfile >> letter;
    if (letter != 'R')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    
    myfile >> robotposeX >> letter >> robotposeY;
    std:: cout << "robot pose: " << robotposeX << letter << robotposeY << std::endl;

    // read trajectory
    std::vector<std::vector<int>> traj;

    do {
        std::getline(myfile, line);
    } while (line != "T");

    while (std::getline(myfile, line) && line != "M")
    {
        std::stringstream ss(line);
        int num1, num2;
        ss >> num1 >> letter >> num2;
        traj.push_back({num1, num2});
    }

    int target_steps = traj.size();  //num rows of map
    int* target_traj = new int[2*target_steps];
    for (size_t i = 0; i < target_steps; ++i)
    {
        target_traj[i] = traj[i][0];
        target_traj[i + target_steps] = traj[i][1]; //i + target steps storesy values, first half of array are x, second half are y values
    }

    std::cout << "target_steps: " << target_steps << std::endl;

    // read map
    int* map = new int[x_size*y_size];
    for (size_t i=0; i<x_size; i++)
    {
        std::getline(myfile, line);
        std::stringstream ss(line);
        for (size_t j=0; j<y_size; j++)
        {
            double value;
            ss >> value;

            map[j*x_size+i] = (int) value;
            if (j != y_size-1) ss.ignore();
        }
    }

    myfile.close();
    std::cout << "\nRunning planner" << std::endl;

    // CONTROL LOOP
    int curr_time = 0;
    int* action_ptr = new int[2];
    int targetposeX, targetposeY;
    int newrobotposeX, newrobotposeY;

    int numofmoves = 0;
    bool caught = false;
    int pathcost = 0;

    std::string outputDir = OUTPUT_DIR;
    std::string outputFilePath = outputDir + "/robot_trajectory.txt";
    std::ofstream output_file(outputFilePath);
    std::cout << "Writing robot trajectory to: " << outputFilePath << std::endl;
    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file: " << "../output/robot_trajectory.txt" << std::endl;
        return 1;
    }

    output_file << curr_time << "," << robotposeX << "," << robotposeY << std::endl;

    //######################################################################################################################
    

    std::unordered_map<std::pair<int, int>, int, hash_pair> heuristic;
    //std::unordered_set<std::pair<int, int>, hash_pair> simpleclosed_list;
    //std::unordered_map<std::pair<int, int>, state, hash_pair> closed_list;
    std::stack<std::shared_ptr<state>> path;
    std::unordered_map<std::pair<int, int>, int, hash_pair> gval;
    std::unordered_map<std::pair<int, int>, int, hash_pair> trajpoints;
    bool backward = true;
    int finalX = target_traj[target_steps-1];
    int finalY = target_traj[target_steps*2-1];
    int goalX=0;
    int goalY=0;
    findGoal(target_traj, target_steps, robotposeX, robotposeY, goalX, goalY);
    //std::cout << "goalX: " << goalX << " goalY: " << goalY << std::endl;
    while (true)
    {
        auto start = std::chrono::high_resolution_clock::now();

        targetposeX = target_traj[curr_time];
        targetposeY = target_traj[curr_time + target_steps];
        if(backward){
            //std::cout << robotposeX << " " << robotposeY << std::endl;
            backward_djikstra(map, collision_thresh, x_size, y_size, target_traj, target_steps, goalX, goalY, robotposeX, robotposeY, heuristic, trajpoints);
            backward = false;
            // std::cout << "backward done" << std::endl;
            // std::cout << "heuristic of robot: " << heuristic[std::make_pair(robotposeX, robotposeY)] << std::endl;
            // std::cout << "heuristic of target: " << heuristic[std::make_pair(finalX, finalY)] << std::endl;
    
            // std::cout << "Number of items in heuristic: " << heuristic.size() << std::endl;
            // std::cout << "Map size: " << x_size*y_size << std::endl;    
            Astar(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, targetposeX, targetposeY, target_traj, target_steps, goalX, goalY, heuristic, path, gval, trajpoints);
            // closed_list.clear();
            //std::cout << "A* done" << std::endl;
            //std::cout << "Length of path: " << path.size() << std::endl;
            //check_path(path);
            //auto stop = std::chrono::high_resolution_clock::now();
            //auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
            //std::cout << "Time taken for A*: " << duration2 << std::endl;
            //std::cout << "Top of path: ";
            //path.top().printcoord();
        }
        
        planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time, action_ptr, goalX, goalY,path);
        newrobotposeX = action_ptr[0];
        newrobotposeY = action_ptr[1];

        if (newrobotposeX < 1 || newrobotposeX > x_size || newrobotposeY < 1 || newrobotposeY > y_size)
        {
            std::cout << "ERROR: out-of-map robot position commanded\n" << std::endl;
            return -1;
        }

        if (map[(newrobotposeY-1)*x_size + newrobotposeX-1] >= collision_thresh)
        {
            std::cout << "ERROR: planned action leads to collision\n" << std::endl;
            return -1;
        }

        if (abs(robotposeX-newrobotposeX)>1 || abs(robotposeY-newrobotposeY)>1)
        {
            std::cout << "ERROR: invalid action commanded. robot must move on 8-connected grid.\n" << std::endl;
            return -1;
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();

        int movetime = std::max(1, (int)std::ceil(duration));

        if (newrobotposeX == robotposeX && newrobotposeY == robotposeY)
            numofmoves -= 1;
        
        if (curr_time + movetime >= target_steps)
            break;
        
        curr_time = curr_time + movetime;
        numofmoves = numofmoves + 1;
        pathcost = pathcost + movetime*map[(robotposeY-1)*x_size + robotposeX-1];

        robotposeX = newrobotposeX;
        robotposeY = newrobotposeY;

        output_file << curr_time << "," << robotposeX << "," << robotposeY << std::endl;

        // check if target is caught
        float thresh = 0.5;
        targetposeX = target_traj[curr_time];
        targetposeY = target_traj[curr_time + target_steps];
        if (abs(robotposeX - targetposeX) <= thresh && abs(robotposeY-targetposeY) <= thresh)
        {
            caught = true;
            break;
        }
    }

    output_file.close();

    std::cout << "\nRESULT" << std::endl;
    std::cout << "target caught = " << caught << std::endl;
    std::cout << "time taken (s) = " << curr_time << std::endl;
    std::cout << "moves made = " << numofmoves << std::endl;
    std::cout << "path cost = " << pathcost << std::endl;

    delete target_traj;
    delete map;

    return 0;
}