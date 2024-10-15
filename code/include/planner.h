#ifndef PLANNER_H
#define PLANNER_H

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <vector>
#include <utility>
#include <memory>
#include "utils.h"

// struct hash_pair { //hash function for pair
//     template <class T1, class T2>
//     std::size_t operator()(const std::pair<T1, T2>& p) const
//     {
//         auto hash1 = std::hash<T1>{}(p.first);
//         auto hash2 = std::hash<T2>{}(p.second);
//         return hash1 ^ (hash2<<1);
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

// // Define the CompareSimpleState comparator
// struct CompareSimpleState { //compare function for simplestate
//     bool operator()(const simplestate& s1, const simplestate& s2) {
//         return s1.g > s2.g;
//     }
// };

bool is_free(int* map, int x, int y, int x_size, int y_size, int collision_thresh);
bool in_map(int x, int y, int x_size, int y_size);
int cost(int* map, int x, int y, int x_size, int y_size);
int manhattan(int x1, int y1, int x2, int y2);

void findGoal(
    int* target_traj,
    int target_steps,
    int robotposeX,
    int robotposeY, 
    int& minx, int& miny);

void get_goalpoints(int robotposeX, int robotposeY, int* target_traj, int target_steps, std::unordered_map<std::pair<int, int>, int, hash_pair>& trajpoints);

bool InTraj(std::unordered_map<std::pair<int, int>, int, hash_pair>& trajpoints, std::pair<int, int> coord);

int findTrajIndex(int* target_traj, int target_steps, int x, int y);

int closestTrajIndex(int* target_traj, int target_steps, int x, int y);

void backward_djikstra(
    int* map, 
    int collision_thresh,
    int x_size,
    int y_size,
    int* target_traj,
    int target_steps,
    int goalX,
    int goalY,
    int robotposeX,
    int robotposeY,
    std::unordered_map<std::pair<int, int>, int, hash_pair>& heuristic,
    std::unordered_map<std::pair<int, int>, int, hash_pair>& trajpoints);
    //std::priority_queue<simplestate, std::vector<simplestate>, CompareSimpleState>& simpleopen_list,
    //std::unordered_set<std::pair<int, int>, hash_pair>& simpleclosed_list);

void Astar(
    int* map, 
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int targetposeX,
    int targetposeY,
    int* target_traj,
    int target_steps,
    int goalX,
    int goalY,
    std::unordered_map<std::pair<int, int>, int, hash_pair>& heuristic,
    //std::priority_queue<state, std::vector<state>, CompareState>& open_list,
    //std::unordered_map<std::pair<int, int>, state, hash_pair>& closed_list,
    std::stack<std::shared_ptr<state>>& path,
    std::unordered_map<std::pair<int, int>, int, hash_pair>& gval,
    std::unordered_map<std::pair<int, int>, int, hash_pair>& trajpoints);

void check_path(std::stack<std::shared_ptr<state>> path);

// Declare the plan function
void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr,
    int goalX,
    int goalY,
    std::stack<std::shared_ptr<state>>& path
    );

#endif // PLANNER_H