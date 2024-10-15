#include "../include/utils.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits.h>
//my stuff
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <memory>

double dist_heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// struct state{ //state for A*
//     std::pair<int, int> coord;
//     bool is_open;
//     double g;
//     double h;
//     double f;
//     long time;
//     state * parent;

// state::state(int x, int y, bool open, int gc, int hc, state* parent) 
//   : coord(x, y), g(gc), h(hc), f(g + h), parent(parent), is_open(open) {}

state::state(int x, int y, bool open, int gc, int hc, std::shared_ptr<state> parent){
    coord = std::make_pair(x, y);
    is_open = open;
    g = gc;
    h = hc;
    f = g + h;
    this->parent = parent;
}

state::state(int x, int y, int xg, int yg, int cost, bool o, std::shared_ptr<state> parent){
    coord = std::make_pair(x, y);
    is_open = true;;
    h = dist_heuristic(x, y, xg, yg);
    f = g + h;
    this->parent = parent;
    g= parent->g + cost;
}

state::state(int x, int y, int hc, int cost, std::shared_ptr<state> parent){
    coord = std::make_pair(x, y);
    is_open = true;;
    h = hc;
    this->parent = parent;
    g= parent->g + cost;
    f = g + h;
    steps = 0;
}

state::state(int x, int y, std::unordered_map<std::pair<int, int>, int, hash_pair>& heuristic, int gv, std::shared_ptr<state> parent){
    coord = std::make_pair(x, y);
    is_open = true;
    this->parent = parent;
    g= gv;
    steps = 0;
    if(!isInHeuristic2(heuristic, x, y)){
        heuristic[coord] = INT_MAX;
        //std::cout << "heuristic not found" << std::endl;
        f = INT_MAX;
    }
    else{
        h = heuristic[coord];
        
        f = g + h;
    }
}

void state::printcoord(){
    std::cout << "x: " << coord.first << " y: " << coord.second << " f: " << f << std::endl;
}

void state::close(){
    is_open = false;
}

void state::updatef(){
    f = g + h;
}

void state::updateg(int gc){
    g = gc;
    updatef();
}

void state::increase_steps(){
    steps++;
}


// struct CompareState { //compare function for state
//     bool operator()(const state* s1, const state* s2) {
//         return s1->f > s2->f;
//     }
// };

// struct simplestate{ //simple state for backward djikstra
//     std::pair<int, int> coord;
//     int g;
simplestate::simplestate(int x, int y){
    coord = std::make_pair(x, y);
    g=10000;
}

simplestate::simplestate(int x, int y, int gc){
    coord = std::make_pair(x, y);
    g=gc;
}


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

void printOpenList(const std::priority_queue<std::shared_ptr<state>, std::vector<std::shared_ptr<state>>, CompareState>& open_list) {
    std::cout << "Open List: ";
    auto temp = open_list;
    while (!temp.empty()) {
        temp.top()->printcoord();
        temp.pop();
    }
    std::cout << std::endl;
}

bool isInClosedList(const std::unordered_set<std::pair<int, int>, hash_pair>& closed_list, int x, int y) {
    return closed_list.find(std::make_pair(x, y)) != closed_list.end();
}

bool isInClosedList2(const std::unordered_set<std::pair<int, int>, hash_pair>& closed_list, const simplestate* s) {
    if (s == nullptr) return false;
    return closed_list.find(s->coord) != closed_list.end();
}

bool isInHeuristic(const std::unordered_map<std::pair<int, int>, int, hash_pair>& heuristic, const simplestate& s) {
    return heuristic.find(s.coord) != heuristic.end();
}

bool isInHeuristic2(const std::unordered_map<std::pair<int, int>, int, hash_pair>& heuristic, int x, int y) {
    return heuristic.find(std::make_pair(x, y)) != heuristic.end();
}

bool isInStructClosedList(const std::unordered_map<std::pair<int, int>, std::shared_ptr<state>, hash_pair>& closed_list, int x, int y) {
    return closed_list.find(std::make_pair(x, y)) != closed_list.end();
}

bool isInStructClosedList2(const std::unordered_map<std::pair<int, int>, std::shared_ptr<state>, hash_pair>& closed_list, const std::shared_ptr<state>& s) {
    if (s == nullptr) return false;
    return closed_list.find(s->coord) != closed_list.end();
}

bool isInGval(const std::unordered_map<std::pair<int, int>, int, hash_pair>& gval, int x, int y) {
    return gval.find(std::make_pair(x, y)) != gval.end();
}
