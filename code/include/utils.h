#ifndef UTILS_H
#define UTILS_H

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <utility> // For std::pair

struct hash_pair { //hash function for pair
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const
    {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ (hash2<<1);
    }

};

struct state{ //state for A*
    std::pair<int, int> coord;
    bool is_open;
    int g;
    int h;
    int f;
    int steps;
    std::shared_ptr<state> parent;

    state() : coord(0, 0), g(0), h(0), f(0), parent(nullptr), is_open(true) {}

    state(int x, int y, bool open, int gc, int hc=0, std::shared_ptr<state> parent =nullptr);

    state(int x, int y, int xg, int yg, int cost, bool o, std::shared_ptr<state> parent =nullptr);

    state(int x, int y, int hc, int cost, std::shared_ptr<state> parent =nullptr);

    state(int x, int y, std::unordered_map<std::pair<int, int>, int, hash_pair>& heuristic, int cost, std::shared_ptr<state> parent=nullptr);

    void printcoord();

    void close();
    
    void updatef();

    void updateg(int gc);
    
    void increase_steps();
};

struct CompareState { //compare function for state
    bool operator()(const std::shared_ptr<state>& s1, const std::shared_ptr<state>& s2) {
        if (s1->f == s2->f){
            return s1->g > s2->g;
        }
        return s1->f > s2->f;
    }
};

struct simplestate{ //simple state for backward djikstra
    std::pair<int, int> coord;
    int g;
    simplestate(int x, int y);

    simplestate(int x, int y, int gc);
};

struct CompareSimpleState { //compare function for simplestate
    bool operator()(const simplestate& s1, const simplestate& s2) {
        return s1.g > s2.g;
    }
};



void printOpenList(const std::priority_queue<std::shared_ptr<state>, std::vector<std::shared_ptr<state>>, CompareState>& open_list);
bool isInClosedList(const std::unordered_set<std::pair<int, int>, hash_pair>& closed_list, int x, int y);
bool isInClosedList2(const std::unordered_set<std::pair<int, int>, hash_pair>& closed_list, const simplestate* s);
bool isInHeuristic(const std::unordered_map<std::pair<int, int>, int, hash_pair>& heuristic, const simplestate& s);
bool isInHeuristic2(const std::unordered_map<std::pair<int, int>, int, hash_pair>& heuristic, int x, int y);
bool isInStructClosedList(const std::unordered_map<std::pair<int, int>, std::shared_ptr<state>, hash_pair>& closed_list, int x, int y);
bool isInStructClosedList2(const std::unordered_map<std::pair<int, int>, std::shared_ptr<state>, hash_pair>& closed_list, const std::shared_ptr<state>& s);
bool isInGval(const std::unordered_map<std::pair<int, int>, int, hash_pair>& gval, int x, int y);

#endif // UTILS_H