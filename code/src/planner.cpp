/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include "../include/utils.h"
#include <math.h>
#include <limits.h>
#include <iostream>
#include <memory>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

bool is_free(int* map, int x, int y, int x_size, int y_size, int collision_thresh){
    return (map[GETMAPINDEX(x, y, x_size, y_size)] >= 0) && (map[GETMAPINDEX(x, y, x_size, y_size)] < collision_thresh);
}

bool in_map(int x, int y, int x_size, int y_size){
    return (x >= 1) && (x <= x_size) && (y >= 1) && (y <= y_size);
}

bool check_limit(int x, int y, std::shared_ptr<state> parent, int target_steps){
    if(parent == nullptr){
        return true;
    }
    if(parent->steps + 1 > target_steps){
        return false;
    }
    else{
        return true;
    }
}

int cost(int* map, int x, int y, int x_size, int y_size){
    return map[GETMAPINDEX(x, y, x_size, y_size)];
}

void findGoal(
    int* target_traj,
    int target_steps,
    int robotposeX,
    int robotposeY, 
    int& minx, int& miny){

        int dist;
        int offset = target_steps/3.3;
        bool done = false;
        //std::cout << "start " << target_traj[0] << " " << target_traj[target_steps] << std::endl;
        for(int i=0; i<target_steps; i++){

            dist= MAX(abs(robotposeX - target_traj[i]), abs(robotposeY - target_traj[i+target_steps]));

            if(dist < i-offset){
                minx = target_traj[i];
                miny = target_traj[i+target_steps];
                done = true;
                break;
            }

            
        }

        if (!done){
            minx = target_traj[target_steps-1];
            miny = target_traj[target_steps*2-1];
        }
    
}

int manhattan(int x1, int y1, int x2, int y2){
    return abs(x1-x2) + abs(y1-y2);
}

void get_goalpoints(int robotposeX, int robotposeY, int* target_traj, int target_steps, std::unordered_map<std::pair<int, int>, int, hash_pair>& trajpoints){
    int dist;
    int offset = target_steps/4;
    for(int i=0; i<target_steps; i++){
        dist= 0.6*manhattan(robotposeX, robotposeY, target_traj[i], target_traj[i+target_steps]);
        if(dist < i-offset){
            trajpoints[std::make_pair(target_traj[i], target_traj[i+target_steps])] = 500*i;//100*i ;//0.1*(target_steps - i);
        }
    }
}

bool InTraj(std::unordered_map<std::pair<int, int>, int, hash_pair>& trajpoints, std::pair<int, int> coord){
    return trajpoints.find(coord) != trajpoints.end();
}

int findTrajIndex(int* target_traj, int target_steps, int x, int y){
    for(int i=0; i<target_steps; i++){
        if(target_traj[i] == x && target_traj[i+target_steps] == y){
            return i;
        }
    }
    return -1;
}

int closestTrajIndex(int* target_traj, int target_steps, int x, int y){
    int min = INT_MAX;
    int index = -1;
    for(int i=0; i<target_steps; i++){
        int dist = manhattan(target_traj[i], target_traj[i+target_steps], x, y);
        if(dist < min){
            min = dist;
            index = i;
        }
    }
    return index;
}

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
    std::unordered_map<std::pair<int, int>, int, hash_pair>& trajpoints)
    //std::priority_queue<simplestate, std::vector<simplestate>, CompareSimpleState>& simpleopen_list,
    //std::unordered_set<std::pair<int, int>, hash_pair>& simpleclosed_list)
{
    std::unordered_set<std::pair<int, int>, hash_pair> simpleclosed_list;
    std::priority_queue<simplestate, std::vector<simplestate>, CompareSimpleState> simpleopen_list;

    //std::priority_queue<simplestate*, std::vector<simplestate*>, CompareSimpleState> simpleopen_list;

    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    int goalposeX = target_traj[(target_steps-1)];
    int goalposeY = target_traj[(target_steps-1)*2+1];

    // int goalposeX  = target_traj[target_steps/2];
    // int goalposeY  = target_traj[target_steps/2 +target_steps];
    
    //std::cout << "goalposeX: " << goalposeX << " goalposeY: " << goalposeY << std::endl;
    int poseX;
    int poseY;
    bool add = false;
    //simpleopen_list.push(simplestate(goalX,goalY)); //push(simplestate(goalposeX, goalposeY, 0));

    // for(int i=0; i<target_steps; i++){
    //     int x = target_traj[i];
    //     int y = target_traj[i+target_steps];
    //     if(0.5*manhattan(robotposeX, robotposeY,x,y) < i){
    //         add=true;
    //     }
    //     if(add){
    //         simpleopen_list.push(simplestate(x, y, 0));
    //     }
    // }
    get_goalpoints(robotposeX, robotposeY, target_traj, target_steps, trajpoints);

    for(auto it = trajpoints.begin(); it != trajpoints.end(); it++){
        simpleopen_list.push(simplestate(it->first.first, it->first.second, it->second));
    }


    while (!simpleopen_list.empty() && !isInClosedList2(simpleclosed_list, &simpleopen_list.top())){
        simplestate curr=simpleopen_list.top();     //gets the top element
        simpleopen_list.pop();                      //removes top element
        simpleclosed_list.insert(curr.coord);    //inserts the top element into closed list
        if(!isInHeuristic(heuristic, curr)){
            heuristic[curr.coord] = curr.g;
        }
        poseX = curr.coord.first;
        poseY = curr.coord.second;
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = poseX + dX[dir];
            int newy = poseY + dY[dir];

            if(in_map(newx,newy,x_size,y_size) && is_free(map, newx,newy,x_size,y_size, collision_thresh)){

                    if (!isInClosedList(simpleclosed_list, newx, newy)){

                        int newcost = cost(map, newx, newy, x_size, y_size);

                        if(!isInHeuristic2(heuristic, newx, newy)){
                            heuristic[std::make_pair(newx, newy)] = INT_MAX;
                            simpleopen_list.push(simplestate(newx, newy, curr.g + newcost));
                            heuristic[std::make_pair(newx, newy)] = curr.g + newcost;
                        }
                        else{
                            if (heuristic[std::make_pair(newx, newy)] > curr.g + newcost){
                                simpleopen_list.push(simplestate(newx, newy, curr.g + newcost));
                                heuristic[std::make_pair(newx, newy)] = curr.g + newcost;
                            }
                        }


                    }
                

            }

        }
    }

}

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
    std::unordered_map<std::pair<int, int>, int, hash_pair>& trajpoints)
{   
    std::priority_queue<std::shared_ptr<state>, std::vector<std::shared_ptr<state>>, CompareState> open_list;
    std::unordered_map<std::pair<int, int>, std::shared_ptr<state>, hash_pair> closed_list;
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    //std::cout << "Entering A* search" << std::endl;
    int goalposeX = target_traj[target_steps-1];
    int goalposeY = target_traj[target_steps-1+target_steps];
    // int goalposeX  = target_traj[target_steps/2];
    // int goalposeY  = target_traj[target_steps/2 +target_steps];

    // if(!isInHeuristic2(heuristic, robotposeX, robotposeY)){
    //     std::cout << "heuristic of start not found" << std::endl;
    // }
    std::shared_ptr<state> start = std::make_shared<state>(robotposeX, robotposeY, heuristic, 0, nullptr);
    start->increase_steps();
    open_list.push(start);
    int count = 0;
    while(!open_list.empty() ){//&& !isInStructClosedList2(closed_list, open_list.top())){
        std::shared_ptr<state> curr = open_list.top();
        open_list.pop();
        closed_list[curr->coord] = curr;
        curr->close();
        count++;
        //if(curr->coord == std::make_pair(goalX, goalY)){ //goal found
        if(InTraj(trajpoints, curr->coord)){
            //std::cout << "entered while loop" << std::endl;
            //std::cout << "curr->coord: " << curr->coord.first << " " << curr->coord.second << "curr->parent: " << curr->parent->coord.first << " " << curr->parent->coord.second << std::endl;
            int index = findTrajIndex(target_traj, target_steps, curr->coord.first, curr->coord.second);
            //int index2 = closestTrajIndex(target_traj, target_steps, curr->coord.first, curr->coord.second);
            //if((index<=target_steps-1)&&(abs(index-curr->steps)>6)){
            //    std::cout << "Expanding" << std::endl;
            //    break;
            //}
            bool b=false;
            while(curr->steps < target_steps-1){
                if(curr->steps < index){
                    b=true;
                    index -=1;
                    std::shared_ptr<state> newst = std::make_shared<state>(target_traj[index], target_traj[index+target_steps], 0, 0, curr);
                    newst->steps = curr->steps + 1;
                    curr = newst;
                    //std::cout << "adding " << curr->coord.first << " " << curr->coord.second << std::endl; 
                }
                // else if(!b){
                //     index +=1;
                //     std::shared_ptr<state> newst = std::make_shared<state>(target_traj[index], target_traj[index+target_steps], 0, 0, curr);
                //     newst->steps = curr->steps + 1;
                //     curr = newst;
                // }
                else{ break;}
            }
            //if(manhattan(curr->coord.first, curr->coord.second, target_traj[index], target_traj[index+target_steps]) > 35 && curr->steps < target_steps){
            //   break;
            //}
            path.push(curr);
            while(curr->parent != nullptr){
                path.push(curr->parent);
                curr = (curr->parent);
            }
            return;
        }
        int currposeX = curr->coord.first;
        int currposeY = curr->coord.second;
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = currposeX + dX[dir];
            int newy = currposeY + dY[dir];

            if(in_map(newx,newy,x_size,y_size) && is_free(map, newx,newy,x_size,y_size, collision_thresh) && check_limit(newx, newy, curr, target_steps)){
                if (!isInStructClosedList(closed_list, newx, newy)){

                    // if(goalposeX == newx && goalposeY == newy){
                    //     std::cout << "goal found in A* search" << std::endl;
                    // }

                    if(!isInGval(gval, newx, newy)){
                            gval[std::make_pair(newx, newy)] = INT_MAX;
                            //std::cout << "gval not found" << std::endl;
                    }

                    if(!isInHeuristic2(heuristic, newx, newy)){
                        //std::cout << "heuristic not found" << std::endl;
                    }

                    int newcost = cost(map, newx, newy, x_size, y_size);
                    if (gval[std::make_pair(newx, newy)] > curr->g + newcost){
                        std::shared_ptr<state> newstate = std::make_shared<state>(newx, newy, heuristic, curr->g + newcost, curr);
                        newstate->steps = curr->steps + 1;
                        open_list.push(newstate);
                        gval[std::make_pair(newx, newy)] = curr->g + newcost;
                    }

                }
            }
        }
    }
    // if (!isInGval(gval, goalX, goalY)){
    //     std::cout << "goal not found in A* search" << std::endl;
    // }
    //std::cout << "Number of iterations: " << count << std::endl; 

    
}
void check_path(std::stack<std::shared_ptr<state>> path){
    
    std::cout << "First point in path " << path.top()->coord.first << " " << path.top()->coord.second << std::endl;
    while(!path.empty()){
        if(path.size() == 1){
            std::cout << "Last point in path " << path.top()->coord.first << " " << path.top()->coord.second << std::endl;
            path.pop();
            break;
        }
        std::shared_ptr<state> curr = path.top();
        path.pop();
        std::shared_ptr<state> next = path.top();
        if(abs(curr->coord.first - next->coord.first) > 1 || abs(curr->coord.second - next->coord.second) > 1){
            std::cout << "ERROR: invalid path" << std::endl;
            return;
        }
    }
    std::cout << "Path is valid" << std::endl;
}

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
    )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    int goalposeX = target_traj[target_steps-1];
    int goalposeY = target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    double disttotarget;
    if (robotposeX == target_traj[target_steps-1] && robotposeY == target_traj[target_steps*2-1]){
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }
    if(!path.empty()){
        if(robotposeX == path.top()->coord.first && robotposeY == path.top()->coord.second){
        path.pop();
    }
    }
    

    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];
        
        
        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) //Check that robot still in map after move
        {
            if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {   
                if (!path.empty()){
                    
                
                    if(newx == path.top()->coord.first && newy == path.top()->coord.second){
                        bestX = dX[dir];
                        bestY = dY[dir];
                        path.pop();
                        //std::cout << "size of path: " << path.size() << std::endl;
                        break;
                    }
                }
                //else{if(newx == target_traj[target_steps-1] && newy == target_traj[target_steps*2-1]){
                else{if(newx == goalX && newy == goalY){
                    //std::cout << "Goal reached" << std::endl;
                    bestX = 0;
                    bestY = 0;
                    break;
                }
                //std::cout << "No more path available stuck here" << std::endl;
                }
                // disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                // if(disttotarget < olddisttotarget)
                // {
                //     olddisttotarget = disttotarget;
                //     bestX = dX[dir];
                //     bestY = dY[dir];
                // }
            }
        }
    }
    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
    return;
}