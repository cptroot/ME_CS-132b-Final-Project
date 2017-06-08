#include <stdio.h>
#include "Dstar.hpp"
#include <algorithm>

std::vector<coordinate> Dstar::init_path() {
    cell * goal_c = get_ptr(goal);
    *goal_c = cell(0, 0, goal_c, Tag::Open, goal);
    
    while (1) {
        get_kmin_result k_min = process_state();
        cell * start_c = get_ptr(start);
        if ((k_min.exists == false) || (start_c->t == Tag::Closed)) {
            return get_path(start_c);
        }
    }
}

void change_map(octomap::AbstractOcTree*) {
    for (int i = 0; i < get_neighhors.size(); i++) {
        coordinate locate = get_neighhors[i].loc;
    }
}


std::vector<coordinate> Dstar::navigate_map(coordinate curr) {
    while(1) {
        get_kmin_result k_min = process_state();
        
        if ((k_min.exists == false) || (curr.h <= k_min.kmin)) && !(curr.t == Tag::Open) {
            return get_path();
        }
    }
}

void Dstar::insert(cell * curr, double new_h) {
    if (curr->t == Tag::New) {
        curr->h = new_h;
        curr->k = new_h;
        curr->t = Tag::Open;
        put_open(curr);
    }
    elif (curr->t = Tag::Open) {
        curr->h = new_h;
        curr->k = std::min(curr->k, new_h);
        // Do STUFF
    }
    elif (curr->t == Tag::Closed) {
        curr->h = new_h;
        curr->k = std::min(curr->h, new_h);
        curr->t = Tag::Open;
        put_open(curr);
    }
}

//void Dstar::change_map(octomap::AbstractOcTree*);
//std::vector<coordinate> Dstar::navigate_map(coordinate curr);
//double Dstar::modify_costs(cell curr1, cell curr2, float new_cost);
//void Dstar::insert(cell curr, double new_h);
//get_kmin_result Dstar::process_state();

Dstar::Dstar(coordinate size, coordinate start, coordinate goal) :
        size(size), start(start), goal(goal) {
    world = new cell[size.x * size.y];
    costs = new double[size.x * size.y];
}

Dstar::~Dstar() {
    delete world;
}

cell Dstar::get(coordinate n) {
    return world[n.x + n.y * size.x];
}

cell * Dstar::get_ptr(coordinate n) {
    return &world[n.x + n.y * size.x];
}

void Dstar::put(coordinate n, cell curr) {
    *get_ptr(n) = curr;   
}

std::tuple<double, cell *> Dstar::get_open() {
    if (!pqueue.empty()) {
        auto item = pqueue.top();
        pqueue.pop();
        coordinate location = std::get<1>(item);
        cell *curr = get_ptr(location);
        curr->t = Tag::Closed;

        return std::make_tuple(std::get<0>(item), curr);
        
    }
    else {
        return std::make_tuple(-1.0, (cell *)NULL);
    }
}

void Dstar::put_open(cell * curr) {
    curr->t = Tag::Open;
    // open_cells
    
    auto item = std::make_tuple(curr->h, curr->loc);
    pqueue.push(item);
}

get_kmin_result Dstar::get_kmin() {
    get_kmin_result result;
    if (!pqueue.empty()) {
        auto item = pqueue.top();
        result.exists = true;
        result.kmin = std::get<0>(item);
        return result;
    }
    else {
        result.exists = false;
        result.kmin = -1;
        return result;
    }
}

double Dstar::get_cost(cell curr1, cell curr2) {
    // Unsure what structure of cost
    return costs[curr2.loc.x + curr2.loc.y * size.x];
}

std::vector<coordinate> Dstar::get_path(cell *curr) {
    std::vector<coordinate> path;
    path.push_back(curr->loc);
    
    cell * temp = curr;
    while (1) {
        temp = temp->b;
        
        if (temp == NULL) {
            return std::vector<coordinate>(0);
        }
        else {
            path.push_back(temp->loc);
            
            if ((temp->loc.x == goal.x) and (temp->loc.y == goal.y)) {
                return path;
            }
        }          
    }
}

std::vector<cell> Dstar::get_neighhors(cell curr) {
    std::vector<cell> neighbors;
    int x = curr.loc.x;
    int y = curr.loc.y;
    
    int min_y = std::max(0, y - 1);
    int max_y = std::min(size.y - 1, y + 1);
    int min_x = std::max(0, x - 1);
    int max_x = std::min(size.x - 1, x + 1);
    
    for (int j = min_y; j < max_y + 1; j++) {
        for (int i = min_x; i < max_x + 1; i++) {
            coordinate loc;
            loc.x = i;
            loc.y = j;
            neighbors.push_back(get(loc));
        }
    }
    return neighbors;
}


