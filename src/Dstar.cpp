#include <stdio.h>
#include "Dstar.hpp"
#include <algorithm>

std::vector<coordinate> Dstar::init_path() {
    cell goal_c = Cell(0, 0, 0 ,Tag::Open, goal);
    goal_c.b = goal;
    put(goal, goal_c);
    
    while (1) {
        double k_min = process_state();
        cell start_c = get(start);
        if (k_min == NONE) || (start.t == Tag::Closed) {
            return get_path(start_c)
        }
    }
}

Dstar::Dstar() {
    world = new cell[world_x * world_y]
}

Dstar::~Dstar() {
    delete world;
}

cell Dstar::get(coordinate n) {
    row_length = world_x;
    return world[n.x + n.y * row_length];
}

void Dstar::put(coordinate n, cell curr) {
    row_length = world_x;
    world[n.x + n.y * row_length] = curr;   
}

std::tuple<double, cell> Dstar::get_open() {
    if (!pqueue.empty()) {
        auto item = pqueue.top();
        pqueue.pop();
        coordinate location = std::get<1>(item);
        cell curr = get(location);
        curr.t = Tag::Closed;

        std::tuple<double, cell> val = std::make_tuple(std::get<0>(item), curr);
        return val;
        
    }
    else {
        return std::tuple<double, cell> val = std::make_tuple(NONE, NONE);
    }
}

void Dstar::put_open(cell curr) {
    curr.t = Tag::Open;
    // open_cells
    
    std::tuple<double, coordinate> item = std::make_tuple(cell.h, cell.loc);
    pqueue.push(item);
}

double Dstar::get_kmin() {
    if (!pqueue.empty()) {
        auto item = pqueue.top();
        return std::get<0>(item);
    }
    else {
        return NONE;
    }
}

double Dstar::get_cost(cell curr1, cell curr2) {
    // Unsure what structure of cost
    return cost;
}

std::vector<coordinate> Dstar::get_path(cell curr) {
    std::vector<coordinate> path;
    path.push_back(curr.loc);
    
    cell temp;
    while (1) {
        temp = curr.b;
        
        if (curr == NONE) {
            return NONE;
        }
        else {
            path.push_back(temp.loc);
            
            if (temp.loc.x == goal.x) and (temp.loc.y == goal.y) {
                return path;
            }
        }          
    }
}

std::vector<cell> Dstar::get_neighhors(cell curr) {
    std::vector<coordinate> neighbors;
    int x = curr.loc.x;
    int y = curr.loc.y;
    
    int min_y = std::max(0, y - 1);
    int max_y = std::min(world_y - 1, y + 1);
    int min_x = std::max(0, x - 1);
    int max_x = std::min(world_x - 1, x + 1);
    
    for (int j = min_y; j < max_y + 1; j++) {
        for (int i = min_x, i < max_x + 1) {
            coordinate loc;
            loc.x = i;
            loc.y = j;
            neighbors.push_back(get(loc));
        }
    }
    return neighbors;
}


