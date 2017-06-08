#include <stdio.h>
#include "Dstar.hpp"
#include <algorithm>

std::vector<coordinate> Dstar::init_path() {
    cell * goal_c = this->get_ptr(goal);
    *goal_c = cell(0, 0, goal_c, Tag::Open, goal);
    
    while (1) {
        get_kmin_result k_min = this->process_state();
        cell * start_c = this->get_ptr(start);
        if ((k_min.exists == false) || (start_c->t == Tag::Closed)) {
            return this->get_path(start_c);
        }
    }
}

void Dstar::change_map(octomap::AbstractOcTree*) {
    auto curr = this->get_ptr(coordinate(0, 0));
    auto neighbors = this->get_neighbors(curr->loc);
    for (auto neighbor_it = neighbors.begin(); neighbor_it < neighbors.end(); neighbor_it++) {
        auto neighbor = *neighbor_it;
        coordinate locate = neighbor->loc;
    }
}

//double Dstar::modify_costs(cell curr1, cell curr2, float new_cost);

std::vector<coordinate> Dstar::navigate_map(coordinate curr) {
    cell * curr_cell_ptr = this->get_ptr(curr);
    while(1) {
        get_kmin_result k_min = this->process_state();
        
        if (((k_min.exists == false) ||
                    (curr_cell_ptr->h <= k_min.kmin)) &&
                !(curr_cell_ptr->t == Tag::Open)) {
            return this->get_path(curr_cell_ptr);
        }
    }
}

void Dstar::insert(cell * curr, double new_h) {
    if (curr->t == Tag::New) {
        curr->h = new_h;
        curr->k = new_h;
        curr->t = Tag::Open;
        this->put_open(curr);
    }
    else if (curr->t = Tag::Open) {
        curr->h = new_h;
        curr->k = std::min(curr->k, new_h);
        // Do STUFF
    }
    else if (curr->t == Tag::Closed) {
        curr->h = new_h;
        curr->k = std::min(curr->h, new_h);
        curr->t = Tag::Open;
        this->put_open(curr);
    }
}

get_kmin_result Dstar::process_state() {

    auto tuple = this->get_open();
    double k_old = std::get<0>(tuple);
    cell * curr = std::get<1>(tuple);
    
    if (curr == NULL) {
        get_kmin_result result;
        result.exists = false;
        result.kmin = -1.0;
        return result;
    }

    if (k_old < curr->h) {
        auto neighbors = this->get_neighbors(curr->loc);
        for (auto neighbor_it = neighbors.begin(); neighbor_it < neighbors.end(); neighbor_it++) {
            auto neighbor = *neighbor_it;
            if (neighbor->t != 'n' && neighbor->h <= k_old \
            && curr->h > neighbor->h + this->get_cost(neighbor->loc, curr->loc)) {
                curr->b = neighbor;
                curr->h = neighbor->h + this->get_cost(neighbor->loc, curr->loc);
            }
        }
    }

    if (k_old == curr->h) {
        auto neighbors = this->get_neighbors(curr->loc);
        for (auto neighbor_it = neighbors.begin(); neighbor_it < neighbors.end(); neighbor_it++) {
            auto neighbor = *neighbor_it;
            if ((neighbor->t == 'n') ||
                    (neighbor->b == curr && neighbor->h != curr->h + this->get_cost(curr->loc, neighbor->loc)) ||
                    (neighbor->b != curr && neighbor->h > curr->h + this->get_cost(curr->loc, neighbor->loc))) {
                neighbor->b = curr;
                this->insert(neighbor, curr->h + this->get_cost(curr->loc, neighbor->loc));
            }
        }
    } else {
        auto neighbors = this->get_neighbors(curr->loc);
        for (auto neighbor_it = neighbors.begin(); neighbor_it < neighbors.end(); neighbor_it++) {
            auto neighbor = *neighbor_it;
            if (neighbor->t == 'n' ||
                    (neighbor->b == curr && neighbor->h != curr->h + this->get_cost(curr->loc, neighbor->loc))) {
                neighbor->b = curr;
                this->insert(neighbor, curr->h + this->get_cost(curr->loc, neighbor->loc));

            } else if (neighbor->b != curr && neighbor->h > curr->h + this->get_cost(curr->loc, neighbor->loc)) {
                this->insert(curr, curr->h);
            } else if (neighbor->b != curr && curr->h > neighbor->h + this->get_cost(curr->loc, neighbor->loc) &&
                    neighbor->t == 'c' && neighbor->h > k_old) {
                this->insert(neighbor, neighbor->h);
            }
        }
    }

    return this->get_kmin();
}

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
    *this->get_ptr(n) = curr;   
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

double Dstar::get_cost(coordinate loc1, coordinate loc2) {
    // Unsure what structure of cost
    return costs[loc2.x + loc2.y * size.x];
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
            
            if ((temp->loc.x == goal.x) && (temp->loc.y == goal.y)) {
                return path;
            }
        }          
    }
}

std::vector<cell *> Dstar::get_neighbors(coordinate loc) {
    std::vector<cell *> neighbors;
    int x = loc.x;
    int y = loc.y;
    
    int min_y = std::max(0, y - 1);
    int max_y = std::min(size.y - 1, y + 1);
    int min_x = std::max(0, x - 1);
    int max_x = std::min(size.x - 1, x + 1);
    
    for (int j = min_y; j < max_y + 1; j++) {
        for (int i = min_x; i < max_x + 1; i++) {
            if (i == x && j == y) { continue; }
            coordinate loc;
            loc.x = i;
            loc.y = j;
            neighbors.push_back(get_ptr(loc));
        }
    }
    return neighbors;
}


