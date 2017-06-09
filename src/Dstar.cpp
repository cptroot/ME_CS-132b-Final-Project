#include <stdio.h>
#include "Dstar.hpp"
#include <algorithm>
#include <cmath>

#include <ros/ros.h>

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

void Dstar::change_map(octomap::OcTree * tree) {
    //auto center = octomap::point3d(0, 0, 0);
    //auto half_size = tree->getNodeSize(0) / 2;
    //auto min_bbx = octomap::point3d(center.x() - half_size, center.y() - half_size, height - 0.5);
    //auto max_bbx = octomap::point3d(center.x() + half_size, center.y() + half_size, height + 0.5);
    for (auto leaf_bbx_it = tree->begin_leafs(16);
            leaf_bbx_it != tree->end_leafs();
            leaf_bbx_it++) {
        // Find current bounds, iterate through affected cells
        auto leaf_center = leaf_bbx_it.getCoordinate();
        auto leaf_half_size = leaf_bbx_it.getSize() / 2;

        int min_x = tree->coordToKey(leaf_center.x() - leaf_half_size);
        int max_x = tree->coordToKey(leaf_center.x() + leaf_half_size);
        int min_y = tree->coordToKey(leaf_center.y() - leaf_half_size);
        int max_y = tree->coordToKey(leaf_center.y() + leaf_half_size);

        min_x = std::max(min_x - offset, 0);
        max_x = std::min(max_x - offset, size.x);
        min_y = std::max(min_y - offset, 0);
        max_y = std::min(max_y - offset, size.y);

        for (int j = min_y; j < max_y; j++) {
            for (int i = min_x; i < max_y; i++) {
                if (!this->occupancy[i + j * size.x] < leaf_bbx_it->getOccupancy()) {
                    // Update new cost
                    this->add_obstacle(coordinate(i, j), leaf_bbx_it->getOccupancy());
                }
            }
        }
    }
}

void Dstar::add_obstacle(coordinate loc, double new_probability) {
    ROS_INFO("Updated obstacle at (%d, %d): %f", loc.x, loc.y, new_probability * 1000);

    double diff = new_probability - this->occupancy[loc.x + loc.y * size.x];

    int min_x = std::max(0, loc.x - 5);
    int max_x = std::min(size.x, loc.x + 6);
    int min_y = std::max(0, loc.y - 5);
    int max_y = std::min(size.y, loc.y + 6);

    double max_distance = std::sqrt(5.0 * 5 * 2);

    for (int j = min_y; j < max_y; j++) {
        for (int i = min_x; i < max_x; i++) {
            double distance = std::sqrt(i * i + j * j);
            double multiplier = 1000.0 - distance * 1000 / max_distance;
            this->costs[i + j * size.x] += diff * multiplier;

            cell * curr = this->get_ptr(coordinate(i, j));
            if (curr->t == Tag::Closed) {
                this->insert(curr, curr->h);
            }
        }
    }
}

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
    } else if (curr->t = Tag::Open) {
        curr->h = new_h;
        curr->k = std::min(curr->k, new_h);

        std::make_heap(pqueue.begin(), pqueue.end());
    } else if (curr->t == Tag::Closed) {
        curr->h = new_h;
        curr->k = std::min(curr->h, new_h);
        curr->t = Tag::Open;
        this->put_open(curr);
    }
}

get_kmin_result Dstar::process_state() {
    auto curr = this->get_open();
    
    if (curr == NULL) {
        get_kmin_result result;
        result.exists = false;
        result.kmin = -1.0;
        return result;
    }

    if (curr->k < curr->h) {
        auto neighbors = this->get_neighbors(curr->loc);
        for (auto neighbor_it = neighbors.begin(); neighbor_it < neighbors.end(); neighbor_it++) {
            auto neighbor = *neighbor_it;
            if (neighbor->t != 'n' &&
                    neighbor->h <= curr->k &&
                    curr->h > neighbor->h + this->get_cost(neighbor->loc, curr->loc)) {
                curr->b = neighbor;
                curr->h = neighbor->h + this->get_cost(neighbor->loc, curr->loc);
            }
        }
    }

    if (curr->k == curr->h) {
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
                    neighbor->t == 'c' && neighbor->h > curr->k) {
                this->insert(neighbor, neighbor->h);
            }
        }
    }

    return this->get_kmin();
}

Dstar::Dstar(coordinate start, coordinate goal, double height) :
        start(start), goal(goal), height(height) {
    size = coordinate(65536/16, 65536/16);
    world = new cell[size.x * size.y];
    costs = new double[size.x * size.y];
    std::fill(costs, costs + size.x * size.y, 0.0);

    occupancy = new double[size.x * size.y];
    std::fill(occupancy, occupancy + size.x * size.y, 0.0);

    offset = (65536 - size.x) / 2;
}

Dstar::~Dstar() {
    delete world;
    delete costs;
    delete occupancy;
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

cell * Dstar::get_open() {
    if (!pqueue.empty()) {
        auto curr = pqueue.front();
        std::pop_heap(pqueue.begin(), pqueue.end());
        pqueue.pop_back();
        curr->t = Tag::Closed;
        return curr;
    } else {
        return NULL;
    }
}

void Dstar::put_open(cell * curr) {
    curr->t = Tag::Open;
    // open_cells
    
    pqueue.push_back(curr);
    std::push_heap(pqueue.begin(), pqueue.end());
}

get_kmin_result Dstar::get_kmin() {
    get_kmin_result result;
    if (!pqueue.empty()) {
        auto curr = pqueue.front();
        result.exists = true;
        result.kmin = curr->k;
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
    int diff_x = loc1.x - loc2.x;
    int diff_y = loc1.y - loc2.y;
    double distance = std::sqrt((double)(diff_x * diff_x + diff_y * diff_y));
    return costs[loc2.x + loc2.y * size.x] + distance;
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


