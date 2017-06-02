#include <stdio.h>
#include "Dstar.hpp"

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
        location = std::get<1>(item);
        cell curr = get(location);
        curr.t = Tag::Closed;

        std::tuple<double, cell> val = std::make_tuple(std::get<0>(item), curr);
        return val;
        
    }
    else {
        return std::tuple<double, cell> val = std::make_tuple(None, None);
    }
}

void put_open(cell curr) {
    curr.t = Tag::Open;
    // open_cells
    
}
