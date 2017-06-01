#include <stdio.h>
#include <queue>

struct cell {
    float h;
    float k;
    float b;
    char t;
    
    cell(float a, float b, float c, char d) : h(a), k(b), b(c), t(d) {}    
};

Dstar::Dstar() {
    int world_x;
    int world_y;
    
    int start;
    int goal;
    int size;
    std::priority_queue<cell> pqueue;
}

cell Dstar::get(n) {
    
}

void Dstar::put(n, cell) {
    
}

void Dstar::get_open() {
    if (!pqueue.empty()) {
        cell item = pqueue.top();
        pqueue.pop();
        cu
        
    }
    else {
        
    }
}