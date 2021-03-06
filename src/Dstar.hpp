#include <queue>
#include <octomap/octomap.h>

struct coordinate {
    int x;
    int y;

    coordinate() {};
    coordinate(int a, int b) : x(a), y(b) {};
};

/*
Access cell values:
   h:      path cost
   k:      smallest value of h seen so far
   b:      back pointer
   t:      tag ('c' - closed; 'o' - open; 'n' - new)
*/

enum Tag {
    Closed,
    Open,
    New
};

struct cell {
    double h;
    double k;
    cell * b;
    Tag t = Tag::New;
    coordinate loc;
    cell() {};
    cell(double a, double b, cell * c, Tag d, coordinate e) : h(a), k(b), b(c), t(d), loc(e) {}    
};

struct get_kmin_result {
    bool exists;
    double kmin;
};

using pqueue_type = std::tuple<double, coordinate>;

class Compare {
    public:
        bool operator() (pqueue_type e1, pqueue_type e2) {
            return std::get<0>(e1) > std::get<0>(e2);
        }
};

class Dstar {
    public:
        Dstar(coordinate start, coordinate goal, double height);
        ~Dstar();
       
        std::vector<coordinate> init_path();
        void change_map(octomap::OcTree*);
        std::vector<coordinate> navigate_map(coordinate curr);
        void add_obstacle(coordinate loc, double new_cost);
        void insert(cell *curr, double new_h);
        get_kmin_result process_state();
        
    private:
        double height;
        int offset;

        cell* world;
        double* costs;
        double* occupancy;
        coordinate start;
        coordinate goal;
        coordinate size;
        std::vector<cell *> pqueue;

        cell get(coordinate n);
        cell * get_ptr(coordinate n);
        void put(coordinate n, cell item);
        cell * get_open();  
        void put_open(cell *);
        get_kmin_result get_kmin();
            
        double get_cost(cell curr1, cell curr2);
        double get_cost(coordinate loc1, coordinate loc2);
        std::vector<coordinate> get_path(cell *curr);
        std::vector<cell *> get_neighbors(coordinate loc);
};

