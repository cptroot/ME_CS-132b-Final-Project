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
        Dstar(coordinate size, coordinate start, coordinate goal);
        ~Dstar();
       
        std::vector<coordinate> init_path();
        void change_map(octomap::AbstractOcTree*);
        std::vector<coordinate> navigate_map(coordinate curr);
        double modify_costs(cell *curr1, cell *curr2, float new_cost);
        void insert(cell *curr, double new_h);
        get_kmin_result process_state();
        
    private:
        cell* world;
        double* costs;
        coordinate start;
        coordinate goal;
        coordinate size;
        std::priority_queue<pqueue_type, std::vector<pqueue_type>, Compare> pqueue;

        cell get(coordinate n);
        cell * get_ptr(coordinate n);
        void put(coordinate n, cell item);
        std::tuple<double, cell *> get_open();  
        void put_open(cell *);
        get_kmin_result get_kmin();
            
        double get_cost(cell curr1, cell curr2);
        std::vector<coordinate> get_path(cell *curr);
        std::vector<cell> get_neighhors(cell curr);
};

