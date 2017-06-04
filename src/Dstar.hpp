#include <queue>

struct coordinate {
    int x;
    int y;
    
    coordinate(int a, int b) : x(a), y(b) {}
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
    cell b;
    Tag t = Tag::New;
    coordinate loc;
    cell(double a, double b, double c, Tag d, coordinate e) : h(a), k(b), b(c), t(d), loc(e) {}    
};

class Dstar {
    public:
        Dstar();
        ~Dstar();
       
        std::vector<coordinate> init_path();
        void change_map(octomap::AbstractOcTree*);
        std::vector<coordinate> navigate_map(coordinate curr);
        double modify_costs(cell curr1, cell curr2, float new_cost);
        void insert(cell curr, double new_h);
        double process_state();
        
    private:
        int world_x;
        int world_y;
        cell* world;
        coordinate start;
        coordinate goal;
        int size;
        std::priority_queue<std::tuple<double, coordinate> pqueue;

        cell get(coordinate n);
        cell put(coordinate n, cell item);
        std::tuple<double, cell> get_open();  
        void put_open(cell);
        double get_kmin():
            
        double get_cost(cell curr1, cell curr2);
        std::vector<coordinate> get_path(cell curr);
        std::vector<cell> get_neighhors(cell curr);
};

