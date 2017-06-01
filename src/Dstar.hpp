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
    double b;
    Tag t = Tag::New;
    coordinate loc;
    cell(double a, double b, double c, Tag d, coordinate e) : h(a), k(b), b(c), t(d), loc(e) {}    
};

class Dstar {
    public:
        Dstar();
        ~Dstar();
        cell get(coordinate n);
        cell put(coordinate n, cell item);
        // auto get_open();  
        void put_open(cell);
        // get_kmin():
        
       double get_cost(cell curr1, cell curr2);
       std::vector<coordinate> get_path(cell curr);
       std::vector<coordinate> get_neighhors(cell curr);
       
       /*************************************/
       
       std::vector<coordinate> init_path();
       void change_map(octomap::AbstractOcTree*);
       std::vector<coordinate> navigate_map(coordinate curr);
       // depends on get_kmin // modify_costs(cell curr1, cell curr2, float new_cost);
       void insert(cell curr, double new_h);
       // depends on get_kmin // process_state();
        
    private:
        int world_x;
        int world_y;
        cell* world;
        int start;
        int goal;
        int size;
        std::priority_queue<cell> pqueue;
};

