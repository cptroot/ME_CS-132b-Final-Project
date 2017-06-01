
struct cell {
    float h;
    float k;
    float b;
    char t;
    
    cell(float a, float b, float c, char d) : h(a), k(b), b(c), t(d) {}    
};

class Dstar() {
    public:
        Dstar();
        ~Dstar();
        cell get();
        
    private:
    
};


