#ifndef WAYPOINT_H
#define WAYPOINT_H

// TODO:

struct Waypoint {
    int x;
    int y;
};

class Path {
private:
    std::vector<Waypoint> cpp_vect;
public:
    void initialize(double x, double y);

    Waypoint get_latest();
    void pop_latest();

    void add_queue(Waypoint goal);
    void add_queue(double x, double y);

    int size();
};

#endif // WAYPOINT_H