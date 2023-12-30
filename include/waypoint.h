#ifndef WAYPOINT_H
#define WAYPOINT_H

// TODO:

struct Waypoint {
    std::string command = "";
    double param1;
    double param2;

    /// @brief Executes a command for the bot to follow during the waypoint
    /// @param robot Robot instance
    void execute_aux_command(Robot& robot);
};

class Path {
private:
    std::vector<Waypoint> cpp_vect;
public:
    // allowable error to reach goal:
    double allowable_error = 0.25;
    /// @brief Non-default constructor
    /// @param x x coord of first waypoint
    /// @param y y coord of first waypoint
    void initialize(double x, double y);
    /// @brief Non-default constructor
    /// @param a vector of all points to add to path
    void initialize(std::vector<Waypoint> a);
    /// @brief Checks if the given goal in queue has been reached.
    /// @param goal goal to check
    /// @param x x coord of bot
    /// @param y y coord of bot
    /// @return true (yes), false (no)
    bool goal_reached(Waypoint& goal, double x, double y);
    /// @brief Returns latest goal in queue
    /// @return Waypoint object of goal
    Waypoint get_latest();
    /// @brief Removes the latest goal, and sets the next one as the latest.
    void pop_latest();
    /// @brief Adds a goal to the queue in terms of a Waypoint object.
    /// @param goal goal to add.
    void add_queue(Waypoint goal);
    /// @brief Adds a goal to the queue in terms of x, y
    /// @param x x coord of goal
    /// @param y y coord of goal
    void add_queue(double x, double y);
    /// @brief Gives the size of the queue
    /// @return the size of the queue
    int size();
};

#endif // WAYPOINT_H