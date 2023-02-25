#include "ros/ros.h"
#include "grid.hpp"
#include "common.hpp"
#include <vector>
#include <queue>

#ifndef PLANNER_HPP
#define PLANNER_HPP
class Planner 
{       
    public:
        struct Node
        {
            double g, h;
            bool visited;
            Index idx, parent;
            Node();
        };
        struct Open
        {
            double f;
            Index idx;
            Open();
            Open(double f, Index idx);
        };
        struct CompareOpen {
            bool operator()(Open const& o1, Open const& o2)
            {
                return o1.f > o2.f;
            }
        };
        Index start, goal;
        Grid & grid; // REFERENCE <-- you cannot put the Planner class into containers (vectors , arrays etc.) 
        
        Planner(Grid & grid);
        std::vector<Index> get(Index idx_start, Index idx_goal);
        std::vector<Position> get(Position pos_start, Position pos_goal);

    private:
        std::vector<Node> nodes; // keeps a record of the cheapest cost of every cell in the grid, as well as their parents
        std::priority_queue<Open, std::vector<Open>, CompareOpen> open_list;
        Index NB_LUT[8] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};

        void add_to_open(Node * node);
        Node * poll_from_open();
};
#endif