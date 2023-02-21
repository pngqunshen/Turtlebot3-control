#include "planner.hpp"
Planner::Node::Node() 
    : g(0), h(0), visited(false), idx(-1, -1), parent(-1, -1) 
    {}
Planner::Open::Open() 
    : f(0), idx(-1, -1) 
    {}
Planner::Open::Open(double f, Index idx) 
    : f(f), idx(idx) 
    {}
Planner::Planner(Grid & grid) // assumes the size of the grid is always the same
    : start(-1, -1), goal(-1, -1), grid(grid), nodes(grid.size.i * grid.size.j), open_list()
    {
        // write the nodes' indices
        int k = 0;
        for (int i = 0; i < grid.size.i; ++i)
        {
            for (int j = 0; j < grid.size.j; ++j)
            {
                nodes[k].idx.i = i;
                nodes[k].idx.j = j;
                ++k;
            }
        }
    }
    

void Planner::add_to_open(Node * node)
{   // sort node into the open list
    double node_f = node->g + node->h;
    Open new_open_node = Open(node_f, node->idx);
    open_list.push(new_open_node);
}
Planner::Node * Planner::poll_from_open()
{   
    Index idx = open_list.top().idx; //ref is faster than copy
    int k = grid.get_key(idx);
    Node * node = &(nodes[k]);

    open_list.pop();

    return node;
}
std::vector<Position> Planner::get(Position pos_start, Position pos_goal)
{
    std::vector<Index> path_idx = get(grid.pos2idx(pos_start), grid.pos2idx(pos_goal));
    std::vector<Position> path;
    for (Index & idx : path_idx)
    {
        path.push_back(grid.idx2pos(idx));
    }
    return path;
}
std::vector<Index> Planner::get(Index idx_start, Index idx_goal)
{
    std::vector<Index> path_idx; // clear previous path

    // initialise data for all nodes
    for (Node & node : nodes)
    {
        node.h = dist_oct(node.idx, idx_goal);
        node.g = 1e5; // a reasonably large number. You can use infinity in clims as well, but clims is not included
        node.visited = false;
    }

    // set start node g cost as zero
    int k = grid.get_key(idx_start);
    ROS_INFO("idx_start %d %d", idx_start.i, idx_start.j);
    ROS_INFO("idx_goal %d %d", idx_goal.i, idx_goal.j);
    Node * node = &(nodes[k]);
    node->g = 0;

    // append all accessible neighbors around start to open list
    for (int dir = 0; dir < 8; ++dir)
    {
        // get their index
        Index & idx_nb_relative = NB_LUT[dir];
        Index idx_nb(
            node->idx.i + idx_nb_relative.i,
            node->idx.j + idx_nb_relative.j
        );

        // check if in map and accessible
        if (!grid.get_cell(idx_nb))
        {   // if not, move to next nb
            continue;
        }

        // find costs
        double tg_cost = dist_euc(idx_nb, idx_start);
        double h_cost = dist_euc(idx_nb, idx_goal);
        double rf_cost = round_up(tg_cost+h_cost, 5);
    
        // assign parent
        int nb_k = grid.get_key(idx_nb);
        Node & nb_node = nodes[nb_k];
        nb_node.g = round_up(tg_cost, 5);
        nb_node.h = round_up(h_cost, 5);
        nb_node.parent = node->idx;

        // add to open
        add_to_open(&nb_node);
    }

    // main loop
    while (!open_list.empty())
    {
        // (1) poll node from open
        node = poll_from_open();

        // (2) check if node was visited, and mark it as visited
        if (node->visited)
        {   // if node was already visited ==> cheapest route already found, no point expanding this anymore
            continue; // go back to start of while loop, after checking if open list is empty
        }
        node->visited = true; // mark as visited, so the cheapest route to this node is found


        // (3) return path if node is the goal
        if (node->idx.i == idx_goal.i && node->idx.j == idx_goal.j)
        {   // reached the goal, return the path
            ROS_INFO("reach goal");

            path_idx.push_back(node->idx);

            while (node->idx.i != idx_start.i || node->idx.j != idx_start.j)
            {   // while node is not start, keep finding the parent nodes and add to open list
                k = grid.get_key(node->parent);
                node = &(nodes[k]); // node is now the parent

                path_idx.push_back(node->idx);
            }

            break;
        }

        // (4) check neighbors and add them if cheaper
        for (int dir = 0; dir < 8; ++dir)
        {   // for each neighbor in the 8 directions
            // get their index
            Index & idx_nb_relative = NB_LUT[dir];
            Index idx_nb(
                node->idx.i + idx_nb_relative.i,
                node->idx.j + idx_nb_relative.j
            );

            // check if in map and accessible
            if (!grid.get_cell(idx_nb))
            {   // if not, move to next nb
                continue;
            }

            // assume LOS and assign node parents first
            Index par = node->parent;

            // check if nb_cell has LOS to current cell’s parent
            for (Index los_ind : grid.los.get(par, idx_nb)) {
                // check if in map and accessible
                if (!grid.get_cell(los_ind))
                {   // no line of sight
                    par = node->idx;
                    break;
                }
            }

            // get tentative g cost
            double tg_cost = dist_euc(idx_nb, par);

            // compare the cost to any previous costs. If cheaper, mark the node as the parent
            int nb_k = grid.get_key(idx_nb);
            Node & nb_node = nodes[nb_k]; // use reference so changing nb_node changes nodes[k]
            if (round_up(nb_node.g, 5) > round_up(tg_cost, 5))
            {   // previous cost was more expensive, rewrite with current
                nb_node.g = round_up(tg_cost, 5);
                nb_node.h = round_up(dist_euc(idx_nb, idx_goal), 5);
                nb_node.parent = par;

                // add to open
                add_to_open(&nb_node); // & a reference means getting the pointer (address) to the reference's object.
            }
        }
    }

    // clear open list
    while (!open_list.empty())
    {
        open_list.pop();
    }
    return path_idx; // is empty if open list is empty
}

