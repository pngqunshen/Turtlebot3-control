#include "trajectory.hpp"

std::vector<Position> post_process(std::vector<Position> path, Grid &grid) // returns the turning points
{
    // (1) obtain turning points
    if (path.size() <= 2)
    { // path contains 0 to elements. Nothing to process
        return path;
    }

    // add path[0] (goal) to turning_points
    std::vector<Position> turning_points = {path.front()}; 
    // add intermediate turning points
    for (int n = 2; n < path.size(); ++n)
    {
        Position &pos_next = path[n];
        Position &pos_cur = path[n - 1];
        Position &pos_prev = path[n - 2];

        double Dx_next = pos_next.x - pos_cur.x;
        double Dy_next = pos_next.y - pos_cur.y;
        double Dx_prev = pos_cur.x - pos_prev.x;
        double Dy_prev = pos_cur.y - pos_prev.y;

        // use 2D cross product to check if there is a turn around pos_cur
        if (abs(Dx_next * Dy_prev - Dy_next * Dx_prev) > 1e-5)
        {   // cross product is small enough ==> straight
            turning_points.push_back(pos_cur);
        }
    }
    // add path[path.size()-1] (start) to turning_points
    turning_points.push_back(path.back());

    std::vector<Position> post_process_path;
    
    // (2) make it more any-angle
    // done by students
    
    post_process_path = turning_points; // remove this line if (2) is done
    return post_process_path;
}

std::vector<Position> get_velocities(std::vector<Position> turning_points, Position curr_vel) /// get velocities at turning points
{
    std::vector<Position> velocities = {curr_vel};
    int length = turning_points.size();

    for (int i = 1; i < length - 1; i++){
        double dx = turning_points[i+1].x - turning_points[i-1].x;
        double dy = turning_points[i+1].y - turning_points[i-1].y;
        
        double  theta = atan2(dy, dx);
        Position velocity = {0.22 * cos(theta), 0.22 * sin(theta)};
        velocities.push_back(velocity);
    }
    velocities.push_back(Position(0,0));
    return velocities;
}

std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, Position vel_begin, Position vel_end, double average_speed, double target_dt){
    double duration = dist_euc(pos_begin, pos_end) / average_speed;

    double xCoefficients[6] = {0,0,0,0,0,0};
    double Minv[6][6] = {
        { 1                 ,  0                ,  0                    ,  0                 ,  0                ,  0                    },
        { 0                 ,  1                ,  0                    ,  0                 ,  0                ,  0                    },
        { 0                 ,  0                ,  0.5                  ,  0                 ,  0                ,  0                    },
        {-10/pow(duration,3), -6/pow(duration,2), -3/(2*duration)       ,  10/pow(duration,3), -4/pow(duration,2),  1/(2*duration)       },
        { 15/pow(duration,4),  8/pow(duration,3),  3/(2*pow(duration,2)), -15/pow(duration,4),  7/pow(duration,3), -1/pow(duration,2)    },
        {-6/pow(duration,5) , -3/pow(duration,4), -1/(2*pow(duration,3)),  6/pow(duration,5) , -3/pow(duration,4),  1/(2*pow(duration,3))}
        };
    double x[6] = {pos_begin.x, vel_begin.x, 0, pos_end.x, vel_end.x, 0};

    double yCoefficients[6] = {0,0,0,0,0,0};
    double yMinv[6][6];
    double y[6] = {pos_begin.y, vel_begin.y, 0, pos_end.y, vel_end.y, 0};

    for(int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){
            xCoefficients[i] += (Minv[i][j] * x[j]);
            yCoefficients[i] += (Minv[i][j] * y[j]);
        }
    }
    std::vector<Position> trajectory = {pos_begin};

    for (int i = 0; i < duration/target_dt; i++){
        Position interpolated_target = {0,0};
        for (int j = 0; j < 6; j++){
            interpolated_target.x += xCoefficients[j] * pow(target_dt * i,j);
            interpolated_target.y += yCoefficients[j] * pow(target_dt * i,j);
        }
        trajectory.push_back(interpolated_target);
    }
    return trajectory;
}


std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, double average_speed, double target_dt, Grid & grid)
{
    // (1) estimate total duration
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double duration = sqrt(Dx*Dx + Dy*Dy) / average_speed;

    // (2) generate cubic / quintic trajectory
    // done by students

    // OR (2) generate targets for each target_dt
    std::vector<Position> trajectory = {pos_begin};
    for (double time = target_dt; time < duration; time += target_dt)
    {
        trajectory.emplace_back(
            pos_begin.x + Dx*time / duration,
            pos_begin.y + Dy*time / duration
        );
    }

    return trajectory; 
}

bool is_safe_trajectory(std::vector<Position> trajectory, Grid & grid)
{   // returns true if the entire path is accessible; false otherwise
    if (trajectory.size() == 0)
    {   // no path
        return false; 
    } 
    else if (trajectory.size() == 1)
    {   // goal == start
        return grid.get_cell(trajectory.front()); // depends on the only cell in the path
    }

    // if there are more than one turning points. Trajectory must be fine enough.
    for (int n=1; n<trajectory.size(); ++n)
    {
        if (!grid.get_cell(trajectory[n]))
            return false;
        /* // Use this if the trajectory points are not fine enough (distance > cell_size)
        Index idx_src = grid.pos2idx(trajectory[n-1]);
        Index idx_tgt = grid.pos2idx(trajectory[n]);
        
        grid.los.reset(idx_src, idx_tgt); // interpolate a straight line between points; can do away with los if points are fine enough.
        Index idx = idx_src;
        while (idx.i != idx_tgt.i || idx.j != idx_tgt.j)
        {
            if (!grid.get_cell(idx))
            {
                return false;
            }
            idx = grid.los.next();
        }
        */
    }
    return true;
}