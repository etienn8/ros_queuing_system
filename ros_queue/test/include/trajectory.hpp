#pragma once

#include <list>

using std::list;

class Position3D
{
    public:
        Position3D(int x, int y, int z): x_(x), y_(y), z_(z) {};

        int x_ = 0;
        int y_ = 0;
        int z_ = 0;

        friend bool operator==(const Position3D& traj1, const Position3D& traj2)
        {
            return (traj1.x_ == traj2.x_) && (traj1.y_ == traj2.y_) && (traj1.z_ == traj2.z_);
        }
};

class Trajectory
{
    public:
        Trajectory(string reference_frame, list<Position3D> point_list): 
                    reference_frame_(reference_frame), point_list_(point_list) {};

        list<Position3D> point_list_;
        string reference_frame_;

        friend bool operator==(const Trajectory& traj1, const Trajectory& traj2)
        {
            
            return (traj1.reference_frame_==traj2.reference_frame_) && (traj1.point_list_==traj2.point_list_);
        }

};