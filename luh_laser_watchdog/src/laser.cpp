#include "luh_laser_watchdog/laser.h"
#include <deque>

using namespace luh_laser_watchdog;

//########## LASER CONSTRUCTOR #########################################################################################
Laser::Laser()
{

}

//########## INIT ######################################################################################################
void Laser::init(ros::NodeHandle &node, std::string topic)
{
    // === INITIALISATION ===
    node_ = &node;
    got_new_scan_ = false;

    // === PARAMETERS ===
    node_->param("laser_watchdog/target_frame", target_frame_, std::string("base_link"));
    node_->param("laser_watchdog/footprint_width_x", footprint_width_x_, 0.7);
    node_->param("laser_watchdog/footprint_width_y", footprint_width_y_, 0.5);
    node_->param("laser_watchdog/num_points_for_mean", num_points_for_mean_, 10);

    // === SUBSCRIBERS ===
    laser_subscriber_ = node_->subscribe(topic, 10, &Laser::laserCallback, this);
}

//########## CALLBACK: LASER ###########################################################################################
void Laser::laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if(scan->ranges.empty())
        return;

    // if it is the first received scan: initialise transform
    if(frame_id_.empty())
    {
        tf::TransformListener tf_listener;

        if(!tf_listener.waitForTransform(target_frame_, scan->header.frame_id, ros::Time(0), ros::Duration(1.0)))
        {
            ROS_WARN("Missing transform from %s to %s.", scan->header.frame_id.c_str(), target_frame_.c_str());
            return;
        }

        frame_id_ = scan->header.frame_id;
        tf_listener.lookupTransform(target_frame_, frame_id_, ros::Time(0), transform_);

        ROS_INFO("Translation: [%f; %f; %f]", transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z());

        // calculate sin and cos for all angles
        for(uint i=0; i<scan->ranges.size(); i++)
        {
            double phi = scan->angle_min + i * scan->angle_increment;

            cos_phi_.push_back(cos(phi));
            sin_phi_.push_back(sin(phi));
        }

        ROS_INFO("Initialised scanner at '%s'", frame_id_.c_str());
    }

    // get cartesian scan points
    scan_points_.resize(scan->ranges.size());
    tf::Vector3 p1, p2;
    p1.setZ(0);
    int j = 0;

    for(uint i=0; i<scan->ranges.size(); i++)
    {
        double r = scan->ranges[i];

        if(r < last_scan_.range_min || r > last_scan_.range_max)
            continue;

        p1.setX(r * cos_phi_[i]);
        p1.setY(r * sin_phi_[i]);

        p2 = transform_(p1);

        scan_points_[j].x = p2.x();
        scan_points_[j].y = p2.y();
        j++;
    }
    scan_points_.resize(j);

    // save scan
    got_new_scan_ = true;
    last_scan_ = *scan;
}

//########## GOT NEW SCAN ##############################################################################################
bool Laser::gotNewScan()
{
    return got_new_scan_;
}

double Laser::getMin(std::vector<double> &v)
{
    std::sort_heap(v.begin(), v.end());

    // get min elements
    int num_elements = std::min(num_points_for_mean_, (int)v.size());
    double mean = 0;
    for(int i=0; i<num_elements; i++)
    {
        mean += v[i];
    }

    return mean / num_elements;
}

//########## GET DISTANCES #############################################################################################
Distances Laser::getDistances()
{
    Distances dist;
    dist.front = last_scan_.range_max;
    dist.back = last_scan_.range_max;
    dist.left = last_scan_.range_max;
    dist.right = last_scan_.range_max;


    double fx = footprint_width_x_ / 2;
    double fy = footprint_width_y_ / 2;

    std::vector<double> front_heap;
    std::vector<double> back_heap;
    std::vector<double> left_heap;
    std::vector<double> right_heap;


    for(uint i=0; i<scan_points_.size(); i++)
    {
        float x = scan_points_[i].x;
        float y = scan_points_[i].y;

        if(fabs(x) > fx && fabs(y) > fy)
            continue;

        if(fabs(x) <= fx)
        {
            if(y > 0)
            {
                left_heap.push_back(y);
                std::push_heap(left_heap.begin(), left_heap.end());
            }
            else
            {
                right_heap.push_back(-y);
                std::push_heap(right_heap.begin(), right_heap.end());
            }
        }

        if(fabs(y) <= fy)
        {
            if(x > 0)
            {
                front_heap.push_back(x);
                std::push_heap(front_heap.begin(), front_heap.end());
            }
            else
            {
                back_heap.push_back(-x);
                std::push_heap(back_heap.begin(), back_heap.end());
            }
        }

        dist.front = getMin(front_heap);
        dist.back = getMin(back_heap);
        dist.left = getMin(left_heap);
        dist.right = getMin(right_heap);

//        if(fabs(x) <= fx)
//        {
//            if(y > 0)
//                dist.left = std::min(dist.left, y);
//            else
//                dist.right = std::min(dist.right, -y);
//        }

//        if(fabs(y) <= fy)
//        {
//            if(x > 0)
//                dist.front = std::min(dist.front, x);
//            else
//                dist.back = std::min(dist.back, -x);
//        }
    }

    got_new_scan_ = false;

    dist.front -= fx;
    dist.back  -= fx;
    dist.left  -= fy;
    dist.right -= fy;

    return dist;
}
