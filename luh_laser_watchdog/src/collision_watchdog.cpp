#include "luh_laser_watchdog/collision_watchdog.h"
#include <sensor_msgs/image_encodings.h>

using namespace luh_laser_watchdog;

//########## CONSTRUCTOR ###############################################################################################
CollisionWatchdog::CollisionWatchdog(ros::NodeHandle &node, std::vector<Laser> &lasers):
    lasers_(lasers), image_transport_(node)
{
    // === PARAMETERS ===
    FootprintReader fp_reader;
    footprint_ = fp_reader.readFootprint(node);
    if(footprint_.empty())
        ROS_ERROR("Could not find footprint on parameter server");

    critical_time_ = 2;
    sample_step_ = 0.1;
    progression_factor_ = 1.1;
    show_visualisation_ = true;

    if(!(ros::param::get("laser_watchdog/critical_time", critical_time_) &&
         ros::param::get("laser_watchdog/sample_step", sample_step_) &&
         ros::param::get("laser_watchdog/progression_factor", progression_factor_)&&
         ros::param::get("laser_watchdog/publish_visualisation", show_visualisation_)&&
         ros::param::get("laser_watchdog/safety_margin", safety_margin_)))
    {
        ROS_ERROR("Not all parameters could be loaded.");
    }

    // === SUBSCRIBER ===
    cmd_vel_subscriber_ = node.subscribe("cmd_vel", 10, &CollisionWatchdog::velocityCallback, this);

    // === PUBLISHER ===
    cmd_vel_publisher_ = node.advertise<geometry_msgs::Twist>("cmd_vel_safe", 10);
    image_publisher_ = image_transport_.advertise("laser_watchdog/image", 1);

    // === INITIALISATION ===
    if(show_visualisation_)
    {
        visualisation_ = cv::Mat::zeros(600, 600, CV_8UC3);

        // get footprint bounding box
        double x_max = -1e8;
        double x_min = 1e8;
        double y_max = -1e8;
        double y_min = 1e8;

        for(uint i=0; i<footprint_.size(); i++)
        {
            x_max = std::max(x_max, footprint_[i].x);
            x_min = std::min(x_min, footprint_[i].x);
            y_max = std::max(y_max, footprint_[i].y);
            y_min = std::min(y_min, footprint_[i].y);
        }

        double width = std::max(x_max - x_min, y_max - y_min);
        pixels_per_meter_ = 0.2 * std::max(visualisation_.rows, visualisation_.cols) / width;
    }
}


//########## CALLBACK: CMD VEL #########################################################################################
void CollisionWatchdog::velocityCallback(const geometry_msgs::Twist::ConstPtr &vel)
{
    if(show_visualisation_)
        visualisation_.setTo(cv::Scalar(0,0,0));

    double collision_time = getTimeToCollision(vel);

    geometry_msgs::Twist safe_vel = *vel;

    if(collision_time >= 0)
    {
        double factor = collision_time / critical_time_;

        safe_vel.linear.x *= factor;
        safe_vel.linear.y *= factor;
        safe_vel.angular.z *= factor;
    }

    cmd_vel_publisher_.publish(safe_vel);

    if(show_visualisation_)
    {
        drawScanPoints();
        drawMessage(collision_time);
//        cv::imshow("Watchdog", visualisation_);
//        cv::waitKey(1);
        publishImage();
    }
}

//########## PREDICT FOOTPRINT #########################################################################################
Footprint CollisionWatchdog::predictFootprint(const geometry_msgs::Twist::ConstPtr &vel, double seconds)
{
    if(seconds == 0)
        return footprint_;

    // predict position
    double dx, dy, dt;

    if(fabs(vel->angular.z) < 1e-6)
    {
        dt = 0;
        dx = vel->linear.x * seconds;
        dy = vel->linear.y * seconds;
    }
    else
    {
        dt = vel->angular.z * seconds;
        dx = (vel->linear.x * sin(dt) - vel->linear.y * (1 - cos(dt))) / vel->angular.z;
        dy = (vel->linear.y * sin(dt) + vel->linear.x * (1 - cos(dt))) / vel->angular.z;
    }

    // transform footprint
    Footprint new_fp;
    new_fp.resize(footprint_.size());
    double sdt = sin(dt);
    double cdt = cos(dt);

    for(uint i=0; i<footprint_.size(); i++)
    {
        // rotate
        new_fp[i].x = footprint_[i].x * cdt - footprint_[i].y * sdt;
        new_fp[i].y = footprint_[i].y * cdt + footprint_[i].x * sdt;

        // translate
        new_fp[i].x += dx;
        new_fp[i].y += dy;
    }

    return new_fp;
}

//########## HAS COLLISION #############################################################################################
bool CollisionWatchdog::hasCollision(Footprint &footprint)
{
    // === GET BOUNDING BOX ===
    double x_max = -1e8;
    double x_min = 1e8;
    double y_max = -1e8;
    double y_min = 1e8;

    for(uint i=0; i<footprint.size(); i++)
    {
        x_max = std::max(x_max, footprint[i].x);
        x_min = std::min(x_min, footprint[i].x);
        y_max = std::max(y_max, footprint[i].y);
        y_min = std::min(y_min, footprint[i].y);
    }

    // === CHECK FOR COLLISIONS ===
    for(uint i=0; i<lasers_.size(); i++)
    {
        std::vector<Point2D>& scan_points = lasers_[i].getScanPoints();

        for(uint j=0; j<scan_points.size(); j++)
        {
            // check if point is outside bounding box
            if(scan_points[j].x > x_max ||
                    scan_points[j].x < x_min ||
                    scan_points[j].y > y_max ||
                    scan_points[j].y < y_min)
                continue;

            // check if point is within footprint
            if(pointInPolygon(scan_points[j], footprint))
                return true;
        }
    }
    return false;
}

//########## POINT IN POLYGON TEST #####################################################################################
bool CollisionWatchdog::pointInPolygon(Point2D point, Footprint& footprint)
{
    int i, j, c = 0;
    int nvert = footprint.size();

    for (i = 0, j = nvert-1; i < nvert; j = i++)
    {
        if ( ((footprint[i].y>point.y) != (footprint[j].y>point.y)) &&
             (point.x < (footprint[j].x-footprint[i].x) * (point.y-footprint[i].y)
              / (footprint[j].y-footprint[i].y) + footprint[i].x) )
            c = !c;
    }
    return c;
}

//########## GET TIME TO COLLISION #####################################################################################
double CollisionWatchdog::getTimeToCollision(const geometry_msgs::Twist::ConstPtr &vel)
{
    double time = 0;
    double time_step = sample_step_;
    int num_steps = 0;

    if(show_visualisation_)
        drawFootprint(footprint_, cv::Scalar(42, 125, 235), 2);

    while(time < critical_time_)
    {
        time += time_step;
        time_step *= progression_factor_;
        num_steps++;

        Footprint fp = predictFootprint(vel, time);

        if(hasCollision(fp))
        {
            if(show_visualisation_)
                drawFootprint(fp, cv::Scalar(0, 0, 255), 2);

            if(num_steps <= safety_margin_)
                return 0;
            else
                return time;
        }
        else if(show_visualisation_)
        {
            drawFootprint(fp, cv::Scalar(100, 100, 100));
        }
    }

    // no collision
    return -1;
}

//########## DRAW SCANPOINTS ###########################################################################################
void CollisionWatchdog::drawScanPoints()
{
    cv::Point p;

    for(uint i=0; i<lasers_.size(); i++)
    {
        std::vector<Point2D>& scan_points = lasers_[i].getScanPoints();
        for(uint j=0; j<scan_points.size(); j++)
        {
            p.x = -scan_points[j].y * pixels_per_meter_;
            p.y = -scan_points[j].x * pixels_per_meter_;

            p.x += visualisation_.cols/2;
            p.y += visualisation_.rows/2;

            cv::circle(visualisation_, p, 1, cv::Scalar(200, 200, 200), -1);
        }
    }
}

//########## DRAW FOOTPRINT ############################################################################################
void CollisionWatchdog::drawFootprint(Footprint &footprint, cv::Scalar color, int thickness)
{
    std::vector<cv::Point> points;
    points.resize(footprint.size());
    for(uint i=0; i<footprint.size(); i++)
    {
        points[i].x = -footprint[i].y * pixels_per_meter_ + visualisation_.cols/2;
        points[i].y = -footprint[i].x * pixels_per_meter_ + visualisation_.rows/2;
    }

    cv::polylines(visualisation_, points, true, color, thickness);
}

//########## DRAW MESSAGE ##############################################################################################
void CollisionWatchdog::drawMessage(double time)
{
    std::stringstream ss;
    if(time >= 0)
        ss << "Collision in " << std::setprecision(2) << time << " seconds.";
    else
        ss << "No collision in " << critical_time_ << " seconds.";

    cv::putText(visualisation_, ss.str(), cv::Point(0, visualisation_.rows-5),
                cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255,255,255));
}

//########## PUBLISH IMAGE #############################################################################################
void CollisionWatchdog::publishImage()
{
    // === CREATE IMAGE MESSAGE ===
    sensor_msgs::Image image;
    image.width = visualisation_.cols;
    image.height = visualisation_.rows;
    image.step = visualisation_.cols * 3;
    image.data.assign(visualisation_.data,
                      visualisation_.data + visualisation_.cols*visualisation_.rows*visualisation_.channels());
    image.encoding = sensor_msgs::image_encodings::BGR8;

    // === PUBLISH ===
    image_publisher_.publish(image);
}
