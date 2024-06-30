#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include "tinyxml2.h"

class Gazebo2Grid
{
    public:
        Gazebo2Grid();
        void run();
    private:
        void initialize_grid();
        // void model_states_callback(const gazebo_msgs::ModelStates::ConstPtr &msg);
        void load_world(std::string file_path);
        void add_object_to_grid(tinyxml2::XMLElement *box, tinyxml2::XMLElement *pose);
        ros::NodeHandle nh;
        ros::Publisher pub_grid;
        nav_msgs::OccupancyGrid grid;

        std::string world_file;
        std::string file_path;
};