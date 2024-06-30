#include "gazebo2grid/gazebo2grid.h"

Gazebo2Grid::Gazebo2Grid() : nh("~")
{
    // Initialize the grid
    initialize_grid();

    // Load the world file
    nh.param("world_file", world_file, std::string("ikuta_building_d_1f.world"));
    file_path = ros::package::getPath("amsl_gazebo_worlds") + "/worlds/" + world_file;
    std::cout << file_path << std::endl;

    pub_grid = nh.advertise<nav_msgs::OccupancyGrid>("/grid", 1);
}

void Gazebo2Grid::initialize_grid()
{
    // Set the grid size
    grid.info.width = 100;
    grid.info.height = 100;
    grid.info.resolution = 0.1;
    grid.info.origin.position.x = -5;
    grid.info.origin.position.y = -5;
    grid.info.origin.position.z = 0;
    grid.info.origin.orientation.x = 0;
    grid.info.origin.orientation.y = 0;
    grid.info.origin.orientation.z = 0;
    grid.info.origin.orientation.w = 1;

    // Initialize the grid with -1
    grid.data.resize(grid.info.width * grid.info.height, -1);
}
void Gazebo2Grid::add_object_to_grid(tinyxml2::XMLElement *box, tinyxml2::XMLElement *pose)
{
    // // Get the size of the box
    double size_x, size_y, size_z;
    const char* sizeText = box->FirstChildElement("size")->GetText();
    std::istringstream sizeStream(sizeText);    //split the string into a string stream
    sizeStream >> size_x >> size_y >> size_z;
    std::cout << "size_x: " << size_x << " size_y: " << size_y << " size_z: " << size_z << std::endl;
    
    // Get the pose of the box
    double x, y, z, roll, pitch, yaw;
    const char* poseText = pose->GetText();
    std::istringstream poseStream(poseText);    //split the string into a string stream
    poseStream >> x >> y >> z >> roll >> pitch >> yaw;

    // Convert the pose to the grid coordinates
    int x_min = (x - size_x / 2 - grid.info.origin.position.x) / grid.info.resolution;
    int x_max = (x + size_x / 2 - grid.info.origin.position.x) / grid.info.resolution;
    int y_min = (y - size_y / 2 - grid.info.origin.position.y) / grid.info.resolution;
    int y_max = (y + size_y / 2 - grid.info.origin.position.y) / grid.info.resolution;

    // Update the grid
    for (int i = x_min; i < x_max; i++)
    {
        for (int j = y_min; j < y_max; j++)
        {
            grid.data[i + j * grid.info.width] = 100;
        }
    }
}

void Gazebo2Grid::load_world(std::string file_path)
{
    // Load the world
    tinyxml2::XMLDocument doc;
    doc.LoadFile(file_path.c_str());
    tinyxml2::XMLElement *sdf = doc.FirstChildElement("sdf");
    tinyxml2::XMLElement *world = sdf->FirstChildElement("world");
    tinyxml2::XMLElement *model = world->FirstChildElement("model");

    while (model != NULL)
    {
        if(model->Attribute("name")=="ground_plane") continue;
        else{
            tinyxml2::XMLElement *link = model->FirstChildElement("link");
            while(link != NULL){
                tinyxml2::XMLElement *visual = link->FirstChildElement("visual");
                while(visual != NULL){
                    tinyxml2::XMLElement *geometry = visual->FirstChildElement("geometry");
                    tinyxml2::XMLElement *pose = visual->FirstChildElement("pose");
                    tinyxml2::XMLElement *box = geometry->FirstChildElement("box");
                    if(box != NULL){
                        add_object_to_grid(box, pose);
                    }
                    visual = visual->NextSiblingElement("visual");
                }
                link = link->NextSiblingElement("link");
            }

        }

        model = model->NextSiblingElement("model");
    }
    grid.header.frame_id = "map";
    grid.header.stamp = ros::Time::now();
    pub_grid.publish(grid);
}

void Gazebo2Grid::run()
{
    // Publish the grid
    load_world(file_path);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo2grid");
    Gazebo2Grid gazebo2grid;
    while(ros::ok())
    {
        gazebo2grid.run();
        ros::spinOnce();
    }
    return 0;
}