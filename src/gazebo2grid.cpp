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
    grid.info.width = 1000;
    grid.info.height = 1000;
    grid.info.resolution = 0.1;
    grid.info.origin.position.x = -50;
    grid.info.origin.position.y = -50;
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
    const char *sizeText = box->FirstChildElement("size")->GetText();
    std::istringstream sizeStream(sizeText); // split the string into a string stream
    sizeStream >> size_x >> size_y >> size_z;

    // Get the pose of the box
    double x, y, z, roll, pitch, yaw;
    const char *poseText = pose->GetText();
    std::istringstream poseStream(poseText); // split the string into a string stream
    poseStream >> x >> y >> z >> roll >> pitch >> yaw;
    // calculate box vertices
    double x1 = x + size_x / 2 * cos(yaw) + size_y / 2 * sin(yaw);
    double y1 = y + size_x / 2 * sin(yaw) + size_y / 2 * cos(yaw);
    double x2 = x - size_x / 2 * cos(yaw) + size_y / 2 * sin(yaw);
    double y2 = y - size_x / 2 * sin(yaw) + size_y / 2 * cos(yaw);
    double x3 = x - size_x / 2 * cos(yaw) - size_y / 2 * sin(yaw);
    double y3 = y - size_x / 2 * sin(yaw) - size_y / 2 * cos(yaw);
    double x4 = x + size_x / 2 * cos(yaw) - size_y / 2 * sin(yaw);
    double y4 = y + size_x / 2 * sin(yaw) - size_y / 2 * cos(yaw);

    // calculate occupied cells
    int x1_cell = (x1 - grid.info.origin.position.x) / grid.info.resolution;
    int y1_cell = (y1 - grid.info.origin.position.y) / grid.info.resolution;
    int x2_cell = (x2 - grid.info.origin.position.x) / grid.info.resolution;
    int y2_cell = (y2 - grid.info.origin.position.y) / grid.info.resolution;
    int x3_cell = (x3 - grid.info.origin.position.x) / grid.info.resolution;
    int y3_cell = (y3 - grid.info.origin.position.y) / grid.info.resolution;
    int x4_cell = (x4 - grid.info.origin.position.x) / grid.info.resolution;
    int y4_cell = (y4 - grid.info.origin.position.y) / grid.info.resolution;

    

    // fill the cells
    for (int i = std::min(x1_cell, std::min(x2_cell, std::min(x3_cell, x4_cell)));
        i < std::max(x1_cell, std::max(x2_cell, std::max(x3_cell, x4_cell))); i++)
    {
        for (int j = std::min(y1_cell, std::min(y2_cell, std::min(y3_cell, y4_cell)));
            j < std::max(y1_cell, std::max(y2_cell, std::max(y3_cell, y4_cell))); j++)
        {
            if (i >= 0 && i < grid.info.width && j >= 0 && j < grid.info.height)
            {
                if(j * grid.info.width + i < grid.data.size()){
                    grid.data[j * grid.info.width + i] = 100;
                }
                grid.data[j * grid.info.width + i] = 100;
            }
        }
    }
}
#if 1
void Gazebo2Grid::load_world(std::string file_path)
{
    //Load the world
    tinyxml2::XMLDocument doc;
    doc.LoadFile(file_path.c_str());
    tinyxml2::XMLElement *sdf = doc.FirstChildElement("sdf");
    tinyxml2::XMLElement *world = sdf->FirstChildElement("world");
    tinyxml2::XMLElement *model = world->FirstChildElement("model");

    while(model != NULL){
        const char* model_name = model->Attribute("name");
        // std::cout << strcmp(model_name, "ground_plane") << std::endl;
        
        tinyxml2::XMLElement *link = model->FirstChildElement("link");
        while(link !=NULL){
            const char* link_name = link->Attribute("name");
            // std::cout << "link_name: " << link_name << std::endl;
            if(strstr(link_name, "Wall") != NULL){
                tinyxml2::XMLElement *visual = link->FirstChildElement("visual");
                while(visual != NULL){
                    tinyxml2::XMLElement *geometry = visual->FirstChildElement("geometry");
                    tinyxml2::XMLElement *pose = link->FirstChildElement("pose");
                    tinyxml2::XMLElement *box = geometry->FirstChildElement("box");
                    if(box != NULL){
                        add_object_to_grid(box, pose);
                    }
                    visual = visual->NextSiblingElement("visual");
                }
            }
            link = link->NextSiblingElement("link");
        }


        model = model->NextSiblingElement("model");
    }
    grid.header.frame_id = "map";
    grid.header.stamp = ros::Time::now();
    pub_grid.publish(grid);
}
#endif

void Gazebo2Grid::run()
{
    // Publish the grid
    load_world(file_path);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo2grid");
    Gazebo2Grid gazebo2grid;
    while (ros::ok())
    {
        gazebo2grid.run();
        ros::spinOnce();
    }
    return 0;
}