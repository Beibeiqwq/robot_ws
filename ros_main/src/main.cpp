#include <ros/ros.h>
#include <main/main.hpp>

using namespace Main;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Main_node");
    MainNode main_node;
    main_node.init();
    ros::spin();
    return 0; 
}