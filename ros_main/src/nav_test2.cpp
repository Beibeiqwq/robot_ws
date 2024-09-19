// 航点导航初步实现
// 功能：订阅test1中的话题数据 "/robot_state/is_entrance"
// 1. 判断门是否打开
// 2. 门打开后自主导航至指定航点“cmd”
// 3. 请使用ROS_INFO来提供目前所处的状态 例如：等待开门 // 导航完成
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetWaypointByName.h>
using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "navtest");//节点名称
    ros::NodeHandle n;
    ros::ServiceClient cliGetWPName = n.serviceClient< waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    waterplus_map_tools::GetWaypointByName srvName;//初始化查询航点服务
    srvName.request.name = "cmd";
    if (cliGetWPName.call(srvName)) // 查询航点坐标信息
    {
        std::string name = srvName.response.name;   // 航点名称
        float x = srvName.response.pose.position.x; // x坐标
        float y = srvName.response.pose.position.y; // y坐标
        ROS_INFO("get!");
    }
    else
    {
        ROS_ERROR("Failed to call service GetWaypointByName");
    }
    MoveBaseClient ac("move_base", true); // 调用导航服务
    while (!ac.waitForServer(ros::Duration(5.0))) // 等待服务端启动
    {
        ROS_INFO("The move_base action server is no running. action abort...");
    }
        move_base_msgs::MoveBaseGoal goal;                // 目标信息
        goal.target_pose.header.frame_id = "map";         // 基于map坐标系
        goal.target_pose.header.stamp = ros::Time::now(); // 时间戳
        goal.target_pose.pose = srvName.response.pose;    // 坐标信息以及朝向信息
        ac.sendGoal(goal);                                // 提交到客户端
        ac.waitForResult();                               // 等待客户端回应
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)//如果导航成功
        {
            ROS_INFO("ok");
        }
}
