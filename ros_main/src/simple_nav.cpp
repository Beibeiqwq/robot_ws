//航点导航初步实现
//功能：订阅test1中的话题数据 "/robot_state/is_entrance"
//1. 判断门是否打开
//2. 门打开后自主导航至指定航点“cmd”
//3. 请使用ROS_INFO来提供目前所处的状态 例如：等待开门 // 导航完成
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include<string.h>
using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static waterplus_map_tools::GetWaypointByName srvName; //
static int nOpenCount = 0;

//开门状态的回调函数
void Entrance_Detect(const std_msgs::String::ConstPtr & msg)
{   
   //ROS_WARN("[Entrance_Detect] - %s",msg->data.c_str());
    string strDoor = msg->data;//拿到字符串
    ROS_INFO("1111111111111222222222222222");
    if(strDoor== "door open")
    {
        nOpenCount++;
        ROS_INFO("op++++++++");
    }
    else
    {
        nOpenCount =0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_nav");
    ros::NodeHandle n;
    ros::Subscriber sub_ent = n.subscribe("/robot_state/is_entrance", 10, Entrance_Detect);
    ros::ServiceClient cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    ros::Rate r(10);
    while(ros::ok())
    {
            //等待开门,一旦检测到开门,便去往发令地点
            if(nOpenCount > 20)
            {
                string strGoto = "cmd";     //cmd是发令地点名称,请在地图里设置这个航点
                srvName.request.name = strGoto;
                if (cliGetWPName.call(srvName))
                {
                    std::string name = srvName.response.name;//将得到的航点名字存下来
                    float x = srvName.response.pose.position.x;//航点x坐标
                    float y = srvName.response.pose.position.y;//航点y坐标
                    ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)", strGoto.c_str(),x,y);

                    MoveBaseClient ac("move_base", true);//初始化移动客户端
                    if(!ac.waitForServer(ros::Duration(5.0)))//等待服务端回应
                    {
                        ROS_INFO("The move_base action server is no running. action abort...");
                    }
                    else
                    {
                        move_base_msgs::MoveBaseGoal goal;//初始化目标对象
                        goal.target_pose.header.frame_id = "map";//基于map坐标系
                        goal.target_pose.header.stamp = ros::Time::now();//时间戳
                        goal.target_pose.pose = srvName.response.pose;//目标位置姿态
                        ac.sendGoal(goal);//发送给服务端
                        ac.waitForResult();//等待结果
                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        {
                            ROS_INFO("Arrived at %s!",strGoto.c_str());
                            ros::spinOnce();//成功则直接进入回调
                        }
                        else
                            ROS_INFO("Failed to get to %s ...",strGoto.c_str() );
                    }
                    
                }
                else
                {
                    ROS_ERROR("Failed to call service GetWaypointByName");
                }
            }
        ros::spinOnce();//进入回调
        r.sleep();
    }

    return 0;
}