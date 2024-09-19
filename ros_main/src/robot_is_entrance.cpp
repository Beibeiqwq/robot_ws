//功能：检测门是否打开

//将状态发布到话题"/robot_state/is_entrance"
//节点名称 "robot_is_entrance"
//发布的数据类型std_msgs::String

//变量名不作要求 尽量可读性强

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

static ros::Publisher ent_pub;

static float ranges[360];//保存雷达数据

void Scan_Entrance(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(int i=0;i<360;i++)//雷达数据储存
    {
        ranges[i] = scan->ranges[i];
    }

    int nMidIndex = (scan->ranges.size())/2;//正前方 size=360 ->180
    bool bDoorOpen = true;//默认状态为开门
    for(int i=0;i<5;i++)
    {
        if(ranges[nMidIndex - i] < 1.0)//默认为1m
        {
            bDoorOpen = false;
        }
        if(ranges[nMidIndex + i] < 1.0)
        {
            bDoorOpen = false;
        }
    }

    std_msgs::String strEnt;
    if(bDoorOpen == true)
    {
        strEnt.data = "door open";
    }
    else
    {
        strEnt.data = "door close";
    }
    ent_pub.publish(strEnt);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_is_entrance");
    
    ros::NodeHandle n;

    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1,Scan_Entrance);//订阅雷达 传入回调函数
    
    ent_pub = n.advertise<std_msgs::String>("/robot_state/is_entrance", 10);//初始化发布者 定义发布到的话题

    ros::spin();

    return 0;
}