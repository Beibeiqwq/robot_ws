#ifndef MAIN_H
#define MAIN_H
/*---------------头文件定义区---------------*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "xfyun_waterplus/IATSwitch.h"
#include "wpb_home_tutorials/Follow.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <wpb_yolo5/BBox2D.h>
#include <wpb_yolo5/BBox3D.h>
#include <sound_play/SoundRequest.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "xfyun_waterplus/IATSwitch.h"
#include <waterplus_map_tools/GetWaypointByName.h>
#include <robot_voice/StringToVoice.h>
/*---------------状态机区---------------*/
#define STATE_READY               0
#define STATE_WAIT_ENTR           1
#define STATE_WAIT_CMD            2
#define STATE_ACTION              3
#define STATE_GOTO_EXIT           4
#define ACT_CLEAR                 101
#define ACT_FIND_PERSON           102
#define ACT_FIND_OBJECT           103
#define ACT_GRAB_OBJECT           104
/*---------------别名定义区---------------*/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

/// @brief YOLOV5 BoundingBox2D 格式
typedef struct BBox2D
{
    std::string name;
    int left;
    int right;
    int top;
    int bottom;
    float probability;
} BBox2D;

using namespace std;
namespace Main
{
    class MainNode
    {
    public:
        MainNode() {};
        ~MainNode() {};
        void init()
        {
            ros::NodeHandle n("~");
            Init_keywords();
            /*---------------参数导入区---------------*/
            n.param<string>("name",name_yaml,"default");
            n.param<string>("enter",coord_cmd,"cmdA");
            n.param<string>("place1",arKWPlacement[1],"living room");
            n.param<string>("place2",arKWPlacement[2],"kitchen");
            n.param<string>("place3",arKWPlacement[3],"bedroom");
            n.param<string>("place4",arKWPlacement[4],"dining room");
            n.param<string>("exit",coord_exit,"exitA");
            /*---------------ROS初始化---------------*/
            sub_yolo = n.subscribe("/yolo_bbox_2d", 10, &MainNode::KeywordCB,this);
            sub_ent  = n.subscribe("/wpb_home/entrance_detect", 10, &MainNode::EntranceCB,this);
            client_speak = n.serviceClient<robot_voice::StringToVoice>("str2voice");
            cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
            spk_pub  = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
            vel_pub  = n.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
            yolo_pub = n.advertise<std_msgs::String>("/yolov5/cmd", 20);
            /*---------------主程序区域---------------*/
            cout << "[Main]请检查程序参数...."<< endl;
            Parameter_Check();
            cout << "[Main]键入任意数字开始.... 按CTRL+Z退出" << endl;
            cin >> check_flag;
            nState = STATE_WAIT_ENTR;
            cout << "[Main]State: STATE_WAIT_ENTR" << endl;
            while (ros::ok())
            {
                if (nState == STATE_WAIT_ENTR)
                {
                    if (nOpenCount > 20)
                    {
                        Goto(coord_cmd);
                        Speak("我已到达进门地点");
                        sleep(2);
                        cout << "[Main]State: STATE_ACTION" << endl;
                        nState = STATE_ACTION;
                    }
                }
                if(nState == STATE_ACTION)
                {
                    cout << "[Main]创建主任务进程..." << endl;
                    try
                    {
                        Task_Timer = n.createTimer(ros::Duration(0.05), &MainNode::MainCallback, this);
                    }
                    catch (std::exception& e)
                    {
                        std::cout << e.what() << std::endl;
                    }
                }
            }
        }

    private:
        /*---------------ROS定义区---------------*/
        ros::Publisher spk_pub;
        ros::Publisher vel_pub;
        ros::Publisher yolo_pub;
        ros::Subscriber sub_yolo;
        ros::Subscriber sub_ent;
        ros::ServiceClient client_speak;
        ros::ServiceClient cliGetWPName;
        ros::Timer Task_Timer;
        waterplus_map_tools::GetWaypointByName srvName;
        /*---------------全局变量区---------------*/
        int check_flag;              // 程序进入
        int nState = STATE_READY;    // 初始状态
        int nAct   = ACT_CLEAR;      // 任务状态
        int nOpenCount   = 0;        // 开门计数
        int nPeopleCount = 0;        // 人物计数
        int nLitterCount = 0;        // 垃圾计数
        int nPlaceCount  = 1;        // 地点计数
        int nActionStage = 0;        // 动作标志位
        float vel_max    = 0.5;      // 移动限速
        bool bGotoExit   = false;    // 退出标志位
        bool bArrive     = false;    // 到达标志位
        bool bPeopleFound= false;    // 人物标志位
        bool bObjectFound= false;    // 物品标志位
        bool bGrab       = false;    // 抓取标志位
        string strDetect;            // 物品识别
        string coord_cmd;            // 进门坐标
        string coord_exit;           // 出门坐标
        string name_yaml;            // 配置文件
        /*---------------数组/容器区---------------*/
        std::vector<BBox2D> YOLO_BBOX;                    // 识别结果
        std::vector<BBox2D>::const_iterator YOLO_BBOX_IT; // 迭代器
        // 关键词存放容器
        vector<string> arKWPlacement;//地点
        vector<string> arKWObject;   //物品
        vector<string> arKWPerson;   //人名
        vector<string> arKWAction;   //行为

        /// @brief 关键词初始化
        void Init_keywords()
        {
            arKWPlacement.push_back("none");
            arKWPlacement.push_back("none");
            arKWPlacement.push_back("none");
            arKWPlacement.push_back("none");
            arKWPlacement.push_back("none");
            arKWPlacement.push_back("none");
            // 物品关键词
            arKWObject.push_back("Water");
            arKWObject.push_back("Chip");
            arKWObject.push_back("Sprit");
            arKWObject.push_back("Cola");
            arKWObject.push_back("Biscuit");
            arKWObject.push_back("Bread");
            arKWObject.push_back("Lays");
            arKWObject.push_back("Cookie");
            arKWObject.push_back("Hand wash");
            arKWObject.push_back("Orange juice");
            arKWObject.push_back("Dish soap");

            // 人名关键词
            arKWPerson.push_back("Jack");
            arKWPerson.push_back("Linda");
            arKWPerson.push_back("Lucy");
            arKWPerson.push_back("Grace");
            arKWPerson.push_back("John");
            arKWPerson.push_back("Allen");
            arKWPerson.push_back("Richard");
            arKWPerson.push_back("Mike");
            arKWPerson.push_back("Lily");

            // 行为关键词
            arKWAction.push_back("stand");
            arKWAction.push_back("down");
            arKWAction.push_back("lay");
            arKWAction.push_back("walk");
            arKWAction.push_back("make telephone");
            arKWAction.push_back("raise hand");
            arKWAction.push_back("shake hand");
            arKWAction.push_back("shake both hands");
            arKWAction.push_back("smoking");

            cout<<"[Init]关键词初始化完成！" << endl;
        }

        /// @brief 机器人速度修正
        /// @param inVel 输入速度
        /// @param inMax 最大速度
        /// @return 修正后的速度
        float VelFixed(float inVel, float inMax)
        {
            float retVel = inVel;
            if (retVel > inMax)
                retVel = inMax;
            if (retVel < -inMax)
                retVel = -inMax;
            return retVel;
        }

        /// @brief 机器人速度发布
        /// @param inVx 向前/后移动速度
        /// @param inVy 向左/右移动速度
        /// @param inTz 旋转角速度
        void VelCmd(float inVx, float inVy, float inTz)
        {
            geometry_msgs::Twist vel_cmd;
            vel_cmd.linear.x = VelFixed(inVx, vel_max);
            vel_cmd.linear.y = VelFixed(inVy, vel_max);
            vel_cmd.angular.z = VelFixed(inTz, vel_max);
            cout << "设置机器人速度为" << "X:" << inVx << "Y:" << inVy << "Z:" << inTz << endl;
            vel_pub.publish(vel_cmd);
        }

        /// @brief 航点导航
        /// @param inStr 航点名称
        /// @return Ture -> 导航成功 False ->导航失败
        bool Goto(string inStr)
        {
            string strGoto = inStr;
            srvName.request.name = strGoto;
            if (cliGetWPName.call(srvName))
            {
                std::string name = srvName.response.name;
                float x = srvName.response.pose.position.x;
                float y = srvName.response.pose.position.y;
                ROS_INFO("[Goto]Get_wp_name: name = %s (%.2f,%.2f)", strGoto.c_str(), x, y);

                MoveBaseClient ac("move_base", true);
                if (!ac.waitForServer(ros::Duration(5.0)))
                {
                    ROS_INFO("[Goto]The move_base action server is no running. action abort...");
                    return false;
                }
                else
                {
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose = srvName.response.pose;
                    ac.sendGoal(goal);
                    ac.waitForResult();
                    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO("[Goto]Arrived at %s!", strGoto.c_str());
                        return true;
                    }
                    else
                    {
                        ROS_INFO("[Goto]Failed to get to %s ...", strGoto.c_str());
                        return false;
                    }
                }
            }
            else
            {
                ROS_ERROR("Failed to call service GetWaypointByName");
                return false;
            }
        }

        /// @brief 寻找关键词
        /// @param inSentence 传入的句子 -- YoloV5 Node中
        /// @param arWord     关键词    -- Init_Keywords中
        /// @return 得到的关键词
        string FindWord(vector<BBox2D> &YOLO_BBOX, vector<string> &arWord)
        {
            string strRes = "";
            int nNum = arWord.size();
            for (const auto &bbox : YOLO_BBOX)
            {
                for (int j = 0; j < nNum; j++)
                {
                    int tmpIndex = bbox.name.find(arWord[j]);
                    if (tmpIndex >= 0)
                    {
                        strRes = arWord[j];
                        break;
                    }
                }
            }
            return strRes;
        }

        /// @brief 机器人说话（考虑替换为xfyun）
        /// @param answer_txt 说话内容
        void Speak(const std::string &answer_txt)
        {
            robot_voice::StringToVoice::Request req;
            robot_voice::StringToVoice::Response resp;
            req.data = answer_txt;
            bool ok = client_speak.call(req, resp);
            if (ok)
            {
                printf("[Speak]send str2voice service success: %s", req.data.c_str());
            }
            else
            {
                ROS_ERROR("[Speak]failed to send str2voice service");
            }
        }

        /// @brief Yolov5开启状态
        /// @param inStr “start”为开启 开启后默认扫描一次
        /// @return
        void YoloStart()
        {
            std_msgs::String Str;
            Str.data = "start";
            yolo_pub.publish(Str);
        }

        /// @brief 开门检测
        /// @param msg Entrance Detect节点传来的消息
        void EntranceCB(const std_msgs::String::ConstPtr &msg)
        {
            string strDoor = msg->data;
            if (strDoor == "door open")
            {
                nOpenCount++;
            }
            else
            {
                nOpenCount = 0;
            }
        }

        /// @brief 动作识别
        void ActionDetect()
        {
            cout << "[ActionDetect]动作识别开始...." << endl;
            if (nActionStage == 0)
                return;
            if (nActionStage == 1)
            {
                Speak("Hello I will observe your action now");
            }
            if (nActionStage == 2)
            {
                Speak("OK You can perform next action");
                nPeopleCount++;
            }
            string Action = FindWord(YOLO_BBOX, arKWAction);
            printf("识别到动作 - %s \n", Action.c_str());
            Speak(Action);
            sleep(2);
        }

        /// @brief 关键词回调
        /// @param msg 
        void KeywordCB(const wpb_yolo5::BBox2D &msg)
        {
            cout << "[KeywordCB]:接收到Yolov5数据" << endl;
            YOLO_BBOX.clear();
            int nNum = msg.name.size();
            bool bAction = false;
            if (nNum > 0)
            {
                std::vector<BBox2D> recv_BBOX; // 收到的物品
                BBox2D box_object;             // bbox格式object 存入收到的msg
                for (int i = 0; i < nNum; i++)
                {
                    box_object.name        = msg.name[i];        // 识别到的名字
                    box_object.left        = msg.left[i];        // x_min
                    box_object.right       = msg.right[i];       // x_max
                    box_object.top         = msg.top[i];         // y_min
                    box_object.bottom      = msg.bottom[i];      // y_max
                    box_object.probability = msg.probability[i]; // 置信度
                    recv_BBOX.push_back(box_object);
                    std::string strDetect = msg.name[i];
                }
                YOLO_BBOX = recv_BBOX; // 存入object
            }
        }

        /// @brief 视角修正
        void Fixed_View()
        {
            cout << "[Fixed_View]位姿修正开始..." << endl;
            //int size=sizeof(F2->num);
            // int num = F2->num;

            // ROS_INFO("***********************************");
            // if (num != 0 | num != NULL)
            // {
            //     for (int i = 0; i < num; i++)
            //     {

            //         ROS_INFO("目标%d的坐标信息x为：%f,坐标信息y为：%f,坐标信息w为：%f,坐标信息h为：%f", i, F2->object[i].x, F2->object[i].y, F2->object[i].w, F2->object[i].h);
            //     }
            // }

            // ROS_INFO("------------------GimbalStop-------------");
            // machine.GimbalStop();
            // ROS_INFO("------------------GimbalStop2-------------");

            // if (num != 0 | num != NULL)
            // {
            //     if (F2->object[0].x != NULL & F2->object[0].y != NULL)
            //     {
            //         ROS_INFO("------------------Machinw-------------");
            //         if (F2->object[0].x < 0.4) // SP zuo,DJ you
            //         {
            //             machine.GimbalMove(15, 0);
            //         }
            //         else if (F2->object[0].y < 0.4) // SP xia,DJ xia
            //         {
            //             machine.GimbalMove(0, -15);
            //         }
            //         else if (F2->object[0].x > 0.6) // SP you ,DJ zuo
            //         {
            //             machine.GimbalMove(-15, 0);
            //         }
            //         else if (F2->object[0].y > 0.6) // SP shang,DJ shang
            //         {
            //             machine.GimbalMove(0, 15);
            //         }
            //         else
            //             machine.GimbalStop();
            //     }
            // }
            // else
            // {
            //     machine.GimbalStop();
            //     ROS_INFO("------------------GimbalStop3-------------");
            // }
        }

        /// @brief OpenPose回调
        void OpenPoseCB()
        {
            
        }

        /// @brief 物品抓取
        void Grab()
        {
            cout<< "[Grub]物品抓取开始..."  << endl;
        }

        /// @brief 程序参数打印
        void Parameter_Check()
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>> Parameter_Check <<<<<<<<<<<<<<<<<<<<<<<" << endl;
            cout << "Yaml Name:"   << name_yaml << endl;
            cout << "Enter Coord:" << coord_cmd << endl;
            cout << "Exit  Coord:" << coord_exit << endl;
            cout << "Place1:" << arKWPlacement[1] << endl;
            cout << "Place2:" << arKWPlacement[2] << endl;
            cout << "Place3:" << arKWPlacement[3] << endl;
            cout << "Place4:" << arKWPlacement[4] << endl;
            cout << "State:"  << nState << endl;
            cout << "Act:"    << nAct   << endl;
            cout << ">>>>>>>>>>>>>>>>>>>> Please check the parameter <<<<<<<<<<<<<<<<<<" << endl;

        }

        /// @brief 主程序
        /// @param e 
        void MainCallback(const ros::TimerEvent& e)
        {
            if(nAct == ACT_CLEAR && bGotoExit!= true)
            {
                cout << "[Main]正在前往地点：" << arKWPlacement[nPlaceCount] << endl;
                Goto(arKWPlacement[nPlaceCount]);
                nPlaceCount++;
                nAct = ACT_FIND_PERSON;
                sleep(1);                
            }

            if(nAct == ACT_FIND_PERSON)
            {
                if (!bPeopleFound)
                {
                    VelCmd(0, 0, 0.1);
                }
                else
                {
                    Fixed_View();
                    ActionDetect();
                    nPeopleCount++;
                    nAct = ACT_FIND_OBJECT;
                }
            }

            if(nAct == ACT_FIND_OBJECT)
            {
                if(!bObjectFound)
                {
                    VelCmd(0, 0, 0.1);
                }
                else
                {
                    Grab();//预留接口 wzy写
                    nLitterCount++;
                    nPlaceCount++;
                    nAct = ACT_CLEAR;
                }
            }

            if((nPeopleCount == 3 && nLitterCount == 3) && bGrab!= true)
                bGotoExit = true;

            if(bGotoExit == true)
            {
                cout << "[Main]任务完成!前往退出地点...." << endl;
                Goto(coord_exit);
                sleep(5);
                nState       = STATE_READY;
                nAct         = ACT_CLEAR;
                nActionStage = 0;
            }
        }
    };
}

#endif