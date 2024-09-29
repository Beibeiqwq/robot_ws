#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include "action_manager.h"
#include <sound_play/SoundRequest.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "xfyun_waterplus/IATSwitch.h"
#include <waterplus_map_tools/GetWaypointByName.h>
#include <robot_voice/StringToVoice.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

static ros::Publisher spk_pub;
static ros::Publisher vel_pub;
static ros::Publisher yolo_pub;
static ros::ServiceClient client_speak;
static ros::ServiceClient cliGetWPName;
// static xfyun_waterplus::IATSwitch srvIAT;
static waterplus_map_tools::GetWaypointByName srvName;
static CActionManager action_manager;

// 有限状态机
#define STATE_READY 0
#define STATE_WAIT_ENTR 1
#define STATE_WAIT_CMD 2
#define STATE_ACTION 3
#define STATE_CHECK 4
#define STATE_GOTO_EXIT 5

#define FIND_PERSON 6
#define FIND_OBJECT 7

static int nState = STATE_WAIT_ENTR; // 程序启动时初始状态;
static float vel_max = 0.5;          // 移动限速
static int nPeopleCount = 0;
static int nLitterCount = 0;
static bool bGotoExit = false; // 任务完成后退出
static bool bArrive = false;   // 是否到达目标航点
static string strDetect;

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

static std::vector<BBox2D> YOLO_BBOX;                    // 识别结果
static std::vector<BBox2D>::const_iterator YOLO_BBOX_IT; // 迭代器
// 关键词存放容器
static vector<string> arKWPlacement;
static vector<string> arKWObject;
static vector<string> arKWPerson;
static vector<string> arKWAction;

/// @brief 关键词初始化
static void Init_keywords()
{
    // 地点关键词
    arKWPlacement.push_back("living room");
    arKWPlacement.push_back("kitchen");
    arKWPlacement.push_back("room");

    // 物品关键词
    arKWObject.push_back("Water");
    arKWObject.push_back("Chip");
    // arKWObject.push_back("Cheap");
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
    vel_pub.publish(vel_cmd);
}

/// @brief 寻找关键词
/// @param inSentence 传入的句子 -- YoloV5 Node中
/// @param arWord     关键词    -- Init_Keywords中
/// @return 得到的关键词
static string FindWord(vector<BBox2D> &YOLO_BBOX, vector<string> &arWord)
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

/// @brief 航点导航
/// @param inStr 航点名称
/// @return Ture -> 导航成功 False ->导航失败
static bool Goto(string inStr)
{
    string strGoto = inStr;
    srvName.request.name = strGoto;
    if (cliGetWPName.call(srvName))
    {
        std::string name = srvName.response.name;
        float x = srvName.response.pose.position.x;
        float y = srvName.response.pose.position.y;
        ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)", strGoto.c_str(), x, y);

        MoveBaseClient ac("move_base", true);
        if (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("The move_base action server is no running. action abort...");
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
                ROS_INFO("Arrived at %s!", strGoto.c_str());
                return true;
            }
            else
            {
                ROS_INFO("Failed to get to %s ...", strGoto.c_str());
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

/// @brief 机器人说话（考虑替换为xfyun）
/// @param answer_txt 说话内容
static void Speak(const std::string &answer_txt)
{
    robot_voice::StringToVoice::Request req;
    robot_voice::StringToVoice::Response resp;
    req.data = answer_txt;
    bool ok = client_speak.call(req, resp);
    if (ok)
    {
        printf("send str2voice service success: %s", req.data.c_str());
    }
    else
    {
        ROS_ERROR("failed to send str2voice service");
    }
}

/// @brief Yolov5开启状态
/// @param inStr “start”为开启 开启后默认扫描一次
/// @return
static void YoloStart()
{
    std_msgs::String Str;
    Str.data = "start";
    yolo_pub.publish(Str);
}

static int nOpenCount = 0; // 延时 防止误判
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
static int nActionStage = 0;
void ActionDetect()
{
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
    YoloStart();
    string Action = FindWord(YOLO_BBOX, arKWAction);
    printf("识别到动作 - %s \n", Action.c_str());
    Speak(Action);
    sleep(2);
}
void KeywordCB(const wpb_yolo5::BBox2D &msg)
{
    ROS_WARN("[KeywordCB]:Receive Yolov5 msg");
    YOLO_BBOX.clear();
    int nNum = msg.name.size();
    bool bAction = false;
    if (nNum > 0)
    {
        std::vector<BBox2D> recv_BBOX; // 收到的物品
        BBox2D box_object;             // bbox格式object 存入收到的msg
        for (int i = 0; i < nNum; i++)
        {
            box_object.name = msg.name[i];               // 识别到的名字
            box_object.left = msg.left[i];               // x_min
            box_object.right = msg.right[i];             // x_max
            box_object.top = msg.top[i];                 // y_min
            box_object.bottom = msg.bottom[i];           // y_max
            box_object.probability = msg.probability[i]; // 置信度
            recv_BBOX.push_back(box_object);
            std::string strDetect = msg.name[i];
        }
        YOLO_BBOX = recv_BBOX; // 存入object
    }

    if (nState == FIND_PERSON)
    {
        // 识别到动作 -->开启识别姿态
        string Action = FindWord(YOLO_BBOX, arKWAction);
        if (Action.length() > 0)
        {
            ++nActionStage;
            if (nActionStage == 1)
            {
                Speak("Hello I will observe your action now");
                sleep(2);
                YoloStart();
            }
            if (nActionStage == 2)
            {
                string Action = FindWord(YOLO_BBOX, arKWAction);
                printf("识别到动作 - %s \n", Action.c_str());
                Speak(Action);
                YoloStart();
            }
            if (nActionStage == 3)
            {
                Speak("OK You can perform next action");
                YoloStart();
            }
            if (nActionStage == 4)
            {
                string Action = FindWord(YOLO_BBOX, arKWAction);
                printf("识别到动作 - %s \n", Action.c_str());
                Speak(Action);
                YoloStart();
                nPeopleCount++;
                nState = FIND_OBJECT;
            }
            YoloStart();
            // ActionDetect();
        }
    }
    if (nState == FIND_OBJECT)
    {
        // 识别到物品-->开启物品抓取
        nActionStage = 0;
        string object = FindWord(YOLO_BBOX, arKWObject);
        if (object.length() > 0)
        {
            printf("识别到物品 - %s \n", object.c_str());
            stAct newAct;
            newAct.nAct = ACT_GRAB;
            newAct.strTarget = object;
            action_manager.arAct.push_back(newAct);
            bAction = true;
            // 物品抓取(空)
            nLitterCount++;
            Goto(arKWPlacement.back());
            arKWPlacement.pop_back();
            nState = FIND_PERSON;
        }
    }
    if (bAction == true)
    {
        nState = STATE_ACTION; // 物品抓取
    }

    if (nPeopleCount == 3 && nLitterCount == 3)
    {
        nState = STATE_GOTO_EXIT;
        bGotoExit = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROBOCUP_MAIN");
    Init_keywords();
    action_manager.Init();

    ros::NodeHandle n;
    ros::Subscriber sub_yolo = n.subscribe("/yolo_bbox_2d", 10, KeywordCB);
    ros::Subscriber sub_ent = n.subscribe("/wpb_home/entrance_detect", 10, EntranceCB);

    client_speak = n.serviceClient<robot_voice::StringToVoice>("str2voice");
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    yolo_pub = n.advertise<std_msgs::String>("/yolov5/cmd", 20);

    ROS_INFO("[main] ROBOCUP@HOME START!");
    ros::Rate r(10);
    while (ros::ok())
    {
        if (nState == STATE_WAIT_ENTR)
        {
            if (nOpenCount > 20)
            {
                Goto("cmd");
                Speak("I have reached the target position");
                sleep(2); // 停留
                Goto("living room");
                nState = FIND_PERSON;
                YoloStart();
            }
        }
        if (nState == STATE_ACTION)
        {
            action_manager.Main();
        }
        if (nState == STATE_GOTO_EXIT && bGotoExit == true)
        {
            Goto("exit");
            Speak("Task Finish");
            sleep(1000);
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}