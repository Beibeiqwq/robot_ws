#include "action_manager.h"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static ros::Publisher spk_pub;
static ros::Publisher vel_pub;
static string strToSpeak = "";
static string strKeyWord = "";
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static ros::ServiceClient cliGetWPName;
static waterplus_map_tools::GetWaypointByName srvName;
static ros::ServiceClient follow_start;
static ros::ServiceClient follow_stop;
static wpb_home_tutorials::Follow srvFlw;
static ros::Publisher behaviors_pub;
static std_msgs::String behavior_msg;
static ros::Publisher add_waypoint_pub;
static int nToRecoFrame = 100;
//构造函数
CActionManager::CActionManager()
{
    nCurActIndex = 0;
    nCurActCode = -1;
    strListen = "";
    bGrabDone = false;
    bPassDone = false;
}
//析构函数
CActionManager::~CActionManager()
{

}

//初始化
void CActionManager::Init()
{
    ros::NodeHandle n;
    //客户端::获得航点名字
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    //发布者::语音播报
    spk_pub = n.advertise<std_msgs::String>("/xfyun/tts", 20);
    //发布者::速度控制
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //客户端::语音识别
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");
    //发布者::行为发布
    behaviors_pub = n.advertise<std_msgs::String>("/wpb_home/behaviors", 30);
    //发布者::航点添加
    add_waypoint_pub = n.advertise<waterplus_map_tools::Waypoint>( "/waterplus/add_waypoint", 1);
    //订阅者::抓取结果
    grab_result_sub = n.subscribe<std_msgs::String>("/wpb_home/grab_result",30,&CActionManager::GrabResultCallback,this);
    //订阅者::物品递给结果
    pass_result_sub = n.subscribe<std_msgs::String>("/wpb_home/pass_result",30,&CActionManager::PassResultCallback,this);

}

//抓取开关
static void GrabSwitch(bool inActive)
{
    if(inActive == true)
    {
        behavior_msg.data = "grab start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "grab stop";
        behaviors_pub.publish(behavior_msg);
    }
}
//递给开关
static void PassSwitch(bool inActive)
{
    if(inActive == true)
    {
        behavior_msg.data = "pass start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "pass stop";
        behaviors_pub.publish(behavior_msg);
    }
}
//新航点添加
static void AddNewWaypoint(string inStr)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("/map","/base_footprint",  ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/map","/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("[lookupTransform] %s",ex.what());
        return;
    }

    float tx = transform.getOrigin().x();
    float ty = transform.getOrigin().y();
    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(transform.getRotation() , tf::Point(tx, ty, 0.0)), ros::Time::now(), "map");
    geometry_msgs::PoseStamped new_pos;
    tf::poseStampedTFToMsg(p, new_pos);

    waterplus_map_tools::Waypoint new_waypoint;
    new_waypoint.name = inStr;
    new_waypoint.pose = new_pos.pose;
    add_waypoint_pub.publish(new_waypoint);

    ROS_WARN("[New Waypoint] %s ( %.2f , %.2f )" , new_waypoint.name.c_str(), tx, ty);
}
//初始化上一次任务
static int nLastActCode = -1;
static geometry_msgs::Twist vel_cmd;
//主状态机
bool CActionManager::Main()
{
    //任务个数
    int nNumOfAct = arAct.size();
    //结束判定
    if(nCurActIndex >= nNumOfAct)
    {
        return false;
    }
    //语音识别的关键词
    int nKeyWord = -1;
    //当前任务状态
    nCurActCode = arAct[nCurActIndex].nAct;
    //nCurActIndex == 当前任务ID
    //nLastActCode == 上一个任务ID
    switch (nCurActCode)
	{
	case ACT_GOTO:
		if (nLastActCode != ACT_GOTO)
		{
			string strGoto = arAct[nCurActIndex].strTarget;
            printf("[ActMgr] %d - Goto %s",nCurActIndex,strGoto.c_str());
            srvName.request.name = strGoto;
            if (cliGetWPName.call(srvName))
            {
                std::string name = srvName.response.name;
                float x = srvName.response.pose.position.x;
                float y = srvName.response.pose.position.y;
                ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)", strGoto.c_str(),x,y);

                MoveBaseClient ac("move_base", true);
                if(!ac.waitForServer(ros::Duration(5.0)))
                {
                    ROS_INFO("The move_base action server is no running. action abort...");
                }
                else
                {
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose = srvName.response.pose;
                    ac.sendGoal(goal);
                    ac.waitForResult();
                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        ROS_INFO("Arrived at %s!",strGoto.c_str());
                    else
                        ROS_INFO("Failed to get to %s ...",strGoto.c_str() );
                }
                
            }
            else
            {
                ROS_ERROR("Failed to call service GetWaypointByName");
            }
            nCurActIndex ++;
        }
		break;

	case ACT_FIND_OBJ:
		if (nLastActCode != ACT_FIND_OBJ)
		{
            printf("[ActMgr] %d - Find %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            nCurActIndex ++;
		}
		break;

	case ACT_GRAB:
		if (nLastActCode != ACT_GRAB)
		{
            printf("[ActMgr] %d - Grab %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            bGrabDone = false;
            GrabSwitch(true);
		}
        if(bGrabDone == true)
        {
            printf("[ActMgr] %d - Grab %s done!\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            GrabSwitch(false);
            nCurActIndex ++;
        }
		break;

	case ACT_PASS:
		if (nLastActCode != ACT_PASS)
		{
            printf("[ActMgr] %d - Pass %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            bPassDone = false;
            PassSwitch(true);
		}
        if(bPassDone == true)
        {
            printf("[ActMgr] %d - Pass %s done!\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            PassSwitch(false);
            nCurActIndex ++;
        }
		break;

	case ACT_SPEAK:
		if (nLastActCode != ACT_SPEAK)
		{
            printf("[ActMgr] %d - Speak %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            strToSpeak = arAct[nCurActIndex].strTarget;
            std_msgs::String rosSpeak;
            rosSpeak.data = strToSpeak;
            spk_pub.publish(rosSpeak);
            strToSpeak = "";
            usleep(arAct[nCurActIndex].nDuration*1000*1000);
            nCurActIndex ++;
		}
		break;

	case ACT_LISTEN:
		if (nLastActCode != ACT_LISTEN)
		{
            printf("[ActMgr] %d - Listen %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            strListen = "";
            strKeyWord = arAct[nCurActIndex].strTarget;
            int nDur = arAct[nCurActIndex].nDuration;
            if(nDur < 3)
            {
                nDur = 3;
            }
            //开始语音识别
            srvIAT.request.active = true;
            srvIAT.request.duration = nDur;
            clientIAT.call(srvIAT);
		}
        nKeyWord = strListen.find(strKeyWord);
        if(nKeyWord >= 0)
        {
            //识别完毕,关闭语音识别
            srvIAT.request.active = false;
            clientIAT.call(srvIAT);
            nCurActIndex ++;
        }
		break;

    case ACT_MOVE:
        printf("[ActMgr] %d - Move ( %.2f , %.2f ) - %.2f\n",nCurActIndex,arAct[nCurActIndex].fLinear_x,arAct[nCurActIndex].fLinear_y,arAct[nCurActIndex].fAngular_z);
        vel_cmd.linear.x = arAct[nCurActIndex].fLinear_x;
        vel_cmd.linear.y = arAct[nCurActIndex].fLinear_y;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = arAct[nCurActIndex].fAngular_z;
        vel_pub.publish(vel_cmd);

        usleep(arAct[nCurActIndex].nDuration*1000*1000);
        nCurActIndex ++;
		break;

    case ACT_ADD_WAYPOINT:
		if (nLastActCode != ACT_ADD_WAYPOINT)
		{
            printf("[ActMgr] %d - Add waypoint %s \n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            AddNewWaypoint(arAct[nCurActIndex].strTarget);
            nCurActIndex ++;
		}
		break;

	default:
		break;
	}
    //标记当前行为
	nLastActCode = nCurActCode;
    return true;
}
//状态重置
void CActionManager::Reset()
{
    strToSpeak = "";
    nCurActIndex = 0;
	nLastActCode = 0;
    arAct.clear();
}

string CActionManager::GetToSpeak()
{
    string strRet = strToSpeak;
    strToSpeak = "";
    return strRet;
}

string ActionText(stAct* inAct)
{
    string ActText = "";
    if(inAct->nAct == ACT_GOTO)
    {
        ActText = "去往地点 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_FIND_OBJ)
    {
        ActText = "搜索物品 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_GRAB)
    {
        ActText = "抓取物品 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_PASS)
    {
        ActText = "把物品递给 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_SPEAK)
    {
        ActText = "说话 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_LISTEN)
    {
        ActText = "听取关键词 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_MOVE)
    {
        ActText = "移动 ( ";
        std::ostringstream stringStream;
        stringStream << inAct->fLinear_x << " , " << inAct->fLinear_y << " ) - " << inAct->fAngular_z;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    if(inAct->nAct == ACT_ADD_WAYPOINT)
    {
        ActText = "添加航点 ";
        std::ostringstream stringStream;
        stringStream << inAct->fFollowDist;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    return ActText;
}

void CActionManager::ShowActs()
{
    printf("\n*********************************************\n");
    printf("显示行为队列:\n");
    int nNumOfAct = arAct.size();
    stAct tmpAct;
    for(int i=0;i<nNumOfAct;i++)
    {
        tmpAct = arAct[i];
        string act_txt = ActionText(&tmpAct);
        printf("行为 %d : %s\n",i+1,act_txt.c_str());
    }
    printf("*********************************************\n\n");
}

void CActionManager::GrabResultCallback(const std_msgs::String::ConstPtr& res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if( nFindIndex >= 0 )
    {
        bGrabDone = true;
    }
}

void CActionManager::PassResultCallback(const std_msgs::String::ConstPtr& res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if( nFindIndex >= 0 )
    {
        bPassDone = true;
    }
}
