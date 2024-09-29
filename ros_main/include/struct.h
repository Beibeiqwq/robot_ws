#ifndef HOME_STRUCTS
#define HOME_STRUCTS
#include<string>

using namespace std;

#define ACT_GOTO			1
#define ACT_DETECT_ACTION_0 2
#define ACT_DETECT_ACTION_1 3
#define ACT_FIND_OBJ		4
#define ACT_GRAB			5
#define ACT_PASS			6 //DUSTBIN
#define ACT_SPEAK			101
#define ACT_MOVE			102
#define ACT_ADD_WAYPOINT	103

typedef struct stAct
{
	int nAct;			//行为号
	string  strTarget;	//移动目标航点名称/语音说话内容/语音识别关键词
	float nDuration;	//语音持续时间/语音识别周期/视频录制时间
	float fLinear_x;		//前后平移移动
	float fLinear_y;		//左右平移速度
	float fAngular_z;	//旋转速度,正值向左旋转,负值向右旋转
	float fFollowDist;	//跟随距离
}stAct;


#endif // HOME_STRUCTS