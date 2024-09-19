#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <robot_voice/StringToVoice.h>


class RobotController {
public:
  RobotController() {
    ROS_INFO("RobotController Constructor");
  }

  ~RobotController() {
    ROS_INFO("RobotController Destructor");
  }

  int Init(ros::NodeHandle& nh) {
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    client_ = nh.serviceClient<robot_voice::StringToVoice>("str2voice");
    return 0;
  }

  void ToDownstream(const std::string& answer_txt, float linear_x, float angular_z) {
  	// 通过 str2voice 服务和 /cmd_vel topic向下游 voice_creator 和 mbot_gazebo 发送
    robot_voice::StringToVoice::Request req;
    robot_voice::StringToVoice::Response resp;
    req.data = answer_txt;

    bool ok = client_.call(req, resp);
    if (ok) {
      printf("send str2voice service success: %s, and pub cmd_vel\n", req.data.c_str());
       geometry_msgs::Twist msg;
       msg.linear.x = linear_x;
       msg.angular.z = angular_z;
       cmd_pub_.publish(msg);
    } else {
      ROS_ERROR("failed to send str2voice service");
    }
  }

  bool ChatterCallbback(robot_voice::StringToVoice::Request &req, robot_voice::StringToVoice::Response &resp) {
    printf("i received: %s\n", req.data.c_str());
    std::string voice_txt = req.data;
	// 根据指令关键字，发送对应的语音播包文字和 cmd_vel 命令
    if (voice_txt.find("你好") != std::string::npos) {
      ToDownstream("王泽与大沙比", 0, 0);
    } else if (voice_txt.find("再见") != std::string::npos) {
      ToDownstream("王则与大二比", 0, 0);}
    resp.success = true;
    return resp.success;
  }

  void Start(ros::NodeHandle& nh) {
  	// 申明 human_chatter 服务，ChatterCallbback是回调函数
    chatter_server_ = nh.advertiseService("human_chatter", &RobotController::ChatterCallbback, this);
  }

private:
  ros::ServiceServer chatter_server_;
  ros::Publisher cmd_pub_;
  ros::ServiceClient client_;
};

int main(int argc, char* argv[]) {
  int ret = 0;
  ros::init(argc, argv, "voice_controller");
  ros::NodeHandle nh;

  RobotController rc;
  rc.Init(nh);

  printf("this is a voice controller app for robot, you can say: 向前, 向后, 向左, 向右, 转圈, 结束\n");
  rc.Start(nh);

  ros::spin();
  return 0;
}

