#include<ros/ros.h>
#include<std_msgs/String.h>
#include<vector>
#include<sound_play/SoundRequest.h>
#include "yolov5_ros/BoundingBoxes.h"
#include "yolov5_ros/BoundingBox.h"
static ros::Publisher spk_pub;
static ros::Subscriber yolo_sub;
using namespace std;

static std::string Object_Detect_Result;
static std::string person = "Person";
static bool Person_speak;
//yolov5_ros_msgs::BoundingBox Object_Detect_Result;
// yolov5_ros_msgs::BoundingBox_ BoundingBox_ 
//std::vector<yolov5_ros_msgs::BoundingBox> Object_Detect_Result;
static void Speak(std::string inStr)
{
    sound_play::SoundRequest spk;
    spk.sound = sound_play::SoundRequest::SAY;
    spk.command = sound_play::SoundRequest::PLAY_ONCE;
    spk.arg = inStr;
    spk.volume = 1.0;
    spk_pub.publish(spk);
}
void Yolov5Callback(const yolov5_ros::BoundingBoxes::ConstPtr& Detect_result)
{
     Object_Detect_Result = Detect_result->class_name;
    // Object_Detect_Result_Class = Object_Detect_Result;
    //ROS_INFO("Name==%s",Object_Detect_Result.c_str());
             if(Object_Detect_Result == "person"){
            //  Speak(Object_Detect_Result);
              ROS_INFO("SPEAK PERSON");
            Person_speak = 1;
         }
         else Person_speak = 0;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "object_speak");
    ros::NodeHandle nh;
    ros::Publisher spk_pub = nh.advertise<sound_play::SoundRequest>("/robotsound",20);
    ros::Subscriber yolo_sub = nh.subscribe("/yolov5/BoundingBoxes",10,&Yolov5Callback);
    ROS_INFO("NODE START!");
    Speak(person);
    
    ros::Rate r(10);
    // while (ros::ok())
    // {
    //     if(Person_speak)
    //     {
    //         Speak("Person");
    //         ROS_INFO("SPEAK PERSON");
    //     }
    //     //ros::spinOnce();
    // }
    ros::spin();
    return 0;
}