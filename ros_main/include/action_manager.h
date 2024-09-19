#ifndef WP_ACTION_MANAGER_H
#define WP_ACTION_MANAGER_H
#include "struct.h"
#include <vector>
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
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
//YOLOV5 HEADER
#include <wpb_yolo5/BBox2D.h>
#include <wpb_yolo5/BBox3D.h>

using namespace cv;
class CActionManager
{
public:
	CActionManager();
	~CActionManager();

    vector<stAct> arAct; //任务队列
	int nCurActIndex;
	int nCurActCode;
	std::string strListen;

	ros::Subscriber grab_result_sub;
	ros::Subscriber pass_result_sub;
	bool bGrabDone;
	bool bPassDone;

	bool Main();
	void Init();
	void Reset();
	string GetToSpeak();
	void ShowActs();
	void ProcColorCB(const sensor_msgs::ImageConstPtr& msg);
 	void GrabResultCallback(const std_msgs::String::ConstPtr& res);
	void PassResultCallback(const std_msgs::String::ConstPtr& res);
};

#endif // WP_ACTION_MANAGER_H