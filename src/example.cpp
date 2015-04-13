// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// Object detection
#include <object_detection/DetectObject.h>
#include <PoseEstimation/PoseEstimation.h>
#include <termios.h>

using namespace PoseEstimation;
typedef object_detection::DetectObject MsgT;
vector<ros::Subscriber*> point_cloud_vector;
pcl::PCDWriter pcdWriter;

void pointCloudCallback(sensor_msgs::PointCloud2 & PointCloudROS)
{

	std::string frame_id = PointCloudROS.header.frame_id;
	if (true)
	{
		pcl::PointCloud < pcl::PointXYZRGBA > PointCloudPCL;
		pcl::fromROSMsg < pcl::PointXYZRGBA > (PointCloudROS, PointCloudPCL);
		std::stringstream pcdFilePath;
		pcdFilePath << "cloud00.pcd";

		pcdWriter.writeBinaryCompressed(pcdFilePath.str().c_str(), PointCloudPCL);
		cout << "Files saved: " << pcdFilePath.str() << endl;
	}
}

void kinectPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& PointCloudROS)
{

	sensor_msgs::PointCloud2 PointCloud = *PointCloudROS;
	pointCloudCallback(PointCloud);
	pcl::PointCloud < pcl::PointXYZRGBA > PointCloudPCL;
	pcl::fromROSMsg < pcl::PointXYZRGBA > (PointCloud, PointCloudPCL);

}

void kinectPointCloudSaver(ros::NodeHandle nh, string name)
{

	std::stringstream PointCloudPath;
	PointCloudPath << name << "/depth_registered/points";

	point_cloud_vector.push_back(new ros::Subscriber());
	*point_cloud_vector.back() = nh.subscribe(PointCloudPath.str(), 1, kinectPointCloudCallback);

}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


int main(int argc, char** argv) {
	// Initialize
	ros::init(argc, argv, "example");
	ros::NodeHandle n("~");


	kinectPointCloudSaver(n, "/carmine1");


	ProgramOptions po;
	po.addPositional("save", "pcd save name");
	po.addPositional("objects", "object file(s)");
	po.addPositional("scene", "scene file");

	const char** argv2 = (const char**)argv;
		// Parse and print
	if ( !po.parse(argc, argv2) )
		return 1;
	po.print();
	//get point cloud from carmine
	while (ros::ok())
	{
	    std::cout << "Please enter: " << std::endl;
		int c = getch();   // call your non-blocking input function
		if (c == 'a')
			kinectPointCloudSaver(n, "/carmine1");
	}




	//---POSE ESTIMATION BLOCK
	// Subscribe to global estimation service
	ROS_INFO("Subscribing to global object detection service...");
	ros::service::waitForService("/object_detection/global");
	ros::ServiceClient global = n.serviceClient<MsgT>("/object_detection/global");

	sensor_msgs::PointCloud2 scenei;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr b(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	pcl::io::loadPCDFile( po.getValue("scene"), *b);
	pcl::toROSMsg(*b, scenei);

	MsgT msgglobal;
	const std::vector<std::string> objects = po.getVector("objects");
	for (size_t o = 0; o < objects.size(); o++) {
		msgglobal.request.objects.push_back(objects[o]);
	}
	msgglobal.request.cloud = scenei;
	if(global.call(msgglobal)) {
		std::cout << "number of ints:" << msgglobal.response.labels_int.size() << std::endl;
		std::cout << "number of pose_value:" << msgglobal.response.pose_value.size() << std::endl;
		for(size_t n = 0; n < msgglobal.response.pose_value.size(); n++) {
			pcl::console::print_warn("Pose goodness value: %f\n", msgglobal.response.pose_value[n]);
		}
	} else {
		ROS_ERROR("Something went wrong when calling global object detection service");
	}
	 
	ros::spinOnce();

	return 0;
}
