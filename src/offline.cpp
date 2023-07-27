#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <util.hpp>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bag_to_pcd");

  std::string bag_file = argv[1];
  std::string topic_name = argv[2];
  std::string save_dir = argv[3];
  std::string save_folder_name = save_dir + "/" + create_date();
  boost::filesystem::create_directory(save_folder_name);

  rosbag::Bag bag;
  try {
    bag.open(bag_file, rosbag::bagmode::Read);
  } catch (rosbag::BagException& e) {
    ROS_ERROR("Error opening file %s", bag_file.c_str());
    return -1;
  }

  std::vector<std::string> topics = {topic_name};
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (const rosbag::MessageInstance& m : view) {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != nullptr) {
      std::string time_stamp = std::to_string(msg->header.stamp.sec + msg->header.stamp.nsec / 1e9);
      pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*msg, *points);

      if (points->empty()) {
        ROS_ERROR("No point cloud data in bag file");
        return -1;
      }

      std::string filename = save_folder_name + "/" + time_stamp + ".pcd";
      pcl::io::savePCDFileBinary(filename, *points);
    }
  }

  bag.close();

  return 0;
}