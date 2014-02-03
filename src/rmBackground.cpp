#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/ros/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>


sensor_msgs::PointCloud2::ConstPtr PclMsg;

bool recPcl;

void cloudCb (const sensor_msgs::PointCloud2::ConstPtr& msg) {
    PclMsg = msg;
    recPcl = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yxzFigures");
    ros::NodeHandle nh;
    ros::Subscriber pclSub;
    ros::Publisher pclPub, filterPub;

    pclSub = nh.subscribe("velodyne_points", 1, cloudCb);
    pclPub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_filtered",1);
    filterPub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_filter",1);

    bool firstrun = true;
    ros::Rate loopRate(100);

    sensor_msgs::PointCloud2 msg, filter_msg;
    pcl::PointCloud<pcl::PointXYZ>  in, filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_ptr, in_ptr, out_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // Octree resolution - side length of octree voxels
    float resolution = 0.10f;
    int minLeafPoints = 10;



    while(ros::ok()){

        if(recPcl){
            ROS_INFO("New pointcloud");
            recPcl = false;
            // Instantiate octree-based point cloud change detection class
            pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
            // get pcl::PointCloud<pcl::PointXYZ>::Ptr
            pcl::fromROSMsg(*PclMsg, in);
            in_ptr = in.makeShared();
            if(firstrun){
                ROS_INFO("Firstrun");
                firstrun = false;
                // get pcl::PointCloud<pcl::PointXYZ>::Ptr

                pcl::fromROSMsg(*PclMsg, filter);
                filter_ptr = filter.makeShared();

            } else {
//                octree.deleteTree();

            }

            // Add points from cloudA to octree
            octree.setInputCloud (filter_ptr);
            octree.addPointsFromInputCloud ();

            // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
            octree.switchBuffers ();

            // Add points from cloudB to octree
            octree.setInputCloud (in_ptr);
            octree.addPointsFromInputCloud ();


            // Vector containing all indicies
            std::vector<int> newPointIdxVector;
            // Get vector of point indices from octree voxels which did not exist in previous buffer
            octree.getPointIndicesFromNewVoxels (newPointIdxVector, minLeafPoints);

            ROS_INFO("indicies %d", newPointIdxVector.size());

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;

            filtered_cloud = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ> ();

            filtered_cloud->points.reserve(newPointIdxVector.size());

            for (std::vector<int>::iterator it = newPointIdxVector.begin (); it != newPointIdxVector.end (); it++)
                filtered_cloud->points.push_back(in_ptr->points[*it]);

            msg.header = PclMsg->header;
            filtered_cloud->header = PclMsg->header;
            pcl::toROSMsg(*(filtered_cloud.get()), msg);
            pclPub.publish(msg);

            filter_msg.header = PclMsg->header;
            pcl::toROSMsg(filter, filter_msg);
            filterPub.publish(filter_msg);
        }

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
