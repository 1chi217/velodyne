#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/ros/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#define LEAFE_SIZE                  10
#define RESOLUTION                  0.1f
#define MIN_FILTERED_CLOUD_SIZE     100
#define MIN_CLUSTERED_CLOUD_SIZE    300

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
    ros::Publisher filteredPub, filterPub, clusteredPub;

    pclSub = nh.subscribe("velodyne_points", 1, cloudCb);
    filteredPub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_filtered",1);
    clusteredPub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_clustered",1);
    filterPub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_filter",1);

    bool firstrun = true;
    ros::Rate loopRate(100);

    sensor_msgs::PointCloud2 msg, filter_msg;
    pcl::PointCloud<pcl::PointXYZ>  in, filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_ptr, in_ptr;

    // Octree resolution - side length of octree voxels
    float resolution = RESOLUTION;
    // Cancel out noise -> Leafs should consist of 10 points at least
    int minLeafPoints = LEAFE_SIZE;



    while(ros::ok()){

        // if we have received a new pointcloud
        if(recPcl){
            ROS_INFO("New pointcloud");
            recPcl = false;

            // Instantiate octree-based point cloud change detection class
            pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

            // get pcl::PointCloud<pcl::PointXYZ>::Ptr from pcl ROS msg
            pcl::fromROSMsg(*PclMsg, in);
            in_ptr = in.makeShared();

            if(firstrun){
                ROS_INFO("Firstrun");
                firstrun = false;

                // Use the first frame as filter
                pcl::fromROSMsg(*PclMsg, filter);
                filter_ptr = filter.makeShared();

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

//            ROS_INFO("indicies %d", newPointIdxVector.size());

            // Fill a filtered cloud after background subtraction
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
            filtered_cloud = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ> ();
            filtered_cloud->points.reserve(newPointIdxVector.size());

            for (std::vector<int>::iterator it = newPointIdxVector.begin (); it != newPointIdxVector.end (); it++)
                filtered_cloud->points.push_back(in_ptr->points[*it]);

            // group points with certain distance to each other
            std::vector<pcl::PointIndices> cluster_indices;
            if(filtered_cloud->size() > MIN_FILTERED_CLOUD_SIZE){
                // Creating the KdTree object for the search method of the extraction
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud (filtered_cloud);

                // ClusterExtraction
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance (0.07); // 7cm
                ec.setMinClusterSize (100);
                ec.setMaxClusterSize (25000);
                ec.setSearchMethod (tree);
                ec.setInputCloud (filtered_cloud);
                ec.extract (cluster_indices);
                ROS_INFO("cluster size %d", cluster_indices.size());
            }

            // just a test publish of interesting pointclouds
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                    cloud_cluster->points.push_back (filtered_cloud->points[*pit]); //*
                ROS_INFO("PointCloud representing the Cluster: %d data points.", cloud_cluster->points.size());

                if(cloud_cluster->points.size() > MIN_CLUSTERED_CLOUD_SIZE){
                    msg.header = PclMsg->header;
                    cloud_cluster->header = PclMsg->header;
                    pcl::toROSMsg(*(cloud_cluster.get()), msg);
                    clusteredPub.publish(msg);
                }
            }


            msg.header = PclMsg->header;
            filtered_cloud->header = PclMsg->header;
            pcl::toROSMsg(*(filtered_cloud.get()), msg);
            filteredPub.publish(msg);

            filter_msg.header = PclMsg->header;
            pcl::toROSMsg(filter, filter_msg);
            filterPub.publish(filter_msg);
        }

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
