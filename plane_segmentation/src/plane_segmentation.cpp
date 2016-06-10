/*
* @copyright: Copyright[2015]<Daniel Angelov>
*      @date: 2016-06-07
*     @email: d.angelov@ed.ac.uk
*      @desc: Segmenting a plane from a pointcloud and publishing any resulting clouds
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <plane_segmentation/PlaneSegmentationParamsConfig.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <vtkRenderWindow.h>

// Colour params
unsigned char colour_r_min;
unsigned char colour_r_max;
unsigned char colour_g_min;
unsigned char colour_g_max;
unsigned char colour_b_min;
unsigned char colour_b_max;

unsigned char colour_r_min_new;
unsigned char colour_r_max_new;
unsigned char colour_g_min_new;
unsigned char colour_g_max_new;
unsigned char colour_b_min_new;
unsigned char colour_b_max_new;

// Params
bool segment_objects;
bool advanced_filter;
float leaf_size_m;
float plane_threshold_m;
float thr_scale;
float simple_threshold_z_plane_height_min;
float simple_threshold_z_plane_height_max;
float neighboring_filter_radius;
float neighboring_filter_n_count;
bool colour_filtering;
bool publish_planes;
bool publish_outliers;
bool euclidean_clustering;
int prefiltering;
int postfiltering_segmented;

// New params
bool should_update_params = true;
bool segment_objects_new;
bool advanced_filter_new;
float leaf_size_m_new;
float plane_threshold_m_new;
float thr_scale_new;
float simple_threshold_z_plane_height_min_new;
float simple_threshold_z_plane_height_max_new;
float neighboring_filter_radius_new;
float neighboring_filter_n_count_new;
bool colour_filtering_new;
bool publish_planes_new;
bool publish_outliers_new;
bool euclidean_clustering_new;
int prefiltering_new;
int postfiltering_segmented_new;

// ROS vars
bool request_pointcloud = false;
ros::Publisher plane_pub;
ros::Publisher outlier_pub;
ros::Publisher segmented_pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(
    new pcl::PointCloud<pcl::PointXYZRGB>),
        cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        outliers_p(new pcl::PointCloud<pcl::PointXYZRGB>);

// PCL Viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer(
    new pcl::visualization::PCLVisualizer("Plane segmentation"));

void updatePCLViewer() {
    pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "source cloud");
    pclViewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(
        cloud_p, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color(
        outliers_p, 0, 0, 255);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(cloud_p, red_color,
                                               "segmented cloud");
    pclViewer->addPointCloud<pcl::PointXYZRGB>(outliers_p, blue_color,
                                               "outliers cloud");
    pclViewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segmented cloud");
    pclViewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outliers cloud");
}

void update_params(bool& should_update) {
    if (!should_update) return;
    segment_objects = segment_objects_new;
    advanced_filter = advanced_filter_new;
    leaf_size_m = leaf_size_m_new;
    plane_threshold_m = plane_threshold_m_new;
    thr_scale = thr_scale_new;
    simple_threshold_z_plane_height_min = simple_threshold_z_plane_height_min_new;
    simple_threshold_z_plane_height_max = simple_threshold_z_plane_height_max_new;
    neighboring_filter_radius = neighboring_filter_radius_new;
    neighboring_filter_n_count = neighboring_filter_n_count_new;
    colour_filtering = colour_filtering_new;
    publish_planes = publish_planes_new;
    publish_outliers = publish_outliers_new;
    colour_r_min = colour_r_min_new;
    colour_r_max = colour_r_max_new;
    colour_g_min = colour_g_min_new;
    colour_g_max = colour_g_max_new;
    colour_b_min = colour_b_min_new;
    colour_b_max = colour_b_max_new;
    euclidean_clustering = euclidean_clustering_new;
    prefiltering = prefiltering_new;
    postfiltering_segmented = postfiltering_segmented_new;

    should_update = false;
    ROS_INFO_STREAM("Updated params correcty!");
}

// Callback on new cloud
pcl::PCLPointCloud2::ConstPtr pc_msg;
void new_cloud_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    ROS_INFO_STREAM("Got a new PointCloud!!!");
    pc_msg = msg;
}

void new_cloud_2_process(const pcl::PCLPointCloud2::ConstPtr& msg) {
    ROS_INFO_STREAM("---");
    ros::Time s_ros, e_ros, total_time;
    total_time = ros::Time::now();
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::fromPCLPointCloud2(*msg, *cloud);

    // Create the filtering object: downsample the dataset using a leaf size of 0.5cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(msg);
    sor.setLeafSize(leaf_size_m, leaf_size_m, leaf_size_m); //meters 0.005
    pcl::PCLPointCloud2 cloud_filtered;
    sor.filter(cloud_filtered);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new
                                                           pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud_filtered, *cloud_filtered2);
    e_ros = ros::Time::now();
    ROS_INFO(">> CPU Time VoxelGrid: %.2fms.",
             (e_ros - total_time).toNSec() * 1e-6);

    // Filter all
    if (prefiltering != -1) {
        s_ros = ros::Time::now();
        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> fitlering_obj_all;
        fitlering_obj_all.setInputCloud(cloud_filtered2);
        fitlering_obj_all.setMeanK(prefiltering); // neighboring points to query
        fitlering_obj_all.setStddevMulThresh(1.0); //std deviation
        fitlering_obj_all.filter(*cloud_filtered2); // cloud_filtered

        e_ros = ros::Time::now();
        ROS_INFO(">> CPU Time all prefiltering: %.2fms.",
                 (e_ros - s_ros).toNSec() * 1e-6);
    }


    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    if (publish_planes || publish_outliers || segment_objects) {
        s_ros = ros::Time::now();

        // Extract
        // Create the segmentation object
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
        pcl::SACSegmentation<pcl::PointXYZRGB> sacs_segm;
        sacs_segm.setOptimizeCoefficients(true);
        sacs_segm.setModelType(pcl::SACMODEL_PLANE);
        sacs_segm.setMethodType(pcl::SAC_RANSAC);
        sacs_segm.setMaxIterations(1000); // TODO: export it as a parameter
        sacs_segm.setDistanceThreshold(plane_threshold_m); // 0.01m
        sacs_segm.setInputCloud(cloud_filtered2);
        sacs_segm.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            ROS_WARN_STREAM("Could not estimate a planar model for the given dataset.");
            return;
        }
        ROS_DEBUG_STREAM("Estimated a planar model!");


        // Extract the inliers
        extract.setInputCloud(cloud_filtered2);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        e_ros = ros::Time::now();
        ROS_INFO(">> CPU Time Plane extraction: %.2fms.",
                 (e_ros - s_ros).toNSec() * 1e-6);

        ROS_DEBUG_STREAM("PointCloud representing the planar component: " <<
                         cloud_p->width * cloud_p->height << " data points. From a total of : " <<
                         cloud_filtered2->points.size());
    }
    if (publish_outliers) {
        s_ros = ros::Time::now();

        // Fill in the cloud data for the outliers
        extract.setNegative(true);
        extract.filter(*outliers_p);

        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> fitlering_obj;
        fitlering_obj.setInputCloud(outliers_p);
        fitlering_obj.setMeanK(50); // neighboring points to query
        fitlering_obj.setStddevMulThresh(1.0); //std deviation
        fitlering_obj.filter(*outliers_p); // cloud_filtered
        e_ros = ros::Time::now();
        ROS_INFO(">> CPU Time Outlier extraction: %.2fms.",
                 (e_ros - s_ros).toNSec() * 1e-6);

    }

    if (segment_objects) {
        ROS_DEBUG_STREAM("T: " << segment_objects);
        // Begin thresholding
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new
                                                       pcl::PointCloud<pcl::PointXYZRGB>);
        if (advanced_filter) {
            ROS_DEBUG_STREAM("Using advanced filter.");
            s_ros = ros::Time::now();
            // Copy the points of the plane to a new cloud.
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new
                                                         pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ExtractIndices<pcl::PointXYZRGB> extract_hull;
            extract_hull.setInputCloud(cloud_filtered2);
            // TODO: There is a setKeepOrganized(true); field that doesnt remove the non existing points.
            // This may be used to avoid processing the whole field again.
            extract_hull.setIndices(inliers);
            extract_hull.filter(*plane);

            // Retrieve the convex hull.
            pcl::ConvexHull<pcl::PointXYZRGB> hull;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr convexHull(new
                                                              pcl::PointCloud<pcl::PointXYZRGB>);
            hull.setInputCloud(plane);
            // Make sure that the resulting hull is bidimensional.
            hull.setDimension(2);
            hull.reconstruct(*convexHull);
            // Redundant check.
            if (hull.getDimension() == 2) {
                // Prism object.
                pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;

                prism.setInputCloud(cloud_filtered2);
                prism.setInputPlanarHull(convexHull);
                // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
                // Second parameter: maximum Z value, set to 1.50m. Tune it according to the height of the objects you expect.
                prism.setHeightLimits(plane_threshold_m * thr_scale,
                                      simple_threshold_z_plane_height_max);
                pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

                prism.segment(*objectIndices);

                if (objectIndices->indices.size() > 0) {
                    ROS_DEBUG_STREAM("Found some points in the convex hull prism.");
                    // Get and show all points retrieved by the hull.
                    extract_hull.setIndices(objectIndices);
                    extract_hull.filter(*objects);
                }
            } else {
                ROS_WARN_STREAM("The chosen hull is not planar.");
            }
            e_ros = ros::Time::now();
            ROS_INFO(">> CPU Time hull filtering: %.2fms.",
                     (e_ros - s_ros).toNSec() * 1e-6);

        } else {
            // Simple statistical filtering
            ROS_INFO_STREAM("Passing though simple thresholding.");
            ROS_WARN_STREAM("SIMPLE DOES NOT RESPECT THE TF of the pointcloud. " <<
                            "It assumes z to be along the camera_depth_optical_frame");
            s_ros = ros::Time::now();
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud(cloud_filtered2);
            pass.setFilterFieldName("z");
            pass.setFilterLimitsNegative(false);
            // accept things in the range below
            pass.setFilterLimits(simple_threshold_z_plane_height_min,
                                 simple_threshold_z_plane_height_max);
            pass.filter(*objects);
            e_ros = ros::Time::now();
            ROS_INFO(">> CPU Time simple filtering: %.2fms.",
                     (e_ros - s_ros).toNSec() * 1e-6);

        }

        if (colour_filtering) {
            s_ros = ros::Time::now();
            pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(
                new pcl::ConditionAnd<pcl::PointXYZRGB>());
            range_cond->addComparison(
                pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                    new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT,
                                                                   colour_r_min)));
            range_cond->addComparison(
                pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                    new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT,
                                                                   colour_r_max)));
            range_cond->addComparison(
                pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                    new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT,
                                                                   colour_g_min)));
            range_cond->addComparison(
                pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                    new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT,
                                                                   colour_g_max)));
            range_cond->addComparison(
                pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                    new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b",
                                                                   pcl::ComparisonOps::GT,
                                                                   colour_b_min)));
            range_cond->addComparison(
                pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                    new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT,
                                                                   colour_b_max)));
            // build the filter
            pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
            condrem.setCondition(range_cond);
            condrem.setInputCloud(objects);

            // condrem.setIndices(inliers); // TEST: JUST FOR TESTING REMOVE AFTER COMPILING
            condrem.setKeepOrganized(false); //was true
            condrem.filter(*objects);

            e_ros = ros::Time::now();
            ROS_INFO(">> CPU Time colour filtering: %.2fms.",
                     (e_ros - s_ros).toNSec() * 1e-6);
        }

        if (postfiltering_segmented != -1) {
            // Post filtering
            s_ros = ros::Time::now();
            // Remove singled out points
            for (int i = 0 ; i < 2; ++i) {
                pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
                // build the filter
                outrem.setInputCloud(objects);
                outrem.setRadiusSearch(neighboring_filter_radius); // 0.005 = 5 mm

                // a total of how many points should be in the neighborhood
                outrem.setMinNeighborsInRadius(neighboring_filter_n_count);
                // apply filter
                outrem.filter(*objects);
            }

            // Apply some outlier filtering
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> segm_obj_fitler;
            segm_obj_fitler.setInputCloud(objects);
            segm_obj_fitler.setMeanK(
                postfiltering_segmented); // neighboring points to query
            segm_obj_fitler.setStddevMulThresh(1.0); //std deviation
            segm_obj_fitler.filter(*objects); // cloud_filtered

            e_ros = ros::Time::now();
            ROS_INFO(">> CPU Time postfiltering: %.2fms.", (e_ros - s_ros).toNSec() * 1e-6);
        }

        if (euclidean_clustering) {
            // clock_t tStart = clock();
            // // Creating the KdTree object for the search method of the extraction
            // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new
            //                                                  pcl::search::KdTree<pcl::PointXYZRGB>);
            // tree->setInputCloud(objects);

            // std::vector<pcl::PointIndices> cluster_indices;
            // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            // ec.setClusterTolerance(0.02); // 2cm
            // ec.setMinClusterSize(100);
            // ec.setMaxClusterSize(25000);
            // ec.setSearchMethod(tree);
            // ec.setInputCloud(objects);
            // ec.extract(cluster_indices);

            // ROS_INFO(">> CPU Time euclidean clustering: %.2fms",
            //          (double)(clock() - tStart) / CLOCKS_PER_SEC * 1000);
            // ROS_INFO_STREAM("Found " << cluster_indices.size() << " clusters.");
            /*
            std::vector<int> ivec;
            std::iota(ivec.begin(), ivec.end(), 0); // 0, 1, 2, 3, 4 ...
            */
        }

        s_ros = ros::Time::now();
        ROS_DEBUG_STREAM("Converting and sending segmented objects!");
        sensor_msgs::PointCloud2 segm_obj;
        pcl::toROSMsg(*objects, segm_obj);
        // segm_obj.header.stamp = ros::Time::now();
        segmented_pub.publish(segm_obj);
        e_ros = ros::Time::now();
        ROS_INFO(">> CPU Time publishing segmented obj: %.2fms.",
                 (e_ros - s_ros).toNSec() * 1e-6);
        ROS_DEBUG_STREAM("published segmented objects");
        // End thresholding
    }

    // // TODO: Creating the KdTree object for the search method of the extraction
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // tree->setInputCloud(cloud_filtered);

    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ec.setClusterTolerance(0.02); // 2cm
    // ec.setMinClusterSize(100);
    // ec.setMaxClusterSize(25000);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud_filtered);
    // ec.extract(cluster_indices);

    // int j = 0;
    // for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    // {
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    //   for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    //     cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
    //   cloud_cluster->width = cloud_cluster->points.size();
    //   cloud_cluster->height = 1;
    //   cloud_cluster->is_dense = true;

    //   std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
    //   std::stringstream ss;
    //   ss << "cloud_cluster_" << j << ".pcd";
    //   writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
    //   j++;
    // }

    s_ros = ros::Time::now();
    pcl::PCLPointCloud2 segmented_pcl, outliers_pcl;
    sensor_msgs::PointCloud2 segmented, segmented_outliers;
    if (publish_planes) {
        ROS_DEBUG_STREAM("Publishing plane!\n");
        pcl::toPCLPointCloud2(*cloud_p, segmented_pcl);
        pcl_conversions::fromPCL(segmented_pcl, segmented);
        plane_pub.publish(segmented);
    }
    if (publish_outliers) {
        pcl::toPCLPointCloud2(*outliers_p, outliers_pcl);
        pcl_conversions::fromPCL(outliers_pcl, segmented_outliers);
        outlier_pub.publish(segmented_outliers);
    }
    e_ros = ros::Time::now();
    ROS_INFO(">> CPU Time publishing plane/outliers: %.2fms.",
             (e_ros - s_ros).toNSec() * 1e-6);
    ROS_INFO(">> CPU Time Total: %.2fms.",
             (ros::Time::now() - total_time).toNSec() * 1e-6);
    ROS_INFO("---");
    //Update the viewer
    updatePCLViewer();
}

void dynamic_recongifure_callback(
    plane_segmentation::PlaneSegmentationParamsConfig& config,
    uint32_t level) {
    // Lazy man: change all. Otherwise could use the mask
    segment_objects_new =  config.segment_objects;
    switch (config.extraction_method) {
        case 0: advanced_filter_new = false; break; // naive
        case 1: advanced_filter_new = true;  break; // adv
        default: ROS_WARN_STREAM("Wrong advanced_filter id. Should be 0/1");
    }
    leaf_size_m_new = config.leaf_size_m;
    plane_threshold_m_new = config.plane_threshold_m;
    thr_scale_new = config.threshold_scale;
    simple_threshold_z_plane_height_min_new =
        config.simple_threshold_z_plane_height_min;
    simple_threshold_z_plane_height_max_new =
        config.simple_threshold_z_plane_height_max;
    neighboring_filter_radius_new = config.neighboring_filter_radius;
    neighboring_filter_n_count_new = config.neighboring_filter_n_count;
    colour_filtering_new = config.colour_filtering;
    publish_planes_new = config.publish_planes;
    publish_outliers_new = config.publish_outliers;
    colour_r_min_new = config.colour_r_min;
    colour_r_max_new = config.colour_r_max;
    colour_g_min_new = config.colour_g_min;
    colour_g_max_new = config.colour_g_max;
    colour_b_min_new = config.colour_b_min;
    colour_b_max_new = config.colour_b_max;
    euclidean_clustering_new = config.euclidean_clustering;
    prefiltering_new = config.prefiltering;
    postfiltering_segmented_new = config.postfiltering_segmented;

    // Indicate new params need to be read
    should_update_params = true;

    ROS_DEBUG_STREAM("Reconfigure Request:" <<
                     segment_objects_new << " " <<
                     advanced_filter_new << " " <<
                     leaf_size_m_new << " " <<
                     plane_threshold_m_new << " " <<
                     simple_threshold_z_plane_height_min_new << " " <<
                     simple_threshold_z_plane_height_max_new << " " <<
                     neighboring_filter_radius_new << " " <<
                     neighboring_filter_n_count_new << " " <<
                     colour_filtering_new << " " <<
                     publish_planes_new << " " <<
                     publish_outliers_new << " " <<
                     euclidean_clustering << " " <<
                     prefiltering_new << " " <<
                     postfiltering_segmented_new);
}

bool request_pointcloud_callback(
    std_srvs::Empty::Request&  req,
    std_srvs::Empty::Response& res) {
    request_pointcloud = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_segmentation");
    ros::NodeHandle nh("~");

    // Load params
    // nh.param<bool>("segment_objects", segment_objects, true);
    // nh.param<bool>("advanced_filter", advanced_filter, false);
    // nh.param<float>("leaf_size_m", leaf_size_m, 0.0015);
    // nh.param<float>("plane_threshold_m", plane_threshold_m, 0.005);
    // nh.param<float>("simple_threshold_z_plane_height_min",
    //                 simple_threshold_z_plane_height_min, 0.7f);
    // nh.param<float>("simple_threshold_z_plane_height_max",
    //                 simple_threshold_z_plane_height_max, 3.0f);

    dynamic_reconfigure::Server<plane_segmentation::PlaneSegmentationParamsConfig>
    server;
    dynamic_reconfigure::Server<plane_segmentation::PlaneSegmentationParamsConfig>::CallbackType
    f;
    f = boost::bind(&dynamic_recongifure_callback, _1, _2);
    server.setCallback(f);

    //Possible pointclouds: "/kinect2/sd/points" //xtion: "/camera/depth_registered/points"
    ros::Subscriber sub = nh.subscribe("/kinect2/sd/points", 1, new_cloud_callback);

    // Create a ROS publisher for the output point cloud
    plane_pub = nh.advertise<sensor_msgs::PointCloud2>("extracted_planes", 1);
    outlier_pub = nh.advertise<sensor_msgs::PointCloud2>("extracted_outliers", 1);
    segmented_pub =
        nh.advertise<sensor_msgs::PointCloud2>("segmented_objects_above", 1);
    ros::ServiceServer request_pointcloud_service =
        nh.advertiseService("request_pointcloud",
                            request_pointcloud_callback);

    //PCL Viewer
    pclViewer->setBackgroundColor(0, 0, 0);
    pclViewer->initCameraParameters();
    pclViewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
    vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
    renderWindow->SetSize(640, 480);
    renderWindow->Render();

    ros::Rate r(30);
    while (ros::ok() && !pclViewer->wasStopped()) {
        ros::spinOnce();
        update_params(should_update_params);
        if (request_pointcloud) {
            ROS_INFO_STREAM("Processing new pointcloud");
            new_cloud_2_process(pc_msg);
            request_pointcloud = false;
        }
        pclViewer->spinOnce(100);
        r.sleep();
    }
    return 0;
}
