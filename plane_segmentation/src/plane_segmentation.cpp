/*
* @copyright: Copyright[2015]<Daniel Angelov>
*      @date: 2016-06-07
*     @email: d.angelov@ed.ac.uk
*      @desc: Segmenting a plane from a pointcloud and publishing any resulting clouds
*/

#include <cstddef>
#include <thread>

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pr2_picknplace_msgs/SegmentedObject.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <plane_segmentation/PlaneSegmentationParamsConfig.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

#include <pcl/common/distances.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <vtkRenderWindow.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

#include <Eigen/Dense>

// Colour params
unsigned char colour_gray_min;
unsigned char colour_gray_max;
unsigned char colour_g_min;
unsigned char colour_g_max;
unsigned char colour_b_min;
unsigned char colour_b_max;

unsigned char colour_gray_min_new;
unsigned char colour_gray_max_new;
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
bool mesh_save;
bool mesh_view;
int meshing_method;
double max_cluster_dist;

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
bool mesh_save_new;
bool mesh_view_new;
int meshing_method_new;
double max_cluster_dist_new;

// ROS vars
std::string world_frame;
bool subscribe_to_filtered;
tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;
bool request_pointcloud = false;
ros::Publisher plane_pub;
ros::Publisher outlier_pub;
ros::Publisher segmented_pub;
ros::Publisher cluster_pub;
ros::Publisher table_head_track_pub;
ros::Subscriber pointcloud_sub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(
    new pcl::PointCloud<pcl::PointXYZRGB>),
        cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        outliers_p(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PolygonMesh mesh;

// PCL Viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer(
    new pcl::visualization::PCLVisualizer("Plane segmentation"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> meshViewer;

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

double mean_to_point_dist(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                          cloud, pcl::PointIndices& idxs, Eigen::Vector4d point,
                          Eigen::Affine3d transform) {
    Eigen::Vector4d centroid;
    if (pcl::compute3DCentroid(*cloud, idxs, centroid) != idxs.indices.size()) {
        ROS_WARN_STREAM("Cannot find centre of cluster");
    }
    centroid(3) = 1.0;
    ROS_DEBUG_STREAM("[mean_to_point_dist] tf-ed point: " << transform * centroid);
    return (transform * centroid - point).norm();
}

double rgb2gray(const pcl::PointXYZRGB& p) {
    return double(p.r + p.g + p.b) / 3.0;
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
    colour_gray_min = colour_gray_min_new;
    colour_gray_max = colour_gray_max_new;
    euclidean_clustering = euclidean_clustering_new;
    prefiltering = prefiltering_new;
    postfiltering_segmented = postfiltering_segmented_new;
    mesh_save = mesh_save_new;
    mesh_view = mesh_view_new;
    meshing_method = meshing_method_new;
    max_cluster_dist = max_cluster_dist_new;

    should_update = false;
    ROS_INFO_STREAM("Updated params correcty!");
}

// Callback on new cloud
pcl::PCLPointCloud2::ConstPtr pc_msg;
void new_cloud_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    ROS_INFO_STREAM("Got a new PointCloud!!!");
    pc_msg = msg;
}

void remove_nans(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr fc(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (pcl_isfinite(cloud->points[i].x) &&
                pcl_isfinite(cloud->points[i].y) &&
                pcl_isfinite(cloud->points[i].z)) {
            // Add point
            fc->push_back(cloud->points[i]);
        }
    }
    cloud = fc;
}


void refine_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    ros::Time s_ros = ros::Time::now();
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(0.01);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setUpsamplingMethod(
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.005);
    mls.setUpsamplingStepSize(0.003);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(
        new pcl::PointCloud<pcl::PointXYZ>());
    mls.process(*cloud_smoothed);
    remove_nans(cloud_smoothed);
    for (int i = 0; i < cloud_smoothed->size(); ++i) {
        if (!pcl_isfinite(cloud_smoothed->points[i].x))
            std::cout << cloud_smoothed->points[i] << std::endl;
    }

    ROS_INFO_STREAM("Cloud smoothed: " << cloud_smoothed->size());

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(cloud_smoothed);
    ne.setRadiusSearch(0.005);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_smoothed, centroid);
    ROS_INFO_STREAM("centroid:: " << centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
        new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);

    for (size_t i = 0; i < cloud_normals->size(); ++i) {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(
        new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud_smoothed, *cloud_normals,
                           *cloud_smoothed_normals);
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    // PolygonMesh mesh;
    poisson.reconstruct(mesh);

    ros::Time e_ros = ros::Time::now();
    ROS_INFO(">> CPU Time mesh refinement: %.2fms.",
             (e_ros - s_ros).toNSec() * 1e-6);
}

void generate_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    ros::Time s_ros = ros::Time::now();

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(
        new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);

    //
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    n.setViewPoint(centroid[0], centroid[1], centroid[2]);
    //
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
        new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
        new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    // Set the maximum distance between connected points(maximum edge length)
    gp3.setSearchRadius(0.025);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(400);
    gp3.setMaximumSurfaceAngle(M_PI / 4); //  /4 = 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(mesh);
    ros::Time e_ros = ros::Time::now();
    ROS_INFO(">> CPU Time mesh generation: %.2fms.",
             (e_ros - s_ros).toNSec() * 1e-6);
}

void add_floor_2_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                      pcl::ModelCoefficients::Ptr coefficients) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    *cloud += *cloud_projected;
}

void new_cloud_2_process(const pcl::PCLPointCloud2::ConstPtr& msg) {
    ROS_INFO_STREAM("---");
    ros::Time s_ros, e_ros, total_time;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    total_time = ros::Time::now();
    if (leaf_size_m != 0) {
        // Create the filtering object: downsample the dataset using a leaf size of 0.5cm
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(msg);
        sor.setLeafSize(leaf_size_m, leaf_size_m, leaf_size_m); //meters 0.005
        pcl::PCLPointCloud2 cloud_filtered;
        sor.filter(cloud_filtered);

        pcl::fromPCLPointCloud2(cloud_filtered, *cloud_filtered2);
    } else {
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::fromPCLPointCloud2(*msg, *cloud_filtered2);
    }
    e_ros = ros::Time::now();
    ROS_INFO(">> CPU Time VoxelGrid or msg cpy: %.2fms.",
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
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    if (publish_planes || publish_outliers || segment_objects) {
        s_ros = ros::Time::now();

        // Extract
        // Create the segmentation object
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
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr convexHull(
                new pcl::PointCloud<pcl::PointXYZRGB>);
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
                // First parameter: minimum Z value. Set to 0, segments objects lying on the plane(can be negative).
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

        if (colour_filtering && objects->size() > 0) {
            s_ros = ros::Time::now();
            pcl::PointIndices::Ptr idxs(new pcl::PointIndices());
            // iota(idxs->indices.begin(), idxs->indices.end(), 0);

            for (int i = 0; i < objects->size(); ++i) {
                double gray = rgb2gray(objects->points[i]);
                if (gray >= colour_gray_min && gray <= colour_gray_max)
                    idxs->indices.push_back(i);
            }
            extract.setInputCloud(objects);
            extract.setIndices(idxs);
            extract.setNegative(false);
            extract.filter(*objects);

            e_ros = ros::Time::now();
            ROS_INFO(">> CPU Time colour filtering: %.2fms.",
                     (e_ros - s_ros).toNSec() * 1e-6);
        }

        if (postfiltering_segmented != -1 && objects->size() > 0) {
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

        if (euclidean_clustering && objects->size() > 0) {
            clock_t tStart = clock();
            // Spawn a new thread to find tf!
            Eigen::Affine3d eigen_transf;
            std::thread tf_thr([&eigen_transf, &msg]() ->void {
                geometry_msgs::TransformStamped transform;
                try{
                    transform = tfBuffer->lookupTransform(world_frame,
                    msg->header.frame_id,
                    ros::Time(0), //acquisition_time - ros::Duration().fromSec(4),
                    ros::Duration(0.2) );
                } catch (tf2::TransformException ex) {
                    ROS_ERROR("Error tf lookup %s", ex.what());
                }
                eigen_transf = tf2::transformToEigen(transform);
            });

            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
                new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud(objects);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            // TODO: Extract the numbers as dynamic param
            ec.setClusterTolerance(0.02); // 2cm
            ec.setMinClusterSize(100);
            ec.setMaxClusterSize(250000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(objects);
            ec.extract(cluster_indices);
            ROS_INFO_STREAM("Found " << cluster_indices.size() << " clusters.");

            // Join tf thread
            tf_thr.join();

            ROS_INFO(">> CPU Time euclidean clustering: %.2fms",
                     (double)(clock() - tStart) / CLOCKS_PER_SEC * 1000);

            if (cluster_indices.size() > 0) {
                // Get a cluster
                tStart = clock();
                Eigen::Vector4d master_point(0.5, -0.2, 0.75, 1);
                double min_dist = DBL_MAX;
                int min_idx = 0;
                for (int i = 0; i < cluster_indices.size(); ++i) {
                    double dist = mean_to_point_dist(objects, cluster_indices[i],
                                                     master_point, eigen_transf);
                    if (dist < min_dist) {
                        min_dist = dist;
                        min_idx = i;
                    }
                }

                ROS_INFO_STREAM("min_dist of cluster: " << min_dist);
                if (min_dist <= max_cluster_dist) {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cobj(
                        new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cobj_world_frame(
                        new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::PointIndices::Ptr idxs(new pcl::PointIndices(cluster_indices[min_idx]));
                    extract.setInputCloud(objects);
                    extract.setIndices(idxs);
                    extract.setNegative(false);
                    extract.filter(*cobj);

                    add_floor_2_mesh(cobj, coefficients);

                    // Transform
                    pcl::transformPointCloud(*cobj, *cobj_world_frame, eigen_transf);

                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*cobj_world_frame, centroid);
                    ROS_DEBUG_STREAM("Evaluated centroid: " << centroid);

                    Eigen::Vector4f minPoint, maxPoint;
                    pcl::getMinMax3D(*cobj_world_frame, minPoint, maxPoint);
                    ROS_DEBUG_STREAM("min max points: " << minPoint << " " << maxPoint);

                    sensor_msgs::PointCloud2 segm_obj_world_frame;
                    pcl::toROSMsg(*cobj_world_frame, segm_obj_world_frame);
                    segm_obj_world_frame.header.frame_id = world_frame;
                    segm_obj_world_frame.header.stamp = ros::Time::now();

                    pr2_picknplace_msgs::SegmentedObject segm_msg;
                    segm_msg.cloud = segm_obj_world_frame;
                    segm_msg.centroid.x = centroid.x();
                    segm_msg.centroid.y = centroid.y();
                    segm_msg.centroid.z = centroid.z();
                    segm_msg.size.x = maxPoint.x() - minPoint.x();
                    segm_msg.size.y = maxPoint.y() - minPoint.y();
                    segm_msg.size.z = maxPoint.z() - minPoint.z();
                    cluster_pub.publish(segm_msg);
                    ROS_INFO(">> CPU Time cluster selection and publishing: %.2fms",
                             (double)(clock() - tStart) / CLOCKS_PER_SEC * 1000);

                    if (mesh_save) {
                        bool binary_mode = false;
                        pcl::io::savePLYFile ("/tmp/obj.ply", *cobj_world_frame, binary_mode);
                    }
                    if (mesh_view) {
                        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cobj(
                            new pcl::PointCloud<pcl::PointXYZ>);
                        pcl::copyPointCloud(*cobj_world_frame, *xyz_cobj);
                        if (meshing_method == 0)
                            generate_mesh(xyz_cobj);
                        else if (meshing_method == 1) {
                            refine_mesh(xyz_cobj);
                        } else {
                            ROS_WARN_STREAM("Wrong meshing index.");
                        }
                    }
                }
            }
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

void start_mesh_viewer() {
    ROS_INFO_STREAM("Starting the mesh_viewer");
    mesh_view = (new pcl::visualization::PCLVisualizer("3D mesh viewer"));
    meshViewer->setBackgroundColor(0, 0, 0);
    meshViewer->initCameraParameters();
    // meshViewer->addCoordinateSystem(1.0);
    meshViewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
    vtkSmartPointer<vtkRenderWindow> renderWindow_mesh =
        meshViewer->getRenderWindow();
    renderWindow_mesh->SetSize(640, 480);
    renderWindow_mesh->Render();
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
    colour_gray_min_new = config.colour_gray_min;
    colour_gray_max_new = config.colour_gray_max;
    euclidean_clustering_new = config.euclidean_clustering;
    prefiltering_new = config.prefiltering;
    postfiltering_segmented_new = config.postfiltering_segmented;
    mesh_save_new = config.mesh_save;
    mesh_view_new = config.mesh_view;
    meshing_method_new = config.meshing_method;
    max_cluster_dist_new = config.max_cluster_dist;

    // Indicate new params need to be read
    should_update_params = true;

    // Show viewer if needed
    if (mesh_view_new && !mesh_view) start_mesh_viewer();

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
                     postfiltering_segmented_new << " " <<
                     max_cluster_dist_new);
}

bool request_pointcloud_callback(
    std_srvs::Empty::Request&    req,
    std_srvs::Empty::Response& res) {
    ROS_INFO_STREAM("Received a request for processing!");
    request_pointcloud = true;
    return true;
}

void point2tabletop(ros::NodeHandle& nh) {
    table_head_track_pub  =
        nh.advertise<std_msgs::String>("/pr2_head/target_object", 5, true);
    std_msgs::StringPtr str(new std_msgs::String);
    str->data = "tabletop";
    table_head_track_pub.publish(str);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_segmentation");
    ros::NodeHandle nh("~");

    tfBuffer = new tf2_ros::Buffer();
    tfListener = new tf2_ros::TransformListener(*tfBuffer);
    dynamic_reconfigure::Server<plane_segmentation::PlaneSegmentationParamsConfig>
    server;
    dynamic_reconfigure::Server<plane_segmentation::PlaneSegmentationParamsConfig>::CallbackType
    f;
    f = boost::bind(&dynamic_recongifure_callback, _1, _2);
    server.setCallback(f);

    nh.param<std::string>("world_frame", world_frame, "base_link");
    nh.param<bool>("subscribe_to_filtered", subscribe_to_filtered, false);

    // Possible pointclouds: "/kinect2/hd/points" //xtion: "/camera/depth_registered/points"
    // Pro tip: Use qhd or hd topic, as the sd pointcloud has an offset in y direction.
    pointcloud_sub = (subscribe_to_filtered) ?
                     nh.subscribe("/kinect2/qhd/points/filtered", 1, new_cloud_callback) :
                     nh.subscribe("/kinect2/qhd/points", 1, new_cloud_callback);
    //pointcloud_sub = nh.subscribe("/camera/depth_registered/points", 1, new_cloud_callback);

    // Send message to get the head to rotate to the table
    point2tabletop(nh);

    // Create a ROS publisher for the output point cloud
    plane_pub = nh.advertise<sensor_msgs::PointCloud2>("extracted_planes", 1);
    outlier_pub = nh.advertise<sensor_msgs::PointCloud2>("extracted_outliers", 1);
    segmented_pub =
        nh.advertise<sensor_msgs::PointCloud2>("segmented_objects_above", 1);
    cluster_pub =
        nh.advertise<pr2_picknplace_msgs::SegmentedObject>("clustered_object", 1);
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

    if (mesh_view) start_mesh_viewer();

    ROS_WARN_STREAM("Starting spin");
    ros::Rate r(30);
    while (ros::ok() && !pclViewer->wasStopped() /*&& !meshViewer->wasStopped()*/) {
        ros::spinOnce();
        update_params(should_update_params);
        if (request_pointcloud && pc_msg != nullptr) {
            ROS_INFO_STREAM("Processing new pointcloud");
            new_cloud_2_process(pc_msg);
            request_pointcloud = false;

            if (mesh_view) {
                meshViewer->removePolygonMesh("meshes");
                meshViewer->addPolygonMesh(mesh, "meshes", 0);
            }
        }
        pclViewer->spinOnce(100);
        if (mesh_view)
            meshViewer->spinOnce(100);
        r.sleep();
    }

    delete tfBuffer;
    delete tfListener;
    return 0;
}
