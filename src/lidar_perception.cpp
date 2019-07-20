/**
 * @file: lidar_perception.cpp
 * @author: Z.H
 * @date: 2019.05.9
 */
#include "lidar_perception/lidar_perception.h"

static bool point_cmp(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.z < b.z;
}


LidarPerception::LidarPerception():
			    left_cloud_(new PointCloud()), right_cloud_(new PointCloud()),
			  center_cloud_(new PointCloud()),filter_cloud_(new PointCloud()),
	left_transfor_cloud_ptr_(new PointCloud()), seed_cloud_(new PointCloud()),
 right_transfor_cloud_ptr_(new PointCloud()), ground_cloud_(new PointCloud()),
     unsegmented_cloud_(new PointCloud()), no_ground_cloud_(new PointCloud())			  					  
{
			  					  	
	ROS_WARN("begin to run!");
}

LidarPerception::~LidarPerception() {
	ROS_WARN("begin to end!");
	ros::shutdown();
}

Eigen::Affine3f LidarPerception::getTransformMat(const std::vector<double> &params)
{
	Eigen::Affine3f result = Eigen::Affine3f::Identity();

	result.translation() << params[0], params[1], params[2];

	result.rotate (Eigen::AngleAxisf(params[3] * PI_OVER_180, Eigen::Vector3f::UnitX())
	             * Eigen::AngleAxisf(params[4] * PI_OVER_180, Eigen::Vector3f::UnitY())
	             * Eigen::AngleAxisf(params[5] * PI_OVER_180, Eigen::Vector3f::UnitZ()));

	return result;
}

void LidarPerception::init()
{
	//get params
	nh_.getParam("/lidar_perception/left_lidar_topic", left_lidar_topic_);
	nh_.getParam("/lidar_perception/right_lidar_topic", right_lidar_topic_);
	nh_.getParam("/lidar_perception/center_lidar_topic", center_lidar_topic_);
	nh_.getParam("/lidar_perception/no_ground_topic", no_ground_topic_);
	nh_.getParam("/lidar_perception/detecting_bbox_topic", detecting_bbox_topic_);
	nh_.getParam("/lidar_perception/grid_map_topic", grid_map_topic_);

	nh_.getParam("/lidar_perception/roi/x_min", x_min_);
	nh_.getParam("/lidar_perception/roi/y_min", y_min_);
	nh_.getParam("/lidar_perception/roi/z_min", z_min_);
    nh_.getParam("/lidar_perception/roi/x_max", x_max_);
    nh_.getParam("/lidar_perception/roi/y_max", y_max_);
    nh_.getParam("/lidar_perception/roi/z_max", z_max_);

    nh_.getParam("/lidar_perception/map_resolution", map_resolution_);
    nh_.getParam("/lidar_perception/car_info/car_width", car_width_);
    nh_.getParam("/lidar_perception/car_info/car_length", car_length_);
    nh_.getParam("/lidar_perception/lidar_height", lidar_height_);

	nh_.param("/lidar_perception/trans_params/left", 
		                left_trans_params_, std::vector<double>(0));
	nh_.param("/lidar_perception/trans_params/right", 
		               right_trans_params_, std::vector<double>(0));

	if(left_trans_params_.empty() || right_trans_params_.empty())
    	ROS_ERROR("lidar trans_params load failure!!!");

    // get transform_mat
	left_transform_mat_ = getTransformMat(left_trans_params_);
	right_transform_mat_ = getTransformMat(right_trans_params_);
	std::cout << "left_transform_mat:" << std::endl << left_transform_mat_.matrix() << std::endl;
	std::cout << "right_transform_mat:" << std::endl << right_transform_mat_.matrix() << std::endl;

	//set map roi value
	map_x_min_ = 0;
	map_x_max_ = (int)((x_max_ - x_min_)/map_resolution_);
	map_y_min_ = 0;
	map_y_max_ = (int)((y_max_ - y_min_)/map_resolution_);

	//initialize cluster segment
	seg_distance_ = {10, 20, 30, 40};
    cluster_distance_ = {0.5, 1.0, 1.5, 2.0, 2.5};
    height_threshold_ = 1.2 * lidar_height_;
  
    //sub and pub
    left_lidar_sub_ = nh_.subscribe(left_lidar_topic_, 10, 
    	              &LidarPerception::subLeftCallback, this);
    right_lidar_sub_ = nh_.subscribe(right_lidar_topic_, 10, 
    	              &LidarPerception::subRightCallback, this);

    center_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(center_lidar_topic_, 10, this);
    grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(grid_map_topic_, 1, this);
    no_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(no_ground_topic_, 10, this);
    detecting_bbox_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(detecting_bbox_topic_, 10, this);

}


void LidarPerception::subLeftCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *left_cloud_);
    //ROS_INFO("left point size %ld ", left_cloud_->points.size());
}

void LidarPerception::subRightCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *right_cloud_);
    //ROS_INFO("right point size %ld ", right_cloud_->points.size());
}

void LidarPerception::makeGridMap(const PointCloudPtr &cloudtogrid)
{
	ROS_INFO("begin to make grid map!!!");
	nav_msgs::OccupancyGrid map;
    
	map.header.frame_id = "rslidar";
  	map.header.stamp = ros::Time::now();

	int map_width = (int)(map_x_max_ - map_x_min_);
	int map_height = (int)(map_y_max_ - map_y_min_);
	int map_size = map_width * map_height;
	map.info.resolution = map_resolution_;
	map.info.width = map_width;
	map.info.height = map_height;
	map.info.origin.position.x = x_min_;
	map.info.origin.position.y = y_min_;
	map.info.origin.position.z = 0;

	map.data.resize(map_size, 0);
	/*
	// NO.1
	if(!cloudtogrid->points.empty()) {
		for(size_t i = 0; i < cloudtogrid->points.size(); i++) 
		{
			if(cloudtogrid->points[i].x < x_min_ || cloudtogrid->points[i].x > x_max_ ||
			   cloudtogrid->points[i].y < y_min_ || cloudtogrid->points[i].y > y_max_ ||
			   cloudtogrid->points[i].z < z_min_ || cloudtogrid->points[i].z > z_max_) {
				continue;
			}
			else if((cloudtogrid->points[i].x <= 0.2) && (cloudtogrid->points[i].x >= (-car_length_- 0.2)) &&
					(cloudtogrid->points[i].y <= (car_width_/2 + 0.3)) && (cloudtogrid->points[i].y >= (- car_width_/2 - 0.3))) {
				continue;
			}
			else {
				int map_x = (int)((cloudtogrid->points[i].x - x_min_)/map_resolution_);
				int map_y = (int)((cloudtogrid->points[i].y - y_min_)/map_resolution_);
				int index = map_y * map_width + map_x;
				if(index < map_size && index >= 0) {
					map.data[index] += 1; 
				}
			}
		}
		for(size_t j = 0; j < map_size; j++)
	    {
	        if(map.data[j] >=  5) {
				map.data[j] = 100;          
	        }
	        else {
	        	map.data[j] = 0;
	        }
	    }
	}*/
	//NO.2
	std::vector<std::vector<double> > point_z(map_size);
	if(!cloudtogrid->points.empty()) {
		#pragma omp for
		for(size_t i = 0; i < cloudtogrid->points.size(); i++) 
		{
			if(cloudtogrid->points[i].x < x_min_ || cloudtogrid->points[i].x > x_max_ ||
			   cloudtogrid->points[i].y < y_min_ || cloudtogrid->points[i].y > y_max_ ||
			   cloudtogrid->points[i].z < z_min_ || cloudtogrid->points[i].z > z_max_ ||
			   								  !pcl_isfinite(cloudtogrid->points[i].x) || 
			   								  !pcl_isfinite(cloudtogrid->points[i].y) ||
			   								  !pcl_isfinite(cloudtogrid->points[i].z)) {
				continue;
			}
			else if((cloudtogrid->points[i].x <= 0.2) && (cloudtogrid->points[i].x >= (-car_length_- 0.2)) &&
					(cloudtogrid->points[i].y <= (car_width_/2 + 0.3)) && (cloudtogrid->points[i].y >= (- car_width_/2 - 0.3))) {
				continue;
			}
			else {
				//save the filter pointcloud
				//std::cout << " x: y: z: = " << cloudtogrid->points[i].x << " " << cloudtogrid->points[i].y << " " << cloudtogrid->points[i].z << std::endl;
				filter_cloud_->points.push_back(cloudtogrid->points[i]);
				//calculate the index of the pointcloud in the gridmap
				int map_x = (int)((cloudtogrid->points[i].x - x_min_)/map_resolution_);
				int map_y = (int)((cloudtogrid->points[i].y - y_min_)/map_resolution_);
				int index = map_y * map_width + map_x;
				if(index < map_size && index >= 0) {
					point_z[index].push_back(cloudtogrid->points[i].z);
				}
			}
		}
		for(size_t j = 0; j < map_size; j++)
	    {
	    	std::vector<double> v = point_z[j];
	    	if(v.empty()) {
	    		map.data[j] = 0;
	    	}
	    	else {
	    		//std::cout << "Max:" << *max_element(v.begin(), v.end()) << "  Min:" << *min_element(v.begin(), v.end()) << std::endl;
	    		double value = (*max_element(v.begin(), v.end())) - (*min_element(v.begin(), v.end()));
	    		//std::cout << "value:" << value << std::endl;
	    		if(value >= 0.13) {
	    			map.data[j] = 100;
	    		}
	    		else {
	        		map.data[j] = 0;
	        	}
	    	}	        
	    }
	}

	grid_map_pub_.publish(map);
}

void LidarPerception::doListening()
{
    ros::Rate loop_rate(10);

    while(nh_.ok()) {

        ros::spinOnce();

        //获取当前时间
    	ros::Time begin = ros::Time::now(); 
        /*-----------------------------------main loop-----------------------------------*/

        //1. Receive pointcloud
        //Converting the pointcloud of the left and right lidar coordinate system to car body coordinate system
	    pcl::transformPointCloud(*left_cloud_, *left_transfor_cloud_ptr_, left_transform_mat_);
	    pcl::transformPointCloud(*right_cloud_, *right_transfor_cloud_ptr_, right_transform_mat_);

	    center_cloud_->points.clear();
	    center_cloud_->points.insert(center_cloud_->points.end(),
	    				  left_transfor_cloud_ptr_->points.begin(),left_transfor_cloud_ptr_->points.end());
    	center_cloud_->points.insert(center_cloud_->points.end(),
    		             right_transfor_cloud_ptr_->points.begin(),right_transfor_cloud_ptr_->points.end());
	    ROS_WARN("center cloud size: %ld ", center_cloud_->points.size());
	    
	    //2. Constructing 2-D grid map and publish
	    filter_cloud_->clear();
        makeGridMap(center_cloud_);
        ROS_WARN("filter cloud size: %ld ", filter_cloud_->points.size());
        
        if(!filter_cloud_->points.empty()) {
        	//publish fusion to center pointcloud
			sensor_msgs::PointCloud2 centerMsg;
	        pcl::toROSMsg(*filter_cloud_, centerMsg);
	        centerMsg.header.frame_id = "rslidar";
	        centerMsg.header.stamp = ros::Time::now();        
	        center_lidar_pub_.publish(centerMsg);

	        //3. Its begin to ground plane fitting and remove the ground pointcloud
	        //sort on Z-axis value.
    		std::sort(filter_cloud_->points.begin(), filter_cloud_->points.end(), point_cmp);
    		//ROS_INFO("now filter cloud size: %ld ", filter_cloud_->points.size());

    		//4. Extract init ground seeds.
		    //LPR is the mean of low point representative
		    double sum = 0;
		    int cnt = 0;
		    //calculate the mean height value.
		    for (size_t i = 0; i < filter_cloud_->points.size() && cnt < 25; ++i) {
		        sum += filter_cloud_->points[i].z;
		        ++cnt;    
		    }
		    double lpr_height = (cnt != 0 ? sum / cnt : 0); // in case divide by 0
		    seed_cloud_->clear();
		    ground_cloud_->clear();
		    //iterate pointcloud, filter those height is less than lpr.height+th_seeds_
		    #pragma omp parallel for
		    for (size_t i = 0; i < filter_cloud_->points.size(); ++i)
		    {
		        if (filter_cloud_->points[i].z < lpr_height + 1.2)
		        {
		            seed_cloud_->points.push_back(filter_cloud_->points[i]);
		        }
		    }
		    pcl::copyPointCloud(*seed_cloud_, *ground_cloud_); //copy
		   	ROS_WARN("seed_cloud size: %ld ", seed_cloud_->points.size());
		   	//ROS_INFO("ground_cloud size: %ld ", ground_cloud_->points.size());

		    // 5. Ground plane fitter mainloop
		    for (size_t i = 0; i < 4; ++i)//num_iter = 4
		    {
		        //create covarian matrix in single pass.
			    Eigen::Matrix3f cov;
			    Eigen::Vector4f pc_mean;
			    pcl::computeMeanAndCovarianceMatrix(*ground_cloud_, cov, pc_mean);
			    //Singular Value Decomposition: SVD
			    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
			    //use the least singular vector as normal
			    normal_ = (svd.matrixU().col(2));
			    //std::cout << "AAA:normal" << normal_.matrix() << std::endl;
			    // mean ground seeds value
			    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
			    // according to normal.T*[x,y,z] = -d
			    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
			    // set distance threhold to `th_dist - d`
			    th_dist_d_ = 0.3 - d_;
			    //ROS_WARN_STREAM("d_ = " << d_ << "; th_dist_d_ = " << th_dist_d_);
			    ground_cloud_->clear();
			    no_ground_cloud_->clear();
		        //pointcloud to matrix
		        Eigen::MatrixXf points(filter_cloud_->points.size(), 3);
		        int j = 0;
		        #pragma omp for
		        for (const auto &p : filter_cloud_->points)
		    	{
		            points.row(j++) << p.x, p.y, p.z;
		        }
		        // ground plane model
		        Eigen::VectorXf result = points * normal_;
		        // threshold filter
		        #pragma omp for
		        for (size_t r = 0; r < result.rows(); ++r)
		        {
		            if (result[r] < th_dist_d_) {
		                ground_cloud_->points.push_back(filter_cloud_->points[r]);// means ground
		            }
		            else {
		                no_ground_cloud_->points.push_back(filter_cloud_->points[r]);// means not ground 
		            }
		        }
		    }
		    ROS_WARN_STREAM("ground_cloud: "<< ground_cloud_->points.size() << 
		    	       "; no_ground_cloud: " << no_ground_cloud_->points.size());
		    ROS_INFO("[ get no ground cloud ] use %f s", (ros::Time::now() - begin).toSec());  
		    // 6. Publish no ground points
		    sensor_msgs::PointCloud2 groundless_msg;
		    pcl::toROSMsg(*no_ground_cloud_, groundless_msg);
		    groundless_msg.header.frame_id = "rslidar";
	        groundless_msg.header.stamp = ros::Time::now();  
		    no_ground_pub_.publish(groundless_msg);  

		    /*!
		     * 7. Its begin to cluster by distance 
		     * cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
		     * the scanned point cloud is divided into five pointcloud according to its distance
		     */
		    bbox_candidate_.clear();
		    std::vector<PointCloudPtr> segment_pc_array_(5);
		    #pragma omp for
		    for(size_t i = 0; i < segment_pc_array_.size(); ++i) {
		    	PointCloudPtr tmp(new PointCloud());
		    	segment_pc_array_[i] = tmp;
		    }
		    #pragma omp parallel for
		    for(size_t j = 0; j < no_ground_cloud_->points.size(); ++j)
		    {
		    	pcl::PointXYZ current_point;
		    	current_point.x = no_ground_cloud_->points[j].x;
		        current_point.y = no_ground_cloud_->points[j].y;
		        current_point.z = no_ground_cloud_->points[j].z;

		    	float origin_distance = std::sqrt(std::pow(current_point.x, 2) + std::pow(current_point.y, 2));
		        if (origin_distance > 90 || origin_distance < 0.0 || current_point.y < -10 || 
		        	current_point.y > 10 || current_point.x < -15 || current_point.x > 60  || 
		                    current_point.z < -height_threshold_ || current_point.z > 2.0  ||
		                    								(!pcl_isfinite(origin_distance))) {
		            continue;
		        }
		        //std::cout << "height_threshold_ :" << -height_threshold_ << std::endl;
		        if (origin_distance < seg_distance_[0]) {
		            segment_pc_array_[0]->points.push_back(current_point);
		        }
		        else if (origin_distance < seg_distance_[1]) {
		            segment_pc_array_[1]->points.push_back(current_point);
		        }
		        else if (origin_distance < seg_distance_[2]) {
		            segment_pc_array_[2]->points.push_back(current_point);
		        }
		        else if (origin_distance < seg_distance_[3]) {
		            segment_pc_array_[3]->points.push_back(current_point);
		        }
		        else {
		            segment_pc_array_[4]->points.push_back(current_point);
		        }
		    }

		    //8 Cluster segment
		    //the five pointclouds are clustered by using different radius thresholds, and obstacle centers and Bounding Box are calculated.
		    for(size_t i = 0; i < segment_pc_array_.size(); ++i)
		    {
		    	unsegmented_cloud_->clear();
		    	pcl::copyPointCloud(*segment_pc_array_[i], *unsegmented_cloud_);
		    	// ROS_WARN_STREAM("segment_pc_array[]: "<< segment_pc_array_[i]->points.size() << 
		    	//        "; unsegmented_cloud: " << unsegmented_cloud_->points.size());
		    	#pragma omp for
		    	for(auto &a : unsegmented_cloud_->points)
		    		a.z = 0;
		    	// std::cout << "unsegmented_cloud ::: " << unsegmented_cloud_->points[0].z << " ;" << unsegmented_cloud_->points[0].x << " ;" << unsegmented_cloud_->points[0].y << ";"
		    	//           << unsegmented_cloud_->points[1].z << std::endl;
		    	// kd tree
		    	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		    	if (unsegmented_cloud_->points.size() > 0)
        			tree->setInputCloud(unsegmented_cloud_);
		    	std::vector<pcl::PointIndices> local_indices;

			    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
			    euclid.setInputCloud(unsegmented_cloud_);
			    euclid.setClusterTolerance(cluster_distance_[i]);
			    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
			    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
			    euclid.setSearchMethod(tree);
			    euclid.extract(local_indices);
			    // calcute the bbox
		        for (size_t j = 0; j < local_indices.size(); ++j)
			    {
			        float min_x = std::numeric_limits<float>::max();
			        float max_x = -std::numeric_limits<float>::max();
			        float min_y = std::numeric_limits<float>::max();
			        float max_y = -std::numeric_limits<float>::max();
			        float min_z = std::numeric_limits<float>::max();
			        float max_z = -std::numeric_limits<float>::max();

			        for (auto pit = local_indices[j].indices.begin(); pit != local_indices[j].indices.end(); ++pit) {
			            //fill new colored cluster point by point
			            pcl::PointXYZ p;
			            p.x = segment_pc_array_[i]->points[*pit].x;
			            p.y = segment_pc_array_[i]->points[*pit].y;
			            p.z = segment_pc_array_[i]->points[*pit].z;

			            if (p.x < min_x)
			                min_x = p.x;
			            if (p.y < min_y)
			                min_y = p.y;
			            if (p.z < min_z)
			                min_z = p.z;
			            if (p.x > max_x)
			                max_x = p.x;
			            if (p.y > max_y)
			                max_y = p.y;
			            if (p.z > max_z)
			                max_z = p.z;
			        }
			        //calculate bounding box 
			        double length = max_x - min_x + 0.2;
			        double width = max_y - min_y + 0.2;
			        double height = max_z - min_z + 0.2;
			        //center coordinate
			        jsk_recognition_msgs::BoundingBox bbox;
			        bbox.pose.position.x = min_x + length / 2;
			        bbox.pose.position.y = min_y + width / 2;
			        bbox.pose.position.z = min_z + height / 2;

			        bbox.dimensions.x = ((length < 0) ? -1 * length : length);
			        bbox.dimensions.y = ((width < 0) ? -1 * width : width);
			        bbox.dimensions.z = ((height < 0) ? -1 * height : height);

			        bbox.header.frame_id = "rslidar";
	        		bbox.header.stamp = ros::Time::now();  

			        bbox_candidate_.push_back(bbox);
			    }
		    }
		    ROS_WARN_STREAM("bbox_candidate size: " << bbox_candidate_.size());
		    
		    // 9. Publish the detecting bounding box
		    jsk_recognition_msgs::BoundingBoxArray bbox_array;
		    if(!bbox_candidate_.empty()) {
		    	#pragma omp for
		    	for(size_t i = 0; i < bbox_candidate_.size(); ++i)
		    		bbox_array.boxes.push_back(bbox_candidate_[i]);
		    }
		    bbox_array.header.frame_id = "rslidar";
	        bbox_array.header.stamp = ros::Time::now();  
	        detecting_bbox_pub_.publish(bbox_array); 

		}
		ROS_INFO("---------ending and spinning-------");
		ROS_INFO("[ lidar_perception ] use %f s", (ros::Time::now() - begin).toSec());  

		/*
		// segmentation
		// 创建一个分割器
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		//创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
	    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients ());
		//inliers表示误差能容忍的点 记录的是点云的序号
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// 可选择配置，设置模型系数需要优化
		seg.setOptimizeCoefficients (true);
		// Mandatory-设置目标几何形状
		seg.setModelType (pcl::SACMODEL_PLANE);
		//分割方法：随机采样法
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (1000);
		//设置误差容忍范围(距离阀值)，距离阀值决定了点被认为是局内点是必须满足的条件,表示点到估计模型的距离最大值，
		seg.setDistanceThreshold (0.15);
		//输入点云
		seg.setInputCloud(center_cloud_);
		//分割点云,存储分割结果到点几何inliers及存储平面模型的系数coefficients
		seg.segment (*inliers, *coefficients_plane);
		std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
		// Extract the inliers
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setNegative (true);
	    extract.setInputCloud (center_cloud_);
	    extract.setIndices (inliers);
	    extract.filter (*filter_cloud_);
		
		//Create pointcloud filter
	    pcl::CropBox<pcl::PointXYZ> boxFilter;
		boxFilter.setMin(Eigen::Vector4f(x_min_, y_min_, z_min_, 1.0));
		boxFilter.setMax(Eigen::Vector4f(x_max_, y_max_, z_max_, 1.0));
		boxFilter.setInputCloud(center_cloud_);
		boxFilter.setKeepOrganized(true); 
		boxFilter.setUserFilterValue(0.1f); 
		boxFilter.filter(*filter_cloud_);
		ROS_WARN("filter cloud size %ld ", filter_cloud_->points.size());
	    */    
        loop_rate.sleep();           
    }
}

