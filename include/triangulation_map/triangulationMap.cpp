/*
	FILE: triangulation_map.cpp
	--------------------------------------
	function definition of triangulatorMap class
*/
#include <triangulation_map/triangulationMap.h>

namespace triangulationMap{
    triangulatorMap::triangulatorMap(){
        // constructor
        this->ns_ = "triangulation_map";
        this->hint_ = "[TRIANGULATION] ";
    }

    triangulatorMap::triangulatorMap(const ros::NodeHandle& nh) : nh_(nh) {
        // constructor
        this->ns_ = "triangulation_map";
        this->hint_ = "[TRIANGULATION] ";
        this->initParam();
        this->registerPub();
        this->registerSub();
    }

    void triangulatorMap::initTriangulatorMap(const ros::NodeHandle& nh){
        // initialize triangulatorMap
        this->nh_ = nh;
        this->initParam();
        this->registerPub();
        this->registerSub();
    }

    void triangulatorMap::initParam(){
        // depth topic name
        if (not this->nh_.getParam(this->ns_ + "/depth_image_topic", this->depthTopicName_)){
            this->depthTopicName_ = "/camera/depth/image_raw";
            cout << this->hint_ << ": No depth image topic name. Use default: /camera/depth/image_raw" << endl;
        }
        else{
            cout << this->hint_ << ": Depth topic: " << this->depthTopicName_ << endl;
        }

        //depth aligned topic name
        if (not this->nh_.getParam(this->ns_ + "/depth_aligned_topic", this->depth_alignedTopicName_)){
            this->depth_alignedTopicName_ = "/camera/aligned_depth_to_color/image_raw_t";
            cout << this->hint_ << ": No depth aligned topic name. Use default: /camera/aligned_depth_to_color/image_raw_t" << endl;
        }
        else{
            cout << this->hint_ << ": Depth aligned topic: " << this->depth_alignedTopicName_ << endl;
        }

        // pose topic name
        if (not this->nh_.getParam(this->ns_ + "/pose_topic", this->poseTopicName_)){
            this->poseTopicName_ = "/CERLAB/quadcopter/pose";
            cout << this->hint_ << ": No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
        }
        else{
            cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
        }

        //seg map topic name
        if (not this->nh_.getParam(this->ns_ + "/seg_topic", this->semanticMapTopicName_)){
            this->semanticMapTopicName_ = "/CERLAB/quadcopter/semantic_map";
            cout << this->hint_ << ": No semantic map topic name. Use default: /CERLAB/quadcopter/semantic_map" << endl;
        }
        else{
            cout << this->hint_ << ": Semantic map topic: " << this->semanticMapTopicName_ << endl;
        }

        std::vector<double> robotSizeVec (3);
        if (not this->nh_.getParam(this->ns_ + "/robot_size", robotSizeVec)){
            robotSizeVec = std::vector<double>{0.5, 0.5, 0.3};
        }
        else{
            cout << this->hint_ << ": robot size: " << "[" << robotSizeVec[0]  << ", " << robotSizeVec[1] << ", "<< robotSizeVec[2] << "]" << endl;
        }
        this->robotSize_(0) = robotSizeVec[0]; this->robotSize_(1) = robotSizeVec[1]; this->robotSize_(2) = robotSizeVec[2];

        std::vector<double> depthIntrinsics (4);
        if (not this->nh_.getParam(this->ns_ + "/depth_intrinsics", depthIntrinsics)){
            cout << this->hint_ << ": Please check camera intrinsics!" << endl;
            exit(0);
        }
        else{
            this->fx_ = depthIntrinsics[0];
            this->fy_ = depthIntrinsics[1];
            this->cx_ = depthIntrinsics[2];
            this->cy_ = depthIntrinsics[3];
            cout << this->hint_ << ": fx, fy, cx, cy: " << "["  << this->fx_ << ", " << this->fy_  << ", " << this->cx_ << ", "<< this->cy_ << "]" << endl;
        }

        // depth scale factor
        if (not this->nh_.getParam(this->ns_ + "/depth_scale_factor", this->depthScale_)){
            this->depthScale_ = 1000.0;
            cout << this->hint_ << ": No depth scale factor. Use default: 1000." << endl;
        }
        else{
            cout << this->hint_ << ": Depth scale factor: " << this->depthScale_ << endl;
        }

        // transform matrix: body to camera
        std::vector<double> body2CamVec (16);
        if (not this->nh_.getParam(this->ns_ + "/body_to_camera", body2CamVec)){
            ROS_ERROR("[triangulation_map]: Please check body to camera matrix!");
        }
        else{
            for (int i=0; i<4; ++i){
                for (int j=0; j<4; ++j){
                    this->body2Cam_(i, j) = body2CamVec[i * 4 + j];
                }
            }
            // cout << this->hint_ << ": from body to camera: " << endl;
            // cout << this->body2Cam_ << endl;
        }
    }

    void triangulatorMap::registerCallback() {
        // depth callback
        this->depthSub_.reset(
                new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
        //depth aligned callback
        this->depth_alignedSub_.reset(
                new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depth_alignedTopicName_, 50));
        //pose callback
        this->poseSub_.reset(
                new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
        //TODO: add semantic map subscriber
        this->semanticMapSub_.reset(
                new message_filters::Subscriber<std_msgs::UInt16MultiArray>(this->nh_, this->semanticMapTopicName_, 25));

        this->depthSub_->registerCallback(boost::bind(&triangulatorMap::depthImageCB, this, _1));
        this->depth_alignedSub_->registerCallback(boost::bind(&triangulatorMap::depthAlignedImageCB, this, _1));
        this->poseSub_->registerCallback(boost::bind(&triangulatorMap::poseCB, this, _1));
        this->semanticMapSub_->registerCallback(boost::bind(&triangulatorMap::semanticMapCB, this, _1));

        this->triangulationMap_Timer_ = this->nh_.createTimer(ros::Duration(0.1), &triangulatorMap::triangulationMapCB, this);
    }

    void triangulatorMap::registerPub(){//TODO:
        // depth image publisher
        this->depthImagePub_ = this->nh_.advertise<sensor_msgs::Image>(this->ns_ + "/depth_image", 10);
        this->depthCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/depth_cloud", 10);
    }

    void triangulatorMap::getMask(int height, int width, int channel) {
        this->mask_.clear(); // first, clear the old data

        // create a mask for each channel
        for(int i=0;i<channel;i++){
            cv::Mat mask(height, width, CV_16UC1, cv::Scalar(0));
            this->mask_.push_back(mask);
        }

        // fill the mask with data
        if(channel !=0){
            int data_idx = 0;
            for(int i=0;i<height;i++){
                for(int j=0;j<width;j++){
                    for(int k=0;k<channel;k++){
                        this->mask_[k].at<ushort>(i,j) = this->semanticMap_.data[data_idx];
                        data_idx++;
                    }
                }
            }
        }

    }

    void triangulatorMap::projectDepthImage(){//TODO: (image size: height: 480; width:640) convert the 1-D array back to the original depth image
        int height = 480;
        int width = 640;
        int channel = this->semanticMap_.data.size()/height/width;
        // get mask
        this->getMask(height, width, channel);
        // get depth image from the first channel of mask
        if(!mask_.empty()){
            this->depthImage_ = mask_[0].clone();
        }else{
            std::cout << "Mask is empty, can't get depth image" << std::endl;
            this->depthImage_ = cv::Mat(480, 640, CV_16UC1, cv::Scalar(0));
        }
        // project depth image to point cloud
        this->projPoints_.clear();
        this->projPointsNum_ = 0;

        Eigen::Vector3d currPointCam, currPointMap;
        double depth;
        const double inv_factor = 1.0 / this->depthScale_;
        const double inv_fx = 1.0 / this->fx_;
        const double inv_fy = 1.0 / this->fy_;

        for (int v=0; v<this->depthImage_.rows; ++v){
            for (int u=0; u<this->depthImage_.cols; ++u){
                depth = static_cast<double>(this->depthImage_.at<ushort>(v, u)) * inv_factor;
                if (depth > 0.0){
                    currPointCam(0) = (u - this->cx_) * depth * inv_fx;
                    currPointCam(1) = (v - this->cy_) * depth * inv_fy;
                    currPointCam(2) = depth;

                    currPointMap = this->body2Cam_.block<3, 3>(0, 0) * currPointCam + this->body2Cam_.block<3, 1>(0, 3);

                    this->projPoints_.push_back(currPointMap);
                    this->projPointsNum_++;
                }
            }
        }

        // publish point cloud
        this->publishProjPoints();
    }

    void triangulatorMap::publishProjPoints(){
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        for (int i=0; i<this->projPointsNum_; ++i){
            pt.x = this->projPoints_[i](0);
            pt.y = this->projPoints_[i](1);
            pt.z = this->projPoints_[i](2);
            cloud.push_back(pt);
        }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "map";

        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(cloud, cloudMsg);
        this->depthCloudPub_.publish(cloudMsg);
    }

    void triangulatorMap::publishDepthImage(){
        cv_bridge::CvImage img;
        img.header.stamp = ros::Time::now();
        img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        img.image = this->depthImage_;
        this->depthImagePub_.publish(img.toImageMsg());
    }

    void triangulatorMap::depthImageCB(const sensor_msgs::ImageConstPtr& depthImageMsg){
        // depth image callback
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(depthImageMsg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        this->depthImage_ = cv_ptr->image;
    }

    void triangulatorMap::depthAlignedImageCB(const sensor_msgs::ImageConstPtr& depthAlignedImageMsg){
        // depth aligned image callback
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(depthAlignedImageMsg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        this->depthAlignedImage_ = cv_ptr->image;
    }

    void triangulatorMap::poseCB(const geometry_msgs::PoseStampedConstPtr& poseMsg){
        // pose callback
        this->position_(0) = poseMsg->pose.position.x;
        this->position_(1) = poseMsg->pose.position.y;
        this->position_(2) = poseMsg->pose.position.z;

        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(poseMsg->pose.orientation.w, poseMsg->pose.orientation.x, poseMsg->pose.orientation.y, poseMsg->pose.orientation.z);
        this->orientation_ = quat.toRotationMatrix();
    }

    void triangulatorMap::semanticMapCB(const std_msgs::UInt16MultiArrayConstPtr& semanticMapMsg){
        // semantic map callback
        this->semanticMap_ = *semanticMapMsg;
        std::cout << "semantic map callback" << std::endl;
    }

    void triangulatorMap::triangulationMapCB(const ros::TimerEvent& event){
        // project depth image to point cloud
        this->projectDepthImage();
        // publish depth image
        this->publishDepthImage();
    }

    void triangulatorMap::registerSub(){
        // register subscribers
        this->registerCallback();
    }
}
