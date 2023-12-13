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

        // ------------------------------------------------------------------------------------
        // depth image columns
        if (not this->nh_.getParam(this->ns_ + "/image_cols", this->imgCols_)){
            this->imgCols_ = 640;
            cout << this->hint_ << ": No depth image columns. Use default: 640." << endl;
        }
        else{
            cout << this->hint_ << ": Depth image columns: " << this->imgCols_ << endl;
        }

        // depth skip pixel
        if (not this->nh_.getParam(this->ns_ + "/image_rows", this->imgRows_)){
            this->imgRows_ = 480;
            cout << this->hint_ << ": No depth image rows. Use default: 480." << endl;
        }
        else{
            cout << this->hint_ << ": Depth image rows: " << this->imgRows_ << endl;
        }
        this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
        // ------------------------------------------------------------------------------------

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

        // map resolution
        if (not this->nh_.getParam(this->ns_ + "/map_resolution", this->mapRes_)){
            this->mapRes_ = 0.1;
            cout << this->hint_ << ": No map resolution. Use default: 0.1." << endl;
        }
        else{
            cout << this->hint_ << ": Map resolution: " << this->mapRes_ << endl;
        }

        // ground height
        if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHeight_)){
            this->groundHeight_ = 0.0;
            cout << this->hint_ << ": No ground height. Use default: 0.0." << endl;
        }
        else{
            cout << this->hint_ << ": Ground height: " << this->groundHeight_ << endl;
        }


        // map size
        std::vector<double> mapSizeVec (3);
        if (not this->nh_.getParam(this->ns_ + "/map_size", mapSizeVec)){
            mapSizeVec[0] = 20; mapSizeVec[1] = 20; mapSizeVec[2] = 3;
            cout << this->hint_ << ": No map size. Use default: [20, 20, 3]." << endl;
        }
        else{
            this->mapSize_(0) = mapSizeVec[0];
            this->mapSize_(1) = mapSizeVec[1];
            this->mapSize_(2) = mapSizeVec[2];

            // init min max
            this->mapSizeMin_(0) = -mapSizeVec[0]/2; this->mapSizeMax_(0) = mapSizeVec[0]/2;
            this->mapSizeMin_(1) = -mapSizeVec[1]/2; this->mapSizeMax_(1) = mapSizeVec[1]/2;
            this->mapSizeMin_(2) = this->groundHeight_; this->mapSizeMax_(2) = this->groundHeight_ + mapSizeVec[2];

            // min max for voxel
            this->mapVoxelMin_(0) = 0; this->mapVoxelMax_(0) = ceil(mapSizeVec[0]/this->mapRes_);
            this->mapVoxelMin_(1) = 0; this->mapVoxelMax_(1) = ceil(mapSizeVec[1]/this->mapRes_);
            this->mapVoxelMin_(2) = 0; this->mapVoxelMax_(2) = ceil(mapSizeVec[2]/this->mapRes_);

            // reserve vector for variables
            int reservedSize = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
            this->countHitMiss_.resize(reservedSize, 0);
            this->countHit_.resize(reservedSize, 0);
            this->occupancy_.resize(reservedSize, this->pMinLog_-this->UNKNOWN_FLAG_);
            this->occupancyInflated_.resize(reservedSize, false);
            this->flagTraverse_.resize(reservedSize, -1);
            this->flagRayend_.resize(reservedSize, -1);

            cout << this->hint_ << ": Map size: " << "[" << mapSizeVec[0] << ", " << mapSizeVec[1] << ", " << mapSizeVec[2] << "]" << endl;
        }

        // Raycast max length
        if (not this->nh_.getParam(this->ns_ + "/raycast_max_length", this->raycastMaxLength_)){
            this->raycastMaxLength_ = 5.0;
            cout << this->hint_ << ": No raycast max length. Use default: 5.0." << endl;
        }
        else{
            cout << this->hint_ << ": Raycast max length: " << this->raycastMaxLength_ << endl;
        }

        // p hit
        double pHit;
        if (not this->nh_.getParam(this->ns_ + "/p_hit", pHit)){
            pHit = 0.70;
            cout << this->hint_ << ": No p hit. Use default: 0.70." << endl;
        }
        else{
            cout << this->hint_ << ": P hit: " << pHit << endl;
        }
        this->pHitLog_ = this->logit(pHit);

        // p miss
        double pMiss;
        if (not this->nh_.getParam(this->ns_ + "/p_miss", pMiss)){
            pMiss = 0.35;
            cout << this->hint_ << ": No p miss. Use default: 0.35." << endl;
        }
        else{
            cout << this->hint_ << ": P miss: " << pMiss << endl;
        }
        this->pMissLog_ = this->logit(pMiss);

        // p min
        double pMin;
        if (not this->nh_.getParam(this->ns_ + "/p_min", pMin)){
            pHit = 0.12;
            cout << this->hint_ << ": No p min. Use default: 0.12." << endl;
        }
        else{
            cout << this->hint_ << ": P min: " << pMin << endl;
        }
        this->pMinLog_ = this->logit(pMin);

        // p max
        double pMax;
        if (not this->nh_.getParam(this->ns_ + "/p_max", pMax)){
            pMax = 0.97;
            cout << this->hint_ << ": No p max. Use default: 0.97." << endl;
        }
        else{
            cout << this->hint_ << ": P max: " << pMax << endl;
        }
        this->pMaxLog_ = this->logit(pMax);

        // p occ
        double pOcc;
        if (not this->nh_.getParam(this->ns_ + "/p_occ", pOcc)){
            pOcc = 0.80;
            cout << this->hint_ << ": No p occ. Use default: 0.80." << endl;
        }
        else{
            cout << this->hint_ << ": P occ: " << pOcc << endl;
        }
        this->pOccLog_ = this->logit(pOcc);

        // local update range
        std::vector<double> localUpdateRangeVec;
        if (not this->nh_.getParam(this->ns_ + "/local_update_range", localUpdateRangeVec)){
            localUpdateRangeVec = std::vector<double>{5.0, 5.0, 3.0};
            cout << this->hint_ << ": No local update range. Use default: [5.0, 5.0, 3.0] m." << endl;
        }
        else{
            cout << this->hint_ << ": Local update range: " << "[" << localUpdateRangeVec[0] << ", " << localUpdateRangeVec[1] << ", " << localUpdateRangeVec[2] << "]" << endl;
        }
        this->localUpdateRange_(0) = localUpdateRangeVec[0]; this->localUpdateRange_(1) = localUpdateRangeVec[1]; this->localUpdateRange_(2) = localUpdateRangeVec[2];


        // local bound inflate factor
        if (not this->nh_.getParam(this->ns_ + "/local_bound_inflation", this->localBoundInflate_)){
            this->localBoundInflate_ = 0.0;
            cout << this->hint_ << ": No local bound inflate. Use default: 0.0 m." << endl;
        }
        else{
            cout << this->hint_ << ": Local bound inflate: " << this->localBoundInflate_ << endl;
        }

        // whether to clean local map
        if (not this->nh_.getParam(this->ns_ + "/clean_local_map", this->cleanLocalMap_)){
            this->cleanLocalMap_ = true;
            cout << this->hint_ << ": No clean local map option. Use default: true." << endl;
        }
        else{
            cout << this->hint_ << ": Clean local map option is set to: " << this->cleanLocalMap_ << endl;
        }

        // absolute dir of prebuilt map file (.pcd)
        if (not this->nh_.getParam(this->ns_ + "/prebuilt_map_directory", this->prebuiltMapDir_)){
            this->prebuiltMapDir_ = "";
            cout << this->hint_ << ": Not using prebuilt map." << endl;
        }
        else{
            cout << this->hint_ << ": the prebuilt map absolute dir is found: " << this->prebuiltMapDir_ << endl;
        }

        // local map size (visualization)
        std::vector<double> localMapSizeVec;
        if (not this->nh_.getParam(this->ns_ + "/local_map_size", localMapSizeVec)){
            localMapSizeVec = std::vector<double>{10.0, 10.0, 2.0};
            cout << this->hint_ << ": No local map size. Use default: [10.0, 10.0, 3.0] m." << endl;
        }
        else{
            cout << this->hint_ << ": Local map size: " << "[" << localMapSizeVec[0] << ", " << localMapSizeVec[1] << ", " << localMapSizeVec[2] << "]" << endl;
        }
        this->localMapSize_(0) = localMapSizeVec[0]/2; this->localMapSize_(1) = localMapSizeVec[1]/2; this->localMapSize_(2) = localMapSizeVec[2]/2;
        this->localMapVoxel_(0) = int(ceil(localMapSizeVec[0]/(2*this->mapRes_))); this->localMapVoxel_(1) = int(ceil(localMapSizeVec[1]/(2*this->mapRes_))); this->localMapVoxel_(2) = int(ceil(localMapSizeVec[2]/(2*this->mapRes_)));

        // max vis height
        if (not this->nh_.getParam(this->ns_ + "/max_height_visualization", this->maxVisHeight_)){
            this->maxVisHeight_ = 3.0;
            cout << this->hint_ << ": No max visualization height. Use default: 3.0 m." << endl;
        }
        else{
            cout << this->hint_ << ": Max visualization height: " << this->maxVisHeight_ << endl;
        }

        // visualize global map
        if (not this->nh_.getParam(this->ns_ + "/visualize_global_map", this->visGlobalMap_)){
            this->visGlobalMap_ = false;
            cout << this->hint_ << ": No visualize map option. Use default: visualize local map." << endl;
        }
        else{
            cout << this->hint_ << ": Visualize map option. local (0)/global (1): " << this->visGlobalMap_ << endl;
        }

        // verbose
        if (not this->nh_.getParam(this->ns_ + "/verbose", this->verbose_)){
            this->verbose_ = true;
            cout << this->hint_ << ": No verbose option. Use default: check update info." << endl;
        }
        else{
            if (not this->verbose_){
                cout << this->hint_ << ": Not display messages" << endl;
            }
            else{
                cout << this->hint_ << ": Display messages" << endl;
            }
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
                    int detect = false;
                    int label = 0;
                    for(int i=1;i<channel;i++){
                        if(this->mask_[i].at<ushort>(v,u) != 0){
                            detect = true;
                            label = mask_[i].at<ushort>(v,u);
                            break;
                        }
                    }
                    if(detect){
                        currPointCam(0) = (u - this->cx_) * depth * inv_fx;
                        currPointCam(1) = (v - this->cy_) * depth * inv_fy;
                        currPointCam(2) = depth;

                        currPointMap = this->body2Cam_.block<3, 3>(0, 0) * currPointCam + this->body2Cam_.block<3, 1>(0, 3);

                        this->projPoints_.push_back(currPointMap);
                        this->projPointsNum_++;
                    }
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
