#include <airsim_ros_wrapper.h>
#include <boost/make_shared.hpp>
// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(AirsimROSWrapper, nodelet::Nodelet)
#include "common/AirSimSettings.hpp"
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
constexpr char AirsimROSWrapper::CAM_YML_NAME[];
constexpr char AirsimROSWrapper::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapper::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapper::K_YML_NAME[];
constexpr char AirsimROSWrapper::D_YML_NAME[];
constexpr char AirsimROSWrapper::R_YML_NAME[];
constexpr char AirsimROSWrapper::P_YML_NAME[];
constexpr char AirsimROSWrapper::DMODEL_YML_NAME[];

const std::unordered_map<int, std::string>
    AirsimROSWrapper::image_type_int_to_string_map_ = {
        {0, "Scene"},
        {1, "DepthPlanar"},
        {2, "DepthPerspective"},
        {3, "DepthVis"},
        {4, "DisparityNormalized"},
        {5, "Segmentation"},
        {6, "SurfaceNormals"},
        {7, "Infrared"},
        {8, "OpticalFlow"}};

AirsimROSWrapper::AirsimROSWrapper(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private,
                                   const std::string &host_ip)
    : img_RGBD_async_spinner_(1, &img_timer_cb_queue_RGBD_),
      img_stereo_async_spinner_(1, &img_timer_cb_queue_stereo_),
      img_bottom_async_spinner_(1, &img_timer_cb_queue_bottom_),
      drone_state_async_spinner_(1, &drone_state_timer_cb_queue_),
      command_listener_async_spinner_(1, &command_listener_queue_)
      // , update_commands_async_spinner_(1, &update_commands_queue_)
      // , is_used_lidar_timer_cb_queue_(false)
      ,
      is_used_img_timer_cb_queue_(false), nh_(nh), nh_private_(nh_private),
      host_ip_(host_ip), airsim_settings_parser_(host_ip),
      airsim_client_images_(host_ip)
      // , airsim_client_lidar_(host_ip)
      ,
      airsim_client_states_(host_ip)
// , has_gimbal_cmd_(false)
// , tf_listener_(tf_buffer_)
{
    ros_clock_.clock.fromSec(0);

    if (AirSimSettings::singleton().simmode_name !=
        AirSimSettings::kSimModeTypeCar) {
        airsim_mode_ = AIRSIM_MODE::DRONE;
        ROS_INFO("Setting ROS wrapper to DRONE mode");
    } else {
        airsim_mode_ = AIRSIM_MODE::CAR;
        ROS_INFO("Setting ROS wrapper to CAR mode");
    }

    initialize_ros();

    std::cout << "AirsimROSWrapper Initialized!\n";
}

void AirsimROSWrapper::initialize_airsim() {
    // todo do not reset if already in air?
    try {

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(
                new msr::airlib::MultirotorRpcLibClient(host_ip_));
        } else {
            airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(
                new msr::airlib::CarRpcLibClient(host_ip_));
        }
        airsim_client_->reset();
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-1., 1.);
        double random_start_x = dis(gen) * max_randow_start_x;
        double random_start_y = dis(gen) * max_randow_start_y;
        double random_start_z = 0;
        msr::airlib::Pose cur_drone_pose = airsim_client_->simGetVehiclePose();
        random_start_x += cur_drone_pose.position.x();
        random_start_y += cur_drone_pose.position.y();
        random_start_z += cur_drone_pose.position.z();
        airsim_client_->simSetVehiclePose(
            msr::airlib::Pose(
                msr::airlib::Vector3r(random_start_x, random_start_y,
                                      random_start_z),
                msr::airlib::Quaternionr(cur_drone_pose.orientation.w(),
                                         cur_drone_pose.orientation.x(),
                                         cur_drone_pose.orientation.y(),
                                         cur_drone_pose.orientation.z())),
            true);
        ros::Duration(0.1).sleep();
        airsim_client_->confirmConnection();

        airsim_client_images_.confirmConnection();
        // airsim_client_lidar_.confirmConnection();
        airsim_client_states_.confirmConnection();
        for (const auto &vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            airsim_client_->enableApiControl(
                true,
                vehicle_name_ptr_pair.first); // todo expose as rosservice?
            airsim_client_->armDisarm(
                true,
                vehicle_name_ptr_pair.first); // todo exposes as rosservice?
            msr::airlib::vector<float> angleKpVec(3, angleKp);
            msr::airlib::vector<float> angleKiVec(3, angleKi);
            msr::airlib::vector<float> angleKdVec(3, angleKd);
            get_multirotor_client()->setAngleLevelControllerGains(
                angleKpVec, angleKiVec, angleKdVec,
                vehicle_name_ptr_pair.first);
            msr::airlib::vector<float> angleRateKpVec(3, angleRateKp);
            msr::airlib::vector<float> angleRateKiVec(3, angleRateKi);
            msr::airlib::vector<float> angleRateKdVec(3, angleRateKd);
            get_multirotor_client()->setAngleRateControllerGains(
                angleRateKpVec, angleRateKiVec, angleRateKdVec,
                vehicle_name_ptr_pair.first);
        }
    } catch (rpc::rpc_error &e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong."
                  << std::endl
                  << msg << std::endl;
    }
}

void AirsimROSWrapper::initialize_ros() {

    // ros params
    double update_airsim_control_every_n_sec;
    nh_private_.getParam("is_vulkan", is_vulkan_);
    nh_private_.getParam("update_airsim_control_every_n_sec",
                         update_airsim_control_every_n_sec);
    nh_private_.getParam("publish_clock", publish_clock_);
    nh_private_.param("world_frame_id", world_frame_id_, world_frame_id_);
    odom_frame_id_ = world_frame_id_ == AIRSIM_FRAME_ID ? AIRSIM_ODOM_FRAME_ID
                                                        : ENU_ODOM_FRAME_ID;
    nh_private_.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    isENU_ = !(odom_frame_id_ == AIRSIM_ODOM_FRAME_ID);
    nh_private_.param("coordinate_system_enu", isENU_, isENU_);
    vel_cmd_duration_ = 0.05; // todo rosparam
    nh_private_.getParam("angle_kp", angleKp);
    nh_private_.getParam("angle_ki", angleKi);
    nh_private_.getParam("angle_kd", angleKd);
    nh_private_.getParam("angle_rate_kp", angleRateKp);
    nh_private_.getParam("angle_rate_ki", angleRateKi);
    nh_private_.getParam("angle_rate_kd", angleRateKd);

    nh_private_.getParam("vechile_name", mavrosVechileName);

    nh_private_.getParam("depth_std_dev", depth_std_dev);

    nh_private_.param("max_randow_start_x", max_randow_start_x, 0.0);
    nh_private_.param("max_randow_start_y", max_randow_start_y, 0.0);

    create_ros_pubs_from_settings_json();
    ros::TimerOptions timer_options_control_update_(
        ros::Duration(update_airsim_control_every_n_sec),
        boost::bind(&AirsimROSWrapper::drone_state_timer_cb, this, _1),
        &drone_state_timer_cb_queue_);
    airsim_control_update_timer_ =
        nh_private_.createTimer(timer_options_control_update_);
}

// XmlRpc::XmlRpcValue can't be const in this case
void AirsimROSWrapper::create_ros_pubs_from_settings_json() {
    airsim_img_request_vehicle_name_pair_vec_.clear();
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();
    vehicle_name_ptr_map_.clear();
    size_t lidar_cnt = 0;

    image_transport::ImageTransport image_transporter(nh_private_);

    // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>>
    // vehicles;
    for (const auto &curr_vehicle_elem : AirSimSettings::singleton().vehicles) {
        auto &vehicle_setting = curr_vehicle_elem.second;
        auto curr_vehicle_name = curr_vehicle_elem.first;

        nh_.setParam("/vehicle_name", curr_vehicle_name);

        set_nans_to_zeros_in_pose(*vehicle_setting);

        std::unique_ptr<VehicleROS> vehicle_ros = nullptr;

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            vehicle_ros = std::unique_ptr<MultiRotorROS>(new MultiRotorROS());
        } else {
            vehicle_ros = std::unique_ptr<CarROS>(new CarROS());
        }

        vehicle_ros->odom_frame_id = curr_vehicle_name + "/" + odom_frame_id_;
        vehicle_ros->vehicle_name = curr_vehicle_name;

        vehicle_ros->odom_local_pub = nh_private_.advertise<nav_msgs::Odometry>(
            curr_vehicle_name + "/" + odom_frame_id_, 10);

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {

            auto drone = static_cast<MultiRotorROS *>(vehicle_ros.get());

            ros::SubscribeOptions ops1 =
                ros::SubscribeOptions::create<airsim_ros_pkgs::PoseCmd>(
                    curr_vehicle_name + "/pose_cmd_body_frame", // topic name
                    1,                                          // queue length
                    boost::bind(&AirsimROSWrapper::pose_cmd_body_frame_cb, this,
                                _1,
                                vehicle_ros->vehicle_name), // callback
                    ros::VoidPtr(), // tracked object, we don't need one thus
                                    // NULL
                    &command_listener_queue_ // pointer to callback queue object
                );
            ops1.allow_concurrent_callbacks = true;
            drone->pose_cmd_body_frame_sub = nh_private_.subscribe(ops1);

            // bind to a single callback. todo optimal subs queue length
            // bind multiple topics to a single callback, but keep track of
            // which vehicle name it was by passing curr_vehicle_name as the 2nd
            // argument

            ros::SubscribeOptions ops2 = ros::SubscribeOptions::create<
                airsim_ros_pkgs::AngleRateThrottle>(
                curr_vehicle_name + "/angle_rate_throttle_frame", // topic name
                1, // queue length
                boost::bind(&AirsimROSWrapper::angle_rate_throttle_frame_cb,
                            this, _1, vehicle_ros->vehicle_name), // callback
                ros::VoidPtr(), // tracked object, we don't need one thus NULL
                &command_listener_queue_ // pointer to callback queue object
            );
            ops2.allow_concurrent_callbacks = true;
            drone->angle_rate_throttle_frame_sub = nh_private_.subscribe(ops2);

            ros::SubscribeOptions ops3 =
                ros::SubscribeOptions::create<airsim_ros_pkgs::VelCmd>(
                    curr_vehicle_name + "/vel_cmd_body_frame", // topic name
                    1,                                         // queue length
                    boost::bind(&AirsimROSWrapper::vel_cmd_body_frame_cb, this,
                                _1,
                                vehicle_ros->vehicle_name), // callback
                    ros::VoidPtr(), // tracked object, we don't need one thus
                                    // NULL
                    &command_listener_queue_ // pointer to callback queue object
                );
            ops3.allow_concurrent_callbacks = true;
            drone->vel_cmd_body_frame_sub = nh_private_.subscribe(ops3);
            // TODO: ros::TransportHints().tcpNoDelay();

            ros::SubscribeOptions ops4 =
                ros::SubscribeOptions::create<mavros_msgs::AttitudeTarget>(
                    "/mavros/setpoint_raw/attitude", // topic name
                    1,                               // queue length
                    boost::bind(&AirsimROSWrapper::MavrosPoseCmdCb, this,
                                _1), // callback
                    ros::VoidPtr(),  // tracked object, we don't need one thus
                                     // NULL
                    &command_listener_queue_ // pointer to callback queue object
                );
            ops4.allow_concurrent_callbacks = true;
            drone->vel_cmd_body_frame_sub = nh_private_.subscribe(ops4);
            // TODO: ros::TransportHints().tcpNoDelay();

            ros::AdvertiseServiceOptions sops1 =
                ros::AdvertiseServiceOptions::create<airsim_ros_pkgs::Takeoff>(
                    curr_vehicle_name + "/takeoff",
                    boost::bind(&AirsimROSWrapper::takeoff_srv_cb, this, _1, _2,
                                vehicle_ros->vehicle_name),
                    ros::VoidPtr(), &command_listener_queue_);

            drone->takeoff_srvr = nh_private_.advertiseService(sops1);

            ros::AdvertiseServiceOptions sops2 =
                ros::AdvertiseServiceOptions::create<airsim_ros_pkgs::Land>(
                    curr_vehicle_name + "/land",
                    boost::bind(&AirsimROSWrapper::land_srv_cb, this, _1, _2,
                                vehicle_ros->vehicle_name),
                    ros::VoidPtr(), &command_listener_queue_);

            drone->land_srvr = nh_private_.advertiseService(sops2);
            // vehicle_ros.reset_srvr =
            // nh_private_.advertiseService(curr_vehicle_name +
            // "/reset",&AirsimROSWrapper::reset_srv_cb, this);
        } else {
        }

        // iterate over camera map std::map<std::string, CameraSetting>
        // .cameras;
        for (auto &curr_camera_elem : vehicle_setting->cameras) {
            auto &camera_setting = curr_camera_elem.second;
            auto &curr_camera_name = curr_camera_elem.first;

            set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
            // append_static_camera_tf(vehicle_ros.get(), curr_camera_name,
            // camera_setting); camera_setting.gimbal
            std::vector<ImageRequest> current_image_request_vec;

            // iterate over capture_setting std::map<int, CaptureSetting>
            // capture_settings
            for (const auto &curr_capture_elem :
                 camera_setting.capture_settings) {
                const auto &capture_setting = curr_capture_elem.second;

                // todo why does AirSimSettings::loadCaptureSettings calls
                // AirSimSettings::initializeCaptureSettings() which initializes
                // default capture settings for _all_ NINE
                // msr::airlib::ImageCaptureBase::ImageType
                if (!(std::isnan(capture_setting.fov_degrees))) {
                    const ImageType curr_image_type =
                        msr::airlib::Utils::toEnum<ImageType>(
                            capture_setting.image_type);
                    // if scene / segmentation / surface normals / infrared, get
                    // uncompressed image with pixels_as_floats = false
                    if (curr_image_type == ImageType::Scene ||
                        curr_image_type == ImageType::Segmentation ||
                        curr_image_type == ImageType::SurfaceNormals ||
                        curr_image_type == ImageType::Infrared) {
                        current_image_request_vec.push_back(ImageRequest(
                            curr_camera_name, curr_image_type, false, false));
                    }
                    // if {DepthPlanar, DepthPerspective,DepthVis,
                    // DisparityNormalized}, get float image
                    else {
                        current_image_request_vec.push_back(ImageRequest(
                            curr_camera_name, curr_image_type, true, false));
                    }

                    const std::string cam_image_topic =
                        curr_vehicle_name + "/" + curr_camera_name + "/" +
                        image_type_int_to_string_map_.at(
                            capture_setting.image_type);

                    if (strcmp(curr_camera_name.c_str(), "bottom_center") ==
                        0) {
                        bottom_pub_ =
                            image_transporter.advertise(cam_image_topic, 1);
                        bottom_request_.push_back(ImageRequest(
                            curr_camera_name, curr_image_type, false, false));
                        bottom_cam_info_pub_ =
                            nh_private_.advertise<sensor_msgs::CameraInfo>(
                                cam_image_topic + "/camera_info", 10);
                        bottom_camera_info_ = generate_cam_info(
                            curr_camera_name, camera_setting, capture_setting);
                    }
                    if (strcmp(curr_camera_name.c_str(), "front_center") == 0) {
                        is_RGBD_ = 1;
                        if (curr_image_type == ImageType::Scene) {
                            front_pub_ =
                                image_transporter.advertise(cam_image_topic, 1);
                            RGBD_request_.push_back(
                                ImageRequest(curr_camera_name, curr_image_type,
                                             false, false));
                            front_cam_info_pub_ =
                                nh_private_.advertise<sensor_msgs::CameraInfo>(
                                    cam_image_topic + "/camera_info", 10);
                            front_camera_info_ = generate_cam_info(
                                curr_camera_name, camera_setting,
                                capture_setting);
                        } else if (curr_image_type == ImageType::DepthPlanar) {
                            front_depth_pub_ =
                                image_transporter.advertise(cam_image_topic, 1);
                            RGBD_request_.push_back(
                                ImageRequest(curr_camera_name, curr_image_type,
                                             true, false));
                            front_depth_cam_info_pub_ =
                                nh_private_.advertise<sensor_msgs::CameraInfo>(
                                    cam_image_topic + "/camera_info", 10);
                            front_depth_camera_info_ = generate_cam_info(
                                curr_camera_name, camera_setting,
                                capture_setting);
                        } else if (curr_image_type == ImageType::OpticalFlow) {
                            front_opti_pub_ =
                                image_transporter.advertise(cam_image_topic, 1);
                            RGBD_request_.push_back(
                                ImageRequest(curr_camera_name, curr_image_type,
                                             true, false));
                        }
                    }
                    if (strcmp(curr_camera_name.c_str(), "front_left") == 0) {
                        is_stereo_ = 1;
                        front_left_pub_ =
                            image_transporter.advertise(cam_image_topic, 1);
                        stereo_request_.push_back(ImageRequest(
                            curr_camera_name, curr_image_type, false, false));
                        front_left_cam_info_pub_ =
                            nh_private_.advertise<sensor_msgs::CameraInfo>(
                                cam_image_topic + "/camera_info", 10);
                        front_left_camera_info_ = generate_cam_info(
                            curr_camera_name, camera_setting, capture_setting);
                    }
                    if (strcmp(curr_camera_name.c_str(), "front_right") == 0) {
                        is_stereo_ = 1;
                        front_right_pub_ =
                            image_transporter.advertise(cam_image_topic, 1);
                        stereo_request_.push_back(ImageRequest(
                            curr_camera_name, curr_image_type, false, false));
                        front_right_cam_info_pub_ =
                            nh_private_.advertise<sensor_msgs::CameraInfo>(
                                cam_image_topic + "/camera_info", 10);
                        front_right_camera_info_ = generate_cam_info(
                            curr_camera_name, camera_setting, capture_setting);
                    }
                    collision_info_pub_ = nh_private_.advertise<std_msgs::Bool>(
                        curr_vehicle_name + "/collision", 10);
                }
            }
            // push back pair (vector of image captures, current vehicle name)
            airsim_img_request_vehicle_name_pair_vec_.push_back(
                {current_image_request_vec, curr_vehicle_name});
        }

        // iterate over sensors
        std::vector<SensorPublisher> sensors;
        for (const auto &[sensor_name, sensor_setting] :
             vehicle_setting->sensors) {
            if (sensor_setting->enabled) {
                SensorPublisher sensor_publisher;
                sensor_publisher.sensor_name = sensor_name;
                sensor_publisher.sensor_type = sensor_setting->sensor_type;
                switch (sensor_setting->sensor_type) {
                case SensorBase::SensorType::Barometer: {
                    // ROS_INFO_STREAM(sensor_name << ": Barometer");
                    // sensor_publisher.publisher =
                    // nh_private_.advertise<airsim_ros_pkgs::Altimeter>(curr_vehicle_name
                    // + "/altimeter/" + sensor_name, 10);
                    break;
                }
                case SensorBase::SensorType::Imu: {
                    ROS_INFO_STREAM(sensor_name << ": IMU");
                    sensor_publisher.publisher =
                        nh_private_.advertise<sensor_msgs::Imu>(
                            curr_vehicle_name + "/imu/" + sensor_name, 10);
                    sensors.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Gps: {
                    // ROS_INFO_STREAM(sensor_name << ": GPS");
                    // sensor_publisher.publisher =
                    // nh_private_.advertise<sensor_msgs::NavSatFix>(curr_vehicle_name
                    // + "/gps/" + sensor_name, 10);
                    break;
                }
                case SensorBase::SensorType::Magnetometer: {
                    // ROS_INFO_STREAM(sensor_name << ": Magnetometer");
                    // sensor_publisher.publisher =
                    // nh_private_.advertise<sensor_msgs::MagneticField>(curr_vehicle_name
                    // + "/magnetometer/" + sensor_name, 10);
                    break;
                }
                case SensorBase::SensorType::Distance: {
                    // ROS_INFO_STREAM(sensor_name << ": Distance sensor");
                    // sensor_publisher.publisher =
                    // nh_private_.advertise<sensor_msgs::Range>(curr_vehicle_name
                    // + "/distance/" + sensor_name, 10);
                    break;
                }
                case SensorBase::SensorType::Lidar: {
                    break;
                }
                default: {
                    throw std::invalid_argument("Unexpected sensor type");
                }
                }
                // sensors.emplace_back(sensor_publisher);
            }
        }

        // we want fast access to the lidar sensors for callback handling, sort
        // them out now
        auto isLidar = [](const SensorPublisher &pub) {
            return pub.sensor_type == SensorBase::SensorType::Lidar;
        };
        size_t cnt = std::count_if(sensors.begin(), sensors.end(), isLidar);
        lidar_cnt += cnt;
        vehicle_ros->lidar_pubs.resize(cnt);
        vehicle_ros->sensor_pubs.resize(sensors.size() - cnt);
        std::partition_copy(sensors.begin(), sensors.end(),
                            vehicle_ros->lidar_pubs.begin(),
                            vehicle_ros->sensor_pubs.begin(), isLidar);

        vehicle_name_ptr_map_.emplace(
            curr_vehicle_name,
            std::move(vehicle_ros)); // allows fast lookup in command callbacks
                                     // in case of a lot of drones
    }

    if (publish_clock_) {
        clock_pub_ = nh_private_.advertise<rosgraph_msgs::Clock>("clock", 1);
    }

    // if >0 cameras, add one more thread for img_request_timer_cb
    if (!airsim_img_request_vehicle_name_pair_vec_.empty()) {
        double update_airsim_img_response_every_n_sec;
        nh_private_.getParam("update_airsim_img_response_every_n_sec",
                             update_airsim_img_response_every_n_sec);
        if (is_RGBD_) {
            ros::TimerOptions timer_options_RGBD(
                ros::Duration(update_airsim_img_response_every_n_sec),
                boost::bind(&AirsimROSWrapper::img_response_RGBD_timer_cb, this,
                            _1),
                &img_timer_cb_queue_RGBD_);
            airsim_img_response_RGBD_timer_ =
                nh_private_.createTimer(timer_options_RGBD);
        }

        if (is_stereo_) {
            ros::TimerOptions timer_options_stereo(
                ros::Duration(update_airsim_img_response_every_n_sec),
                boost::bind(&AirsimROSWrapper::img_response_stereo_timer_cb,
                            this, _1),
                &img_timer_cb_queue_stereo_);

            airsim_img_response_stereo_timer_ =
                nh_private_.createTimer(timer_options_stereo);
        }

        ros::TimerOptions timer_options_bottom(
            ros::Duration(update_airsim_img_response_every_n_sec),
            boost::bind(&AirsimROSWrapper::img_response_bottom_timer_cb, this,
                        _1),
            &img_timer_cb_queue_bottom_);

        airsim_img_response_bottom_timer_ =
            nh_private_.createTimer(timer_options_bottom);

        is_used_img_timer_cb_queue_ = true;
    }
    mPubMavrosImuCmd =
        nh_private_.advertise<sensor_msgs::Imu>("/mavros/imu/data", 50);
    mPubLocalOdomCmd =
        nh_private_.advertise<nav_msgs::Odometry>("/mavros/local/odom", 50);
    initialize_airsim();
}

// todo: error check. if state is not landed, return error.
bool AirsimROSWrapper::takeoff_srv_cb(
    airsim_ros_pkgs::Takeoff::Request &request,
    airsim_ros_pkgs::Takeoff::Response &response,
    const std::string &vehicle_name) {
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    if (request.waitOnLastTask)
        get_multirotor_client()
            ->takeoffAsync(20, vehicle_name)
            ->waitOnLastTask(); // todo value for timeout_sec?
    else
        get_multirotor_client()->takeoffAsync(20, vehicle_name);

    response.success = true;
    return response.success;
}

bool AirsimROSWrapper::land_srv_cb(airsim_ros_pkgs::Land::Request &request,
                                   airsim_ros_pkgs::Land::Response &response,
                                   const std::string &vehicle_name) {
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    if (request.waitOnLastTask)
        get_multirotor_client()->landAsync(60, vehicle_name)->waitOnLastTask();
    else
        get_multirotor_client()->landAsync(60, vehicle_name);

    response.success = true;
    return response.success; // todo
}

tf2::Quaternion AirsimROSWrapper::get_tf2_quat(
    const msr::airlib::Quaternionr &airlib_quat) const {
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(),
                           airlib_quat.w());
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(
    const geometry_msgs::Quaternion &geometry_msgs_quat) const {
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x,
                                    geometry_msgs_quat.y, geometry_msgs_quat.z);
}

msr::airlib::Quaternionr
AirsimROSWrapper::get_airlib_quat(const tf2::Quaternion &tf2_quat) const {
    return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(),
                                    tf2_quat.z());
}

// void AirsimROSWrapper::car_cmd_cb(const
// airsim_ros_pkgs::CarControls::ConstPtr& msg, const std::string& vehicle_name)
// {
//     std::lock_guard<std::mutex> guard(drone_control_mutex_);

//     auto car =
//     static_cast<CarROS*>(vehicle_name_ptr_map_[vehicle_name].get());
//     car->car_cmd.throttle = msg->throttle;
//     car->car_cmd.steering = msg->steering;
//     car->car_cmd.brake = msg->brake;
//     car->car_cmd.handbrake = msg->handbrake;
//     car->car_cmd.is_manual_gear = msg->manual;
//     car->car_cmd.manual_gear = msg->manual_gear;
//     car->car_cmd.gear_immediate = msg->gear_immediate;

//     car->has_car_cmd = true;
// }

msr::airlib::Pose AirsimROSWrapper::get_airlib_pose(
    const float &x, const float &y, const float &z,
    const msr::airlib::Quaternionr &airlib_quat) const {
    return msr::airlib::Pose(msr::airlib::Vector3r(x, y, z), airlib_quat);
}
void AirsimROSWrapper::MavrosPoseCmdCb(
    const mavros_msgs::AttitudeTarget::ConstPtr &msg) {
    uint8_t mask = msg->type_mask;
    if (mask == mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE) {
        get_multirotor_client()->moveByAngleRatesThrottleAsync(
            msg->body_rate.x, -msg->body_rate.y, -msg->body_rate.z, msg->thrust,
            vel_cmd_duration_, mavrosVechileName);
    } else if (mask == (mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE)) {
        double qw = msg->orientation.w;
        double qx = msg->orientation.x;
        double qy = -msg->orientation.y;
        double qz = -msg->orientation.z;
        double ySpr = qy * qy;
        double t0 = +2.0 * (qw * qx + qy * qz);
        double t1 = +1.0 - 2.0 * (qx * qx + ySpr);
        double roll = std::atan2(t0, t1);
        double t2 = +2.0 * (qw * qy - qz * qx);
        if (t2 > 1.0) {
            t2 = 1;
        }
        if (t2 < -1.0) {
            t2 = -1.0;
        }
        double pitch = std::asin(t2);

        double t3 = +2.0 * (qw * qz + qx * qy);
        double t4 = +1.0 - 2.0 * (ySpr + qz * qz);
        double yaw = std::atan2(t3, t4);

        double throttle = msg->thrust;
        get_multirotor_client()->moveByRollPitchYawThrottleAsync(
            roll, pitch, yaw, throttle, vel_cmd_duration_, mavrosVechileName);
    }
}
void AirsimROSWrapper::pose_cmd_body_frame_cb(
    const airsim_ros_pkgs::PoseCmd::ConstPtr &msg,
    const std::string &vehicle_name) {
    long long t = std::chrono::system_clock().now().time_since_epoch().count();
    if (t - last_cmd_time > 10000000) {
        std::lock_guard<std::mutex> guard(drone_control_mutex_);

        auto drone = static_cast<MultiRotorROS *>(
            vehicle_name_ptr_map_[vehicle_name].get());

        drone->pose_cmd.roll = msg->roll;
        drone->pose_cmd.pitch = msg->pitch;
        drone->pose_cmd.yaw = msg->yaw;
        drone->pose_cmd.throttle = msg->throttle;
        // airsim uses degrees
        drone->has_pose_cmd = true;
        get_multirotor_client()->moveByRollPitchYawThrottleAsync(
            drone->pose_cmd.roll, drone->pose_cmd.pitch, drone->pose_cmd.yaw,
            drone->pose_cmd.throttle, vel_cmd_duration_, drone->vehicle_name);
        last_cmd_time = t;
    }
}

void AirsimROSWrapper::vel_cmd_body_frame_cb(
    const airsim_ros_pkgs::VelCmd::ConstPtr &msg,
    const std::string &vehicle_name) {
    long long t = std::chrono::system_clock().now().time_since_epoch().count();
    if (t - last_cmd_time > 10000000) {
        std::lock_guard<std::mutex> guard(drone_control_mutex_);

        auto drone = static_cast<MultiRotorROS *>(
            vehicle_name_ptr_map_[vehicle_name].get());

        double roll, pitch, yaw;
        tf2::Matrix3x3(
            get_tf2_quat(
                drone->curr_drone_state.kinematics_estimated.pose.orientation))
            .getRPY(roll, pitch, yaw); // ros uses xyzw

        // todo do actual body frame?
        drone->vel_cmd.x = (msg->twist.linear.x * cos(yaw)) -
                           (msg->twist.linear.y *
                            sin(yaw)); // body frame assuming zero pitch roll
        drone->vel_cmd.y = (msg->twist.linear.x * sin(yaw)) +
                           (msg->twist.linear.y * cos(yaw)); // body frame
        drone->vel_cmd.z = msg->twist.linear.z;
        drone->vel_cmd.drivetrain =
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
        drone->vel_cmd.yaw_mode.is_rate = true;
        // airsim uses degrees
        drone->vel_cmd.yaw_mode.yaw_or_rate =
            math_common::rad2deg(msg->twist.angular.z);
        drone->has_vel_cmd = true;
        get_multirotor_client()->moveByVelocityAsync(
            drone->vel_cmd.x, drone->vel_cmd.y, drone->vel_cmd.z,
            vel_cmd_duration_, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            drone->vel_cmd.yaw_mode, drone->vehicle_name);
        last_cmd_time = t;
    }
}

void AirsimROSWrapper::angle_rate_throttle_frame_cb(
    const airsim_ros_pkgs::AngleRateThrottle::ConstPtr &msg,
    const std::string &vehicle_name) {
    long long t = std::chrono::system_clock().now().time_since_epoch().count();
    if (t - last_cmd_time > 10000000) {

        std::lock_guard<std::mutex> guard(drone_control_mutex_);
        auto drone = static_cast<MultiRotorROS *>(
            vehicle_name_ptr_map_[vehicle_name].get());

        drone->angle_rate_throttle_cmd.rollRate = msg->rollRate;
        drone->angle_rate_throttle_cmd.pitchRate = msg->pitchRate;
        drone->angle_rate_throttle_cmd.yawRate = msg->yawRate;
        drone->angle_rate_throttle_cmd.throttle = msg->throttle;
        drone->has_angle_rate_throttle_cmd = true;
        get_multirotor_client()->moveByAngleRatesThrottleAsync(
            drone->angle_rate_throttle_cmd.rollRate,
            drone->angle_rate_throttle_cmd.pitchRate,
            drone->angle_rate_throttle_cmd.yawRate,
            drone->angle_rate_throttle_cmd.throttle, vel_cmd_duration_,
            drone->vehicle_name);
        last_cmd_time = t;
    }
}

// todo support multiple gimbal commands
// 1. find quaternion of default gimbal pose
// 2. forward multiply with quaternion equivalent to desired euler commands (in
// degrees)
// 3. call airsim client's setCameraPose which sets camera pose wrt world (or
// takeoff?) ned frame. todo
void AirsimROSWrapper::gimbal_angle_euler_cmd_cb(
    const airsim_ros_pkgs::GimbalAngleEulerCmd &gimbal_angle_euler_cmd_msg) {
    try {
        tf2::Quaternion quat_control_cmd;
        quat_control_cmd.setRPY(
            math_common::deg2rad(gimbal_angle_euler_cmd_msg.roll),
            math_common::deg2rad(gimbal_angle_euler_cmd_msg.pitch),
            math_common::deg2rad(gimbal_angle_euler_cmd_msg.yaw));
        quat_control_cmd.normalize();
        gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd);
        gimbal_cmd_.camera_name = gimbal_angle_euler_cmd_msg.camera_name;
        gimbal_cmd_.vehicle_name = gimbal_angle_euler_cmd_msg.vehicle_name;
        has_gimbal_cmd_ = true;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
}

nav_msgs::Odometry AirsimROSWrapper::get_odom_msg_from_multirotor_state(
    const msr::airlib::MultirotorState &drone_state) const {
    nav_msgs::Odometry odom_msg;

    odom_msg.pose.pose.position.x = drone_state.getPosition().x();
    odom_msg.pose.pose.position.y = drone_state.getPosition().y();
    odom_msg.pose.pose.position.z = drone_state.getPosition().z();
    odom_msg.pose.pose.orientation.x = drone_state.getOrientation().x();
    odom_msg.pose.pose.orientation.y = drone_state.getOrientation().y();
    odom_msg.pose.pose.orientation.z = drone_state.getOrientation().z();
    odom_msg.pose.pose.orientation.w = drone_state.getOrientation().w();

    odom_msg.twist.twist.linear.x =
        drone_state.kinematics_estimated.twist.linear.x();
    odom_msg.twist.twist.linear.y =
        drone_state.kinematics_estimated.twist.linear.y();
    odom_msg.twist.twist.linear.z =
        drone_state.kinematics_estimated.twist.linear.z();
    odom_msg.twist.twist.angular.x =
        drone_state.kinematics_estimated.twist.angular.x();
    odom_msg.twist.twist.angular.y =
        drone_state.kinematics_estimated.twist.angular.y();
    odom_msg.twist.twist.angular.z =
        drone_state.kinematics_estimated.twist.angular.z();

    if (isENU_) {
        std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
        odom_msg.pose.pose.position.z = -odom_msg.pose.pose.position.z;
        std::swap(odom_msg.pose.pose.orientation.x,
                  odom_msg.pose.pose.orientation.y);
        odom_msg.pose.pose.orientation.z = -odom_msg.pose.pose.orientation.z;
        std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
        odom_msg.twist.twist.linear.z = -odom_msg.twist.twist.linear.z;
        std::swap(odom_msg.twist.twist.angular.x,
                  odom_msg.twist.twist.angular.y);
        odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;
    }

    return odom_msg;
}

// todo covariances
sensor_msgs::Imu AirsimROSWrapper::get_imu_msg_from_airsim(
    const msr::airlib::ImuBase::Output &imu_data) const {
    sensor_msgs::Imu imu_msg;
    // imu_msg.header.frame_id = "/airsim/odom_local_ned";// todo multiple
    // drones
    imu_msg.header.stamp = airsim_timestamp_to_ros(imu_data.time_stamp);
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();

    // todo radians per second
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z();

    // meters/s2^m
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();

    return imu_msg;
}

ros::Time AirsimROSWrapper::chrono_timestamp_to_ros(
    const std::chrono::system_clock::time_point &stamp) const {
    auto dur = std::chrono::duration<double>(stamp.time_since_epoch());
    ros::Time cur_time;
    cur_time.fromSec(dur.count());
    return cur_time;
}

ros::Time AirsimROSWrapper::airsim_timestamp_to_ros(
    const msr::airlib::TTimePoint &stamp) const {
    // airsim appears to use chrono::system_clock with nanosecond precision
    std::chrono::nanoseconds dur(stamp);
    std::chrono::time_point<std::chrono::system_clock> tp(dur);
    ros::Time cur_time = chrono_timestamp_to_ros(tp);
    return cur_time;
}

msr::airlib::MultirotorRpcLibClient *AirsimROSWrapper::get_multirotor_client() {
    return static_cast<msr::airlib::MultirotorRpcLibClient *>(
        airsim_client_.get());
}

void AirsimROSWrapper::drone_state_timer_cb(const ros::TimerEvent &event) {
    try {
        // todo this is global origin
        // origin_geo_point_pub_.publish(origin_geo_point_msg_);

        // get the basic vehicle pose and environmental state
        const auto now = update_state();

        // on init, will publish 0 to /clock as expected for use_sim_time
        // compatibility
        if (!airsim_client_states_.simIsPaused()) {
            // airsim_client needs to provide the simulation time in a future
            // version of the API
            ros_clock_.clock = now;
        }
        // publish the simulation clock
        if (publish_clock_) {
            clock_pub_.publish(ros_clock_);
        }

        // publish vehicle state, odom, and all basic sensor types
        publish_vehicle_state();

        // send any commands out to the vehicles
        // update_commands();
    } catch (rpc::rpc_error &e) {
        std::cout << "error" << std::endl;
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API:" << std::endl
                  << msg << std::endl;
    }
}

ros::Time AirsimROSWrapper::update_state() {
    bool got_sim_time = false;
    ros::Time curr_ros_time = ros::Time::now();

    // should be easier way to get the sim time through API, something like:
    // msr::airlib::Environment::State env =
    // airsim_client_->simGetGroundTruthEnvironment(""); curr_ros_time =
    // airsim_timestamp_to_ros(env.clock().nowNanos());

    // iterate over drones
    for (auto &vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        ros::Time vehicle_time;
        // get drone state from airsim
        auto &vehicle_ros = vehicle_name_ptr_pair.second;

        // vehicle environment, we can get ambient temperature here and other
        // truths auto env_data =
        // airsim_client_->simGetGroundTruthEnvironment(vehicle_ros->vehicle_name);

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            auto drone = static_cast<MultiRotorROS *>(vehicle_ros.get());
            drone->curr_drone_state = airsim_client_states_.getMultirotorState(
                vehicle_ros->vehicle_name);

            vehicle_time =
                airsim_timestamp_to_ros(drone->curr_drone_state.timestamp);
            if (!got_sim_time) {
                curr_ros_time = vehicle_time;
                got_sim_time = true;
            }

            // vehicle_ros->gps_sensor_msg =
            // get_gps_sensor_msg_from_airsim_geo_point(drone->curr_drone_state.gps_location);
            // vehicle_ros->gps_sensor_msg.header.stamp = vehicle_time;

            vehicle_ros->curr_odom =
                get_odom_msg_from_multirotor_state(drone->curr_drone_state);
        } else {
            // auto car = static_cast<CarROS*>(vehicle_ros.get());
            // car->curr_car_state =
            // get_car_client()->getCarState(vehicle_ros->vehicle_name);

            // vehicle_time =
            // airsim_timestamp_to_ros(car->curr_car_state.timestamp); if
            // (!got_sim_time) {
            //     curr_ros_time = vehicle_time;
            //     got_sim_time = true;
            // }

            // // vehicle_ros->gps_sensor_msg =
            // get_gps_sensor_msg_from_airsim_geo_point(env_data.geo_point);
            // // vehicle_ros->gps_sensor_msg.header.stamp = vehicle_time;

            // vehicle_ros->curr_odom =
            // get_odom_msg_from_car_state(car->curr_car_state);

            // airsim_ros_pkgs::CarState state_msg =
            // get_roscarstate_msg_from_car_state(car->curr_car_state);
            // state_msg.header.frame_id = vehicle_ros->vehicle_name;
            // car->car_state_msg = state_msg;
        }

        vehicle_ros->stamp = vehicle_time;

        // airsim_ros_pkgs::Environment env_msg =
        // get_environment_msg_from_airsim(env_data); env_msg.header.frame_id =
        // vehicle_ros->vehicle_name; env_msg.header.stamp = vehicle_time;
        // vehicle_ros->env_msg = env_msg;

        // convert airsim drone state to ROS msgs
        vehicle_ros->curr_odom.header.frame_id = vehicle_ros->vehicle_name;
        vehicle_ros->curr_odom.child_frame_id = vehicle_ros->odom_frame_id;
        vehicle_ros->curr_odom.header.stamp = vehicle_time;
    }

    return curr_ros_time;
}

void AirsimROSWrapper::publish_vehicle_state() {
    for (auto &vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto &vehicle_ros = vehicle_name_ptr_pair.second;
        if (vehicle_ros->vehicle_name != mavrosVechileName) {
            continue;
        }
        // simulation environment truth
        // vehicle_ros->env_pub.publish(vehicle_ros->env_msg);

        if (airsim_mode_ == AIRSIM_MODE::CAR) {
            // dashboard reading from car, RPM, gear, etc
            auto car = static_cast<CarROS *>(vehicle_ros.get());
            car->car_state_pub.publish(car->car_state_msg);
        }

        auto collision_info = airsim_client_->simGetCollisionInfo();
        if (collision_info.has_collided) {
            std_msgs::Bool collision_msg;
            collision_msg.data = true;
            collision_info_pub_.publish(collision_msg);
        } else {
            std_msgs::Bool collision_msg;
            collision_msg.data = false;
            collision_info_pub_.publish(collision_msg);
        }
        // odom and transforms
        vehicle_ros->odom_local_pub.publish(vehicle_ros->curr_odom);
        nav_msgs::Odometry msgPub;
        msgPub.header = vehicle_ros->curr_odom.header;
        msgPub.pose.pose.orientation.w =
            vehicle_ros->curr_odom.pose.pose.orientation.w;
        msgPub.pose.pose.orientation.x =
            vehicle_ros->curr_odom.pose.pose.orientation.x;
        msgPub.pose.pose.orientation.y =
            -vehicle_ros->curr_odom.pose.pose.orientation.y;
        msgPub.pose.pose.orientation.z =
            -vehicle_ros->curr_odom.pose.pose.orientation.z;
        msgPub.pose.pose.position.x =
            vehicle_ros->curr_odom.pose.pose.position.x;
        msgPub.pose.pose.position.y =
            -vehicle_ros->curr_odom.pose.pose.position.y;
        msgPub.pose.pose.position.z =
            -vehicle_ros->curr_odom.pose.pose.position.z;
        msgPub.twist.twist.angular.x =
            vehicle_ros->curr_odom.twist.twist.angular.x;
        msgPub.twist.twist.angular.y =
            -vehicle_ros->curr_odom.twist.twist.angular.y;
        msgPub.twist.twist.angular.z =
            -vehicle_ros->curr_odom.twist.twist.angular.z;
        msgPub.twist.twist.linear.x =
            vehicle_ros->curr_odom.twist.twist.linear.x;
        msgPub.twist.twist.linear.y =
            -vehicle_ros->curr_odom.twist.twist.linear.y;
        msgPub.twist.twist.linear.z =
            -vehicle_ros->curr_odom.twist.twist.linear.z;
        mPubLocalOdomCmd.publish(msgPub);
        // publish_odom_tf(vehicle_ros->curr_odom);

        // ground truth GPS position from sim/HITL
        // vehicle_ros->global_gps_pub.publish(vehicle_ros->gps_sensor_msg);

        for (auto &sensor_publisher : vehicle_ros->sensor_pubs) {
            switch (sensor_publisher.sensor_type) {
            case SensorBase::SensorType::Barometer: {
                // auto baro_data =
                // airsim_client_->getBarometerData(sensor_publisher.sensor_name,
                // vehicle_ros->vehicle_name); airsim_ros_pkgs::Altimeter
                // alt_msg = get_altimeter_msg_from_airsim(baro_data);
                // alt_msg.header.frame_id = vehicle_ros->vehicle_name;
                // sensor_publisher.publisher.publish(alt_msg);
                break;
            }
            case SensorBase::SensorType::Imu: {
                auto imu_data = airsim_client_states_.getImuData(
                    sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
                sensor_msgs::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
                imu_msg.header.frame_id = vehicle_ros->vehicle_name;
                // sensor_publisher.publisher.publish(imu_msg);
                imu_msg.angular_velocity.y *= -1.;
                imu_msg.angular_velocity.z *= -1.;
                imu_msg.linear_acceleration.y *= -1.;
                imu_msg.linear_acceleration.z *= -1.;
                imu_msg.orientation.y *= -1.;
                imu_msg.orientation.z *= -1.;
                mPubMavrosImuCmd.publish(imu_msg);
                break;
            }
            case SensorBase::SensorType::Distance: {
                // auto distance_data =
                // airsim_client_->getDistanceSensorData(sensor_publisher.sensor_name,
                // vehicle_ros->vehicle_name); sensor_msgs::Range dist_msg =
                // get_range_from_airsim(distance_data);
                // dist_msg.header.frame_id = vehicle_ros->vehicle_name;
                // sensor_publisher.publisher.publish(dist_msg);
                break;
            }
            case SensorBase::SensorType::Gps: {
                // auto gps_data =
                // airsim_client_->getGpsData(sensor_publisher.sensor_name,
                // vehicle_ros->vehicle_name); sensor_msgs::NavSatFix gps_msg =
                // get_gps_msg_from_airsim(gps_data); gps_msg.header.frame_id =
                // vehicle_ros->vehicle_name;
                // sensor_publisher.publisher.publish(gps_msg);
                break;
            }
            case SensorBase::SensorType::Lidar: {
                // handled via callback
                break;
            }
            case SensorBase::SensorType::Magnetometer: {
                // auto mag_data =
                // airsim_client_->getMagnetometerData(sensor_publisher.sensor_name,
                // vehicle_ros->vehicle_name); sensor_msgs::MagneticField
                // mag_msg = get_mag_msg_from_airsim(mag_data);
                // mag_msg.header.frame_id = vehicle_ros->vehicle_name;
                // sensor_publisher.publisher.publish(mag_msg);
                break;
            }
            }
        }

        // update_and_publish_static_transforms(vehicle_ros.get());
    }
}

// airsim uses nans for zeros in settings.json. we set them to zeros here for
// handling tfs in ROS
void AirsimROSWrapper::set_nans_to_zeros_in_pose(
    VehicleSetting &vehicle_setting) const {
    if (std::isnan(vehicle_setting.position.x()))
        vehicle_setting.position.x() = 0.0;

    if (std::isnan(vehicle_setting.position.y()))
        vehicle_setting.position.y() = 0.0;

    if (std::isnan(vehicle_setting.position.z()))
        vehicle_setting.position.z() = 0.0;

    if (std::isnan(vehicle_setting.rotation.yaw))
        vehicle_setting.rotation.yaw = 0.0;

    if (std::isnan(vehicle_setting.rotation.pitch))
        vehicle_setting.rotation.pitch = 0.0;

    if (std::isnan(vehicle_setting.rotation.roll))
        vehicle_setting.rotation.roll = 0.0;
}

// if any nan's in camera pose, set them to match vehicle pose (which has
// already converted any potential nans to zeros)
void AirsimROSWrapper::set_nans_to_zeros_in_pose(
    const VehicleSetting &vehicle_setting,
    CameraSetting &camera_setting) const {
    if (std::isnan(camera_setting.position.x()))
        camera_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(camera_setting.position.y()))
        camera_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(camera_setting.position.z()))
        camera_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(camera_setting.rotation.yaw))
        camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(camera_setting.rotation.pitch))
        camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(camera_setting.rotation.roll))
        camera_setting.rotation.roll = vehicle_setting.rotation.roll;
}

void AirsimROSWrapper::img_response_RGBD_timer_cb(
    const ros::TimerEvent &event) {
    try {
        const std::vector<ImageResponse> &img_response =
            airsim_client_images_.simGetImages(
                RGBD_request_,
                airsim_img_request_vehicle_name_pair_vec_[0].second);
        if (img_response.size() == RGBD_request_.size()) {
            ros::Time curr_ros_time = ros::Time::now();
            for (const auto &curr_img_response : img_response) {
                if (curr_img_response.image_type == ImageType::Scene) {
                    front_camera_info_.header.stamp =
                        airsim_timestamp_to_ros(curr_img_response.time_stamp);
                    front_cam_info_pub_.publish(front_camera_info_);
                    front_pub_.publish(get_img_msg_from_response(
                        curr_img_response, curr_ros_time,
                        curr_img_response.camera_name + "_optical"));
                } else if (curr_img_response.image_type ==
                           ImageType::DepthPlanar) {
                    front_depth_camera_info_.header.stamp =
                        airsim_timestamp_to_ros(curr_img_response.time_stamp);
                    front_depth_cam_info_pub_.publish(front_depth_camera_info_);
                    front_depth_pub_.publish(get_depth_img_msg_from_response(
                        curr_img_response, curr_ros_time,
                        curr_img_response.camera_name + "_optical"));
                } else if (curr_img_response.image_type ==
                           ImageType::OpticalFlow) {
                    front_opti_pub_.publish(get_depth_img_msg_from_response(
                        curr_img_response, curr_ros_time,
                        curr_img_response.camera_name + "_optical"));
                }
            }
        }
    }

    catch (rpc::rpc_error &e) {
        std::string msg = e.get_error().as<std::string>();
        ROS_ERROR("%s", msg.c_str());
    }
}

void AirsimROSWrapper::img_response_stereo_timer_cb(
    const ros::TimerEvent &event) {
    try {
        const std::vector<ImageResponse> &img_response =
            airsim_client_images_.simGetImages(
                stereo_request_,
                airsim_img_request_vehicle_name_pair_vec_[0].second);
        if (img_response.size() == stereo_request_.size()) {
            ros::Time curr_ros_time = ros::Time::now();
            for (const auto &curr_img_response : img_response) {
                if (strcmp(curr_img_response.camera_name.c_str(),
                           "front_left") == 0) {
                    front_left_camera_info_.header.stamp =
                        airsim_timestamp_to_ros(curr_img_response.time_stamp);
                    front_left_cam_info_pub_.publish(front_left_camera_info_);
                    front_left_pub_.publish(get_img_msg_from_response(
                        curr_img_response, curr_ros_time,
                        curr_img_response.camera_name + "_optical"));
                } else {
                    front_right_camera_info_.header.stamp =
                        airsim_timestamp_to_ros(curr_img_response.time_stamp);
                    front_right_cam_info_pub_.publish(front_right_camera_info_);
                    front_right_pub_.publish(get_img_msg_from_response(
                        curr_img_response, curr_ros_time,
                        curr_img_response.camera_name + "_optical"));
                }
            }
        }
    }

    catch (rpc::rpc_error &e) {
        std::string msg = e.get_error().as<std::string>();
        ROS_ERROR("%s", msg.c_str());
    }
}

void AirsimROSWrapper::img_response_bottom_timer_cb(
    const ros::TimerEvent &event) {
    try {
        const std::vector<ImageResponse> &img_response =
            airsim_client_images_.simGetImages(
                bottom_request_,
                airsim_img_request_vehicle_name_pair_vec_[0].second);
        if (img_response.size() == bottom_request_.size()) {
            ros::Time curr_ros_time = ros::Time::now();

            for (const auto &curr_img_response : img_response) {
                bottom_camera_info_.header.stamp =
                    airsim_timestamp_to_ros(curr_img_response.time_stamp);
                bottom_cam_info_pub_.publish(bottom_camera_info_);
                bottom_pub_.publish(get_img_msg_from_response(
                    curr_img_response, curr_ros_time,
                    curr_img_response.camera_name + "_optical"));
            }
        }
    }

    catch (rpc::rpc_error &e) {
        std::string msg = e.get_error().as<std::string>();
        ROS_ERROR("%s", msg.c_str());
    }
}

cv::Mat
AirsimROSWrapper::manual_decode_depth(const ImageResponse &img_response) const {
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1,
                cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) =
                img_response.image_data_float[row * img_width + col];
    return mat;
}

sensor_msgs::ImagePtr
AirsimROSWrapper::get_img_msg_from_response(const ImageResponse &img_response,
                                            const ros::Time curr_ros_time,
                                            const std::string frame_id) {
    sensor_msgs::ImagePtr img_msg_ptr =
        boost::make_shared<sensor_msgs::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step =
        img_response.width * 3; // todo un-hardcode. image_width*num_bytes
    img_msg_ptr->header.stamp =
        airsim_timestamp_to_ros(img_response.time_stamp);
    img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    if (is_vulkan_)
        img_msg_ptr->encoding = "rgb8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

sensor_msgs::ImagePtr AirsimROSWrapper::get_depth_img_msg_from_response(
    const ImageResponse &img_response, const ros::Time curr_ros_time,
    const std::string frame_id) {
    // todo using img_response.image_data_float direclty as done
    // get_img_msg_from_response() throws an error, hence the dependency on
    // opencv and cv_bridge. however, this is an extremely fast op, so no big
    // deal.
    cv::Mat depth_img = manual_decode_depth(img_response);
    cv::Mat noise_image(depth_img.size(), CV_32F);
    cv::randn(noise_image, 0, depth_std_dev);
    cv::add(depth_img, noise_image, depth_img, cv::noArray(), CV_32F);
    // cv::threshold(depth_img, depth_img, 10, 10, cv::THRESH_TOZERO_INV);
    sensor_msgs::ImagePtr depth_img_msg =
        cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    depth_img_msg->header.stamp =
        airsim_timestamp_to_ros(img_response.time_stamp);
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}

// todo have a special stereo pair mode and get projection matrix by calculating
// offset wrt drone body frame?
sensor_msgs::CameraInfo AirsimROSWrapper::generate_cam_info(
    const std::string &camera_name, const CameraSetting &camera_setting,
    const CaptureSetting &capture_setting) const {
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "_optical";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    float f_x = (capture_setting.width / 2.0) /
                tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    // todo focal length in Y direction should be same as X it seems. this can
    // change in future a scene capture component which exactly correponds to a
    // cine camera float f_y = (capture_setting.height / 2.0) /
    // tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.K = {f_x, 0.0, capture_setting.width / 2.0,
                      0.0, f_x, capture_setting.height / 2.0,
                      0.0, 0.0, 1.0};
    cam_info_msg.P = {f_x,
                      0.0,
                      capture_setting.width / 2.0,
                      0.0,
                      0.0,
                      f_x,
                      capture_setting.height / 2.0,
                      0.0,
                      0.0,
                      0.0,
                      1.0,
                      0.0};
    return cam_info_msg;
}

void AirsimROSWrapper::process_and_publish_img_response(
    const std::vector<ImageResponse> &img_response_vec,
    const int img_response_idx, const std::string &vehicle_name) {
    // todo add option to use airsim time (image_response.TTimePoint) like
    // Gazebo /use_sim_time param
    ros::Time curr_ros_time = ros::Time::now();
    int img_response_idx_internal = img_response_idx;

    for (const auto &curr_img_response : img_response_vec) {
        // todo publishing a tf for each capture type seems stupid. but it
        // foolproofs us against render thread's async stuff, I hope. Ideally,
        // we should loop over cameras and then captures, and publish only one
        // tf. publish_camera_tf(curr_img_response, curr_ros_time, vehicle_name,
        // curr_img_response.camera_name);

        // todo simGetCameraInfo is wrong + also it's only for image type -1.
        // msr::airlib::CameraInfo camera_info =
        // airsim_client_.simGetCameraInfo(curr_img_response.camera_name);

        // update timestamp of saved cam info msgs

        camera_info_msg_vec_[img_response_idx_internal].header.stamp =
            airsim_timestamp_to_ros(curr_img_response.time_stamp);
        cam_info_pub_vec_[img_response_idx_internal].publish(
            camera_info_msg_vec_[img_response_idx_internal]);

        // DepthPlanar / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float) {
            image_pub_vec_[img_response_idx_internal].publish(
                get_depth_img_msg_from_response(
                    curr_img_response, curr_ros_time,
                    curr_img_response.camera_name + "_optical"));
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else {
            image_pub_vec_[img_response_idx_internal].publish(
                get_img_msg_from_response(curr_img_response, curr_ros_time,
                                          curr_img_response.camera_name +
                                              "_optical"));
        }
        img_response_idx_internal++;
    }
}
void AirsimROSWrapper::convert_yaml_to_simple_mat(const YAML::Node &node,
                                                  SimpleMatrix &m) const {
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node &data = node["data"];
    for (int i = 0; i < rows * cols; ++i) {
        m.data[i] = data[i].as<double>();
    }
}

void AirsimROSWrapper::read_params_from_yaml_and_fill_cam_info_msg(
    const std::string &file_name, sensor_msgs::CameraInfo &cam_info) const {
    std::ifstream fin(file_name.c_str());
    YAML::Node doc = YAML::Load(fin);

    cam_info.width = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

    SimpleMatrix K_(3, 3, &cam_info.K[0]);
    convert_yaml_to_simple_mat(doc[K_YML_NAME], K_);
    SimpleMatrix R_(3, 3, &cam_info.R[0]);
    convert_yaml_to_simple_mat(doc[R_YML_NAME], R_);
    SimpleMatrix P_(3, 4, &cam_info.P[0]);
    convert_yaml_to_simple_mat(doc[P_YML_NAME], P_);

    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

    const YAML::Node &D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_rows = D_node["rows"].as<int>();
    D_cols = D_node["cols"].as<int>();
    const YAML::Node &D_data = D_node["data"];
    cam_info.D.resize(D_rows * D_cols);
    for (int i = 0; i < D_rows * D_cols; ++i) {
        cam_info.D[i] = D_data[i].as<float>();
    }
}
