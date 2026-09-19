#include <furosim_ros_wrapper.h>
#include "common/AirSimSettings.hpp"
#if __has_include(<tf2_sensor_msgs/tf2_sensor_msgs.hpp>)
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#endif

using namespace std::placeholders;

constexpr char AirsimROSWrapper::CAM_YML_NAME[];
constexpr char AirsimROSWrapper::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapper::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapper::K_YML_NAME[];
constexpr char AirsimROSWrapper::D_YML_NAME[];
constexpr char AirsimROSWrapper::R_YML_NAME[];
constexpr char AirsimROSWrapper::P_YML_NAME[];
constexpr char AirsimROSWrapper::DMODEL_YML_NAME[];

const std::unordered_map<int, std::string> AirsimROSWrapper::image_type_int_to_string_map_ = {
    { 0, "Scene" },
    { 1, "DepthPlanar" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" },
    { 8, "OpticalFlow" },
    { 9, "OpticalFlowVis" },
    { 10, "SegmentationID" }
};

AirsimROSWrapper::AirsimROSWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_img, const std::shared_ptr<rclcpp::Node> nh_lidar, const std::string& host_ip)
    : is_used_lidar_timer_cb_queue_(false)
    , is_used_img_timer_cb_queue_(false)
    , is_used_gpu_sonar_timer_cb_queue_(false)
    , is_used_gpu_sidescan_sonar_timer_cb_queue_(false)
    , airsim_settings_parser_(host_ip)
    , host_ip_(host_ip)
    , airsim_client_(nullptr)
    , airsim_client_images_(host_ip)
    , airsim_client_lidar_(host_ip)
    , airsim_client_gpu_sonar_(host_ip)
    , airsim_client_gpu_sidescan_sonar_(host_ip)
    , nh_(nh)
    , nh_img_(nh_img)
    , nh_lidar_(nh_lidar)
    , isENU_(false)
    , publish_clock_(false)
{
    ros_clock_.clock = rclcpp::Time(0);

    if (AirSimSettings::singleton().simmode_name == AirSimSettings::kSimModeTypeAuv) {
        airsim_mode_ = AIRSIM_MODE::AUV;
        RCLCPP_INFO(nh_->get_logger(), "Setting ROS wrapper to AUV mode");
    }
    else if (AirSimSettings::singleton().simmode_name != AirSimSettings::kSimModeTypeCar) {
        airsim_mode_ = AIRSIM_MODE::DRONE;
        RCLCPP_INFO(nh_->get_logger(), "Setting ROS wrapper to DRONE mode");
    }
    else {
        airsim_mode_ = AIRSIM_MODE::CAR;
        RCLCPP_INFO(nh_->get_logger(), "Setting ROS wrapper to CAR mode");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nh_);

    initialize_ros();

    RCLCPP_INFO(nh_->get_logger(), "AirsimROSWrapper Initialized!");
}

void AirsimROSWrapper::initialize_airsim()
{
    try {

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::MultirotorRpcLibClient(host_ip_));
        }
        else if (airsim_mode_ == AIRSIM_MODE::AUV) {
            airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::AuvRpcLibClient(host_ip_));
        }
        else {
            airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::CarRpcLibClient(host_ip_));
        }
        airsim_client_->confirmConnection();
        airsim_client_images_.confirmConnection();
        airsim_client_lidar_.confirmConnection();
        airsim_client_gpu_sonar_.confirmConnection();
        airsim_client_gpu_sidescan_sonar_.confirmConnection();

        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            airsim_client_->enableApiControl(true, vehicle_name_ptr_pair.first);
            airsim_client_->armDisarm(true, vehicle_name_ptr_pair.first);
        }

        origin_geo_point_ = get_origin_geo_point();
        origin_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(origin_geo_point_);
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, something went wrong.\n%s", msg.c_str());
        rclcpp::shutdown();
    }
}

void AirsimROSWrapper::initialize_ros()
{
    // ros params
    double update_airsim_control_every_n_sec;
    nh_->get_parameter("is_vulkan", is_vulkan_);
    nh_->get_parameter("update_airsim_control_every_n_sec", update_airsim_control_every_n_sec);
    nh_->get_parameter("publish_clock", publish_clock_);
    nh_->get_parameter_or("world_frame_id", world_frame_id_, world_frame_id_);
    odom_frame_id_ = world_frame_id_ == AIRSIM_FRAME_ID ? AIRSIM_ODOM_FRAME_ID : ENU_ODOM_FRAME_ID;
    nh_->get_parameter_or("odom_frame_id", odom_frame_id_, odom_frame_id_);
    isENU_ = (odom_frame_id_ == ENU_ODOM_FRAME_ID);
    nh_->get_parameter_or("coordinate_system_enu", isENU_, isENU_);
    vel_cmd_duration_ = 0.05;

    nh_->declare_parameter("vehicle_name", rclcpp::ParameterValue(""));
    create_ros_pubs_from_settings_json();
    airsim_control_callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    airsim_control_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_airsim_control_every_n_sec), std::bind(&AirsimROSWrapper::drone_state_timer_cb, this), airsim_control_callback_group_);
}

void AirsimROSWrapper::create_ros_pubs_from_settings_json()
{
    gimbal_angle_quat_cmd_sub_ = nh_->create_subscription<furosim_interfaces::msg::GimbalAngleQuatCmd>("~/gimbal_angle_quat_cmd", 50, std::bind(&AirsimROSWrapper::gimbal_angle_quat_cmd_cb, this, _1));
    gimbal_angle_euler_cmd_sub_ = nh_->create_subscription<furosim_interfaces::msg::GimbalAngleEulerCmd>("~/gimbal_angle_euler_cmd", 50, std::bind(&AirsimROSWrapper::gimbal_angle_euler_cmd_cb, this, _1));
    origin_geo_point_pub_ = nh_->create_publisher<furosim_interfaces::msg::GPSYaw>("~/origin_geo_point", 10);

    airsim_img_request_vehicle_name_pair_vec_.clear();
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();
    vehicle_name_ptr_map_.clear();
    size_t lidar_cnt = 0;
    size_t gpu_sonar_cnt = 0;
    size_t gpu_sidescan_sonar_cnt = 0;

    image_transport::ImageTransport image_transporter(nh_);

    for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles) {
        auto& vehicle_setting = curr_vehicle_elem.second;
        auto curr_vehicle_name = curr_vehicle_elem.first;

        nh_->set_parameter(rclcpp::Parameter("vehicle_name", curr_vehicle_name));

        set_nans_to_zeros_in_pose(*vehicle_setting);

        std::unique_ptr<VehicleROS> vehicle_ros = nullptr;

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            vehicle_ros = std::unique_ptr<MultiRotorROS>(new MultiRotorROS());
        }
        else if (airsim_mode_ == AIRSIM_MODE::AUV) {
            vehicle_ros = std::unique_ptr<AuvROS>(new AuvROS());
        }
        else {
            vehicle_ros = std::unique_ptr<CarROS>(new CarROS());
        }

        vehicle_ros->odom_frame_id_ = curr_vehicle_name + "/" + odom_frame_id_;
        vehicle_ros->vehicle_name_ = curr_vehicle_name;

        append_static_vehicle_tf(vehicle_ros.get(), *vehicle_setting);

        const std::string topic_prefix = "~/" + curr_vehicle_name;
        vehicle_ros->odom_local_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>(topic_prefix + "/" + odom_frame_id_, 10);
        vehicle_ros->env_pub_ = nh_->create_publisher<furosim_interfaces::msg::Environment>(topic_prefix + "/environment", 10);
        vehicle_ros->global_gps_pub_ = nh_->create_publisher<sensor_msgs::msg::NavSatFix>(topic_prefix + "/global_gps", 10);

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());

            std::function<void(const furosim_interfaces::msg::VelCmd::SharedPtr)> fcn_vel_cmd_body_frame_sub = std::bind(&AirsimROSWrapper::vel_cmd_body_frame_cb, this, _1, vehicle_ros->vehicle_name_);
            drone->vel_cmd_body_frame_sub_ = nh_->create_subscription<furosim_interfaces::msg::VelCmd>(topic_prefix + "/vel_cmd_body_frame", 1, fcn_vel_cmd_body_frame_sub);

            std::function<void(const furosim_interfaces::msg::VelCmd::SharedPtr)> fcn_vel_cmd_world_frame_sub = std::bind(&AirsimROSWrapper::vel_cmd_world_frame_cb, this, _1, vehicle_ros->vehicle_name_);
            drone->vel_cmd_world_frame_sub_ = nh_->create_subscription<furosim_interfaces::msg::VelCmd>(topic_prefix + "/vel_cmd_world_frame", 1, fcn_vel_cmd_world_frame_sub);

            std::function<bool(std::shared_ptr<furosim_interfaces::srv::Takeoff::Request>, std::shared_ptr<furosim_interfaces::srv::Takeoff::Response>)> fcn_takeoff_srvr = std::bind(&AirsimROSWrapper::takeoff_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
            drone->takeoff_srvr_ = nh_->create_service<furosim_interfaces::srv::Takeoff>(topic_prefix + "/takeoff", fcn_takeoff_srvr);

            std::function<bool(std::shared_ptr<furosim_interfaces::srv::Land::Request>, std::shared_ptr<furosim_interfaces::srv::Land::Response>)> fcn_land_srvr = std::bind(&AirsimROSWrapper::land_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
            drone->land_srvr_ = nh_->create_service<furosim_interfaces::srv::Land>(topic_prefix + "/land", fcn_land_srvr);
        }
        else if (airsim_mode_ == AIRSIM_MODE::AUV) {
            auto auv = static_cast<AuvROS*>(vehicle_ros.get());
            auv->has_force_cmd_ = false;

            std::function<void(const furosim_interfaces::msg::VelCmd::SharedPtr)> fcn_auv_cmd_sub = std::bind(&AirsimROSWrapper::auv_cmd_cb, this, _1, vehicle_ros->vehicle_name_);
            auv->auv_cmd_sub_ = nh_->create_subscription<furosim_interfaces::msg::VelCmd>(topic_prefix + "/force_cmd", 1, fcn_auv_cmd_sub);

            std::function<void(const geometry_msgs::msg::Vector3Stamped::SharedPtr)> fcn_ocean_current_sub = std::bind(&AirsimROSWrapper::ocean_current_cb, this, _1, vehicle_ros->vehicle_name_);
            auv->ocean_current_sub_ = nh_->create_subscription<geometry_msgs::msg::Vector3Stamped>(topic_prefix + "/ocean_current", 1, fcn_ocean_current_sub);

            std::function<bool(std::shared_ptr<furosim_interfaces::srv::MoveToPosition::Request>, std::shared_ptr<furosim_interfaces::srv::MoveToPosition::Response>)> fcn_move_to_position = std::bind(&AirsimROSWrapper::auv_move_to_position_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
            auv->move_to_position_srvr_ = nh_->create_service<furosim_interfaces::srv::MoveToPosition>(topic_prefix + "/move_to_position", fcn_move_to_position);

            std::function<bool(std::shared_ptr<furosim_interfaces::srv::MoveOnPath::Request>, std::shared_ptr<furosim_interfaces::srv::MoveOnPath::Response>)> fcn_move_on_path = std::bind(&AirsimROSWrapper::auv_move_on_path_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
            auv->move_on_path_srvr_ = nh_->create_service<furosim_interfaces::srv::MoveOnPath>(topic_prefix + "/move_on_path", fcn_move_on_path);

            std::function<bool(std::shared_ptr<furosim_interfaces::srv::MoveToZ::Request>, std::shared_ptr<furosim_interfaces::srv::MoveToZ::Response>)> fcn_move_to_z = std::bind(&AirsimROSWrapper::auv_move_to_z_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
            auv->move_to_z_srvr_ = nh_->create_service<furosim_interfaces::srv::MoveToZ>(topic_prefix + "/move_to_z", fcn_move_to_z);

            std::function<bool(std::shared_ptr<furosim_interfaces::srv::Hover::Request>, std::shared_ptr<furosim_interfaces::srv::Hover::Response>)> fcn_hover = std::bind(&AirsimROSWrapper::auv_hover_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
            auv->hover_srvr_ = nh_->create_service<furosim_interfaces::srv::Hover>(topic_prefix + "/hover", fcn_hover);
        }
        else {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            std::function<void(const furosim_interfaces::msg::CarControls::SharedPtr)> fcn_car_cmd_sub = std::bind(&AirsimROSWrapper::car_cmd_cb, this, _1, vehicle_ros->vehicle_name_);
            car->car_cmd_sub_ = nh_->create_subscription<furosim_interfaces::msg::CarControls>(topic_prefix + "/car_cmd", 1, fcn_car_cmd_sub);
            car->car_state_pub_ = nh_->create_publisher<furosim_interfaces::msg::CarState>(topic_prefix + "/car_state", 10);
        }

        // iterate over camera map
        for (auto& curr_camera_elem : vehicle_setting->cameras) {
            auto& camera_setting = curr_camera_elem.second;
            auto& curr_camera_name = curr_camera_elem.first;

            set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
            append_static_camera_tf(vehicle_ros.get(), curr_camera_name, camera_setting);
            std::vector<ImageRequest> current_image_request_vec;
            current_image_request_vec.clear();

            for (const auto& curr_capture_elem : camera_setting.capture_settings) {
                auto& capture_setting = curr_capture_elem.second;

                // AirSimSettings::initializeCaptureSettings() pre-populates the map with default
                // CaptureSetting() for every ImageType slot, all of which have image_type=0 (Scene).
                // Only entries the user explicitly configured have key == capture_setting.image_type;
                // skipping the rest prevents duplicate Scene-topic advertisements.
                if (curr_capture_elem.first != capture_setting.image_type)
                    continue;

                if (!(std::isnan(capture_setting.fov_degrees))) {
                    ImageType curr_image_type = msr::airlib::Utils::toEnum<ImageType>(capture_setting.image_type);
                    if (curr_image_type == ImageType::Scene || curr_image_type == ImageType::Segmentation || curr_image_type == ImageType::SurfaceNormals || curr_image_type == ImageType::Infrared) {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false));
                    }
                    else {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, true));
                    }

                    const std::string camera_topic = topic_prefix + "/" + curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type);
                    image_pub_vec_.push_back(image_transporter.advertise(camera_topic, 1));
                    cam_info_pub_vec_.push_back(nh_->create_publisher<sensor_msgs::msg::CameraInfo>(camera_topic + "/camera_info", 10));
                    camera_info_msg_vec_.push_back(generate_cam_info(curr_camera_name, camera_setting, capture_setting));
                }
            }
            airsim_img_request_vehicle_name_pair_vec_.push_back(std::make_pair(current_image_request_vec, curr_vehicle_name));
        }

        // iterate over sensors
        for (auto& curr_sensor_map : vehicle_setting->sensors) {
            auto& sensor_name = curr_sensor_map.first;
            auto& sensor_setting = curr_sensor_map.second;

            if (sensor_setting->enabled) {

                switch (sensor_setting->sensor_type) {
                case SensorBase::SensorType::Barometer: {
                    SensorPublisher<furosim_interfaces::msg::Altimeter> sensor_publisher =
                        create_sensor_publisher<furosim_interfaces::msg::Altimeter>("Barometer", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/altimeter/" + sensor_name, 10);
                    vehicle_ros->barometer_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Imu: {
                    SensorPublisher<sensor_msgs::msg::Imu> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::Imu>("Imu", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/imu/" + sensor_name, 10);
                    vehicle_ros->imu_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Gps: {
                    SensorPublisher<sensor_msgs::msg::NavSatFix> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::NavSatFix>("Gps", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/gps/" + sensor_name, 10);
                    vehicle_ros->gps_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Magnetometer: {
                    SensorPublisher<sensor_msgs::msg::MagneticField> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::MagneticField>("Magnetometer", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/magnetometer/" + sensor_name, 10);
                    vehicle_ros->magnetometer_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Distance: {
                    SensorPublisher<sensor_msgs::msg::Range> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::Range>("Distance", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/distance/" + sensor_name, 10);
                    vehicle_ros->distance_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Lidar: {
                    auto lidar_setting = *static_cast<LidarSetting*>(sensor_setting.get());
                    msr::airlib::LidarSimpleParams params;
                    params.initializeFromSettings(lidar_setting);
                    append_static_lidar_tf(vehicle_ros.get(), sensor_name, params);

                    SensorPublisher<sensor_msgs::msg::PointCloud2> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::PointCloud2>("Lidar", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/lidar/" + sensor_name, 10);
                    vehicle_ros->lidar_pubs_.emplace_back(sensor_publisher);
                    lidar_cnt += 1;
                    break;
                }
                case SensorBase::SensorType::Pressure: {
                    RCLCPP_INFO_STREAM(nh_->get_logger(), sensor_name << ": Pressure sensor");
                    SensorPublisher<furosim_interfaces::msg::Pressure> sensor_publisher =
                        create_sensor_publisher<furosim_interfaces::msg::Pressure>("Pressure", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/pressure/" + sensor_name, 10);
                    vehicle_ros->pressure_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Dvl: {
                    RCLCPP_INFO_STREAM(nh_->get_logger(), sensor_name << ": DVL sensor");
                    SensorPublisher<furosim_interfaces::msg::DVL> sensor_publisher =
                        create_sensor_publisher<furosim_interfaces::msg::DVL>("Dvl", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/dvl/" + sensor_name, 10);
                    vehicle_ros->dvl_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::GpuSonar: {
                    RCLCPP_INFO_STREAM(nh_->get_logger(), sensor_name << ": GpuSonar (FLS)");
                    auto sonar_setting = *static_cast<GpuSonarSetting*>(sensor_setting.get());
                    msr::airlib::GpuSonarSimpleParams params;
                    params.initializeFromSettings(sonar_setting);
                    append_static_gpu_sonar_tf(vehicle_ros.get(), sensor_name, params);

                    SensorPublisher<sensor_msgs::msg::Image> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::Image>("GpuSonar", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/gpu_sonar/" + sensor_name, 10);
                    sensor_publisher.image_height = params.number_of_samples;
                    sensor_publisher.image_width = params.number_of_beams;
                    vehicle_ros->gpu_sonar_pubs_.emplace_back(sensor_publisher);
                    gpu_sonar_cnt += 1;
                    break;
                }
                case SensorBase::SensorType::GpuSidescanSonar: {
                    RCLCPP_INFO_STREAM(nh_->get_logger(), sensor_name << ": GpuSidescanSonar (SSS)");
                    auto sonar_setting = *static_cast<GpuSidescanSonarSetting*>(sensor_setting.get());
                    msr::airlib::GpuSidescanSonarSimpleParams params;
                    params.initializeFromSettings(sonar_setting);
                    append_static_gpu_sidescan_sonar_tf(vehicle_ros.get(), sensor_name, params);

                    SensorPublisher<sensor_msgs::msg::Image> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::Image>("GpuSidescanSonar", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/gpu_sidescan_sonar/" + sensor_name, 10);
                    vehicle_ros->gpu_sidescan_sonar_pubs_.emplace_back(sensor_publisher);
                    gpu_sidescan_sonar_cnt += 1;
                    break;
                }
                default: {
                    throw std::invalid_argument("Unexpected sensor type");
                }
                }
            }
        }

        vehicle_name_ptr_map_.emplace(curr_vehicle_name, std::move(vehicle_ros));
    }

    // add takeoff and land all services if more than 2 drones
    if (vehicle_name_ptr_map_.size() > 1 && airsim_mode_ == AIRSIM_MODE::DRONE) {
        takeoff_all_srvr_ = nh_->create_service<furosim_interfaces::srv::Takeoff>("~/all_robots/takeoff", std::bind(&AirsimROSWrapper::takeoff_all_srv_cb, this, _1, _2));
        land_all_srvr_ = nh_->create_service<furosim_interfaces::srv::Land>("~/all_robots/land", std::bind(&AirsimROSWrapper::land_all_srv_cb, this, _1, _2));

        vel_cmd_all_body_frame_sub_ = nh_->create_subscription<furosim_interfaces::msg::VelCmd>("~/all_robots/vel_cmd_body_frame", 1, std::bind(&AirsimROSWrapper::vel_cmd_all_body_frame_cb, this, _1));
        vel_cmd_all_world_frame_sub_ = nh_->create_subscription<furosim_interfaces::msg::VelCmd>("~/all_robots/vel_cmd_world_frame", 1, std::bind(&AirsimROSWrapper::vel_cmd_all_world_frame_cb, this, _1));

        vel_cmd_group_body_frame_sub_ = nh_->create_subscription<furosim_interfaces::msg::VelCmdGroup>("~/group_of_robots/vel_cmd_body_frame", 1, std::bind(&AirsimROSWrapper::vel_cmd_group_body_frame_cb, this, _1));
        vel_cmd_group_world_frame_sub_ = nh_->create_subscription<furosim_interfaces::msg::VelCmdGroup>("~/group_of_robots/vel_cmd_world_frame", 1, std::bind(&AirsimROSWrapper::vel_cmd_group_world_frame_cb, this, _1));

        takeoff_group_srvr_ = nh_->create_service<furosim_interfaces::srv::TakeoffGroup>("~/group_of_robots/takeoff", std::bind(&AirsimROSWrapper::takeoff_group_srv_cb, this, _1, _2));
        land_group_srvr_ = nh_->create_service<furosim_interfaces::srv::LandGroup>("~/group_of_robots/land", std::bind(&AirsimROSWrapper::land_group_srv_cb, this, _1, _2));
    }

    reset_srvr_ = nh_->create_service<furosim_interfaces::srv::Reset>("~/reset", std::bind(&AirsimROSWrapper::reset_srv_cb, this, _1, _2));

    if (publish_clock_) {
        clock_pub_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("~/clock", 1);
    }

    if (!airsim_img_request_vehicle_name_pair_vec_.empty()) {
        double update_airsim_img_response_every_n_sec;
        nh_->get_parameter("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec);
        auto cb = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_img_callback_groups_.push_back(cb);
        airsim_img_response_timer_ = nh_img_->create_wall_timer(std::chrono::duration<double>(update_airsim_img_response_every_n_sec), std::bind(&AirsimROSWrapper::img_response_timer_cb, this), cb);
        is_used_img_timer_cb_queue_ = true;
    }

    if (lidar_cnt > 0) {
        double update_lidar_every_n_sec;
        nh_->get_parameter("update_lidar_every_n_sec", update_lidar_every_n_sec);
        auto cb = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_lidar_callback_groups_.push_back(cb);
        airsim_lidar_update_timer_ = nh_lidar_->create_wall_timer(std::chrono::duration<double>(update_lidar_every_n_sec), std::bind(&AirsimROSWrapper::lidar_timer_cb, this), cb);
        is_used_lidar_timer_cb_queue_ = true;
    }

    if (gpu_sonar_cnt > 0) {
        double update_gpu_sonar_every_n_sec;
        nh_->get_parameter_or("update_sonar_every_n_sec", update_gpu_sonar_every_n_sec, 0.1);
        auto cb = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_gpu_sonar_callback_groups_.push_back(cb);
        airsim_gpu_sonar_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_gpu_sonar_every_n_sec), std::bind(&AirsimROSWrapper::gpu_sonar_timer_cb, this), cb);
        is_used_gpu_sonar_timer_cb_queue_ = true;
    }

    if (gpu_sidescan_sonar_cnt > 0) {
        double update_gpu_sidescan_sonar_every_n_sec;
        nh_->get_parameter_or("update_sidescan_sonar_every_n_sec", update_gpu_sidescan_sonar_every_n_sec, 0.1);
        auto cb = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_gpu_sidescan_sonar_callback_groups_.push_back(cb);
        airsim_gpu_sidescan_sonar_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_gpu_sidescan_sonar_every_n_sec), std::bind(&AirsimROSWrapper::gpu_sidescan_sonar_timer_cb, this), cb);
        is_used_gpu_sidescan_sonar_timer_cb_queue_ = true;
    }

    initialize_airsim();
}

// QoS - The depth of the publisher message queue.
template <typename T>
const SensorPublisher<T> AirsimROSWrapper::create_sensor_publisher(const std::string& sensor_type_name, const std::string& sensor_name,
                                                                   SensorBase::SensorType sensor_type, const std::string& topic_name, int QoS)
{
    RCLCPP_INFO_STREAM(nh_->get_logger(), sensor_type_name);
    SensorPublisher<T> sensor_publisher;
    sensor_publisher.sensor_name = sensor_name;
    sensor_publisher.sensor_type = sensor_type;
    sensor_publisher.publisher = nh_->create_publisher<T>("~/" + topic_name, QoS);
    return sensor_publisher;
}

bool AirsimROSWrapper::takeoff_srv_cb(std::shared_ptr<furosim_interfaces::srv::Takeoff::Request> request, std::shared_ptr<furosim_interfaces::srv::Takeoff::Response> response, const std::string& vehicle_name)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name)->waitOnLastTask();
    else
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name);

    return true;
}

bool AirsimROSWrapper::takeoff_group_srv_cb(std::shared_ptr<furosim_interfaces::srv::TakeoffGroup::Request> request, std::shared_ptr<furosim_interfaces::srv::TakeoffGroup::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        for (const auto& vehicle_name : request->vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name)->waitOnLastTask();
    else
        for (const auto& vehicle_name : request->vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name);

    return true;
}

bool AirsimROSWrapper::takeoff_all_srv_cb(std::shared_ptr<furosim_interfaces::srv::Takeoff::Request> request, std::shared_ptr<furosim_interfaces::srv::Takeoff::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name_ptr_pair.first)->waitOnLastTask();
    else
        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name_ptr_pair.first);

    return true;
}

bool AirsimROSWrapper::land_srv_cb(std::shared_ptr<furosim_interfaces::srv::Land::Request> request, std::shared_ptr<furosim_interfaces::srv::Land::Response> response, const std::string& vehicle_name)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name)->waitOnLastTask();
    else
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name);

    return true;
}

bool AirsimROSWrapper::land_group_srv_cb(std::shared_ptr<furosim_interfaces::srv::LandGroup::Request> request, std::shared_ptr<furosim_interfaces::srv::LandGroup::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        for (const auto& vehicle_name : request->vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name)->waitOnLastTask();
    else
        for (const auto& vehicle_name : request->vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name);

    return true;
}

bool AirsimROSWrapper::land_all_srv_cb(std::shared_ptr<furosim_interfaces::srv::Land::Request> request, std::shared_ptr<furosim_interfaces::srv::Land::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name_ptr_pair.first)->waitOnLastTask();
    else
        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name_ptr_pair.first);

    return true;
}

bool AirsimROSWrapper::reset_srv_cb(std::shared_ptr<furosim_interfaces::srv::Reset::Request> request, std::shared_ptr<furosim_interfaces::srv::Reset::Response> response)
{
    unused(request);
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    airsim_client_->reset();
    return true;
}

tf2::Quaternion AirsimROSWrapper::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const
{
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const
{
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, geometry_msgs_quat.y, geometry_msgs_quat.z);
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const tf2::Quaternion& tf2_quat) const
{
    return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z());
}

void AirsimROSWrapper::car_cmd_cb(const furosim_interfaces::msg::CarControls::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    auto car = static_cast<CarROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    car->car_cmd_.throttle = msg->throttle;
    car->car_cmd_.steering = msg->steering;
    car->car_cmd_.brake = msg->brake;
    car->car_cmd_.handbrake = msg->handbrake;
    car->car_cmd_.is_manual_gear = msg->manual;
    car->car_cmd_.manual_gear = msg->manual_gear;
    car->car_cmd_.gear_immediate = msg->gear_immediate;

    car->has_car_cmd_ = true;
}

void AirsimROSWrapper::auv_cmd_cb(const furosim_interfaces::msg::VelCmd::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    auto auv = static_cast<AuvROS*>(vehicle_name_ptr_map_[vehicle_name].get());

    auv->force_cmd_.fx = msg->twist.linear.x;
    auv->force_cmd_.fy = msg->twist.linear.y;
    auv->force_cmd_.fz = msg->twist.linear.z;
    auv->force_cmd_.torque_roll = msg->twist.angular.x;
    auv->force_cmd_.torque_pitch = msg->twist.angular.y;
    auv->force_cmd_.torque_yaw = msg->twist.angular.z;
    auv->has_force_cmd_ = true;
}

void AirsimROSWrapper::ocean_current_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    get_auv_client()->setOceanCurrent(msg->vector.x, msg->vector.y, msg->vector.z, vehicle_name);
}

static void auv_finish_move(msr::airlib::AuvRpcLibClient* client, bool wait, bool& success, std::string& message)
{
    if (wait) {
        client->waitOnLastTask(&success);
        message = success ? "arrived" : "timed out or cancelled";
    }
    else {
        success = true;
        message = "command accepted";
    }
}

static msr::airlib::YawMode auv_yaw_mode(bool yaw_fixed, double yaw_deg)
{
    return msr::airlib::YawMode(!yaw_fixed, static_cast<float>(yaw_deg));
}

static float auv_timeout(double timeout_sec)
{
    return timeout_sec > 0.0 ? static_cast<float>(timeout_sec) : msr::airlib::Utils::max<float>();
}

bool AirsimROSWrapper::auv_move_to_position_srv_cb(const std::shared_ptr<furosim_interfaces::srv::MoveToPosition::Request> request, const std::shared_ptr<furosim_interfaces::srv::MoveToPosition::Response> response, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    auto* client = get_auv_client();
    client->moveToPositionAsync(request->x, request->y, request->z, request->velocity, auv_timeout(request->timeout_sec),
                                auv_yaw_mode(request->yaw_fixed, request->yaw_deg), vehicle_name);
    auv_finish_move(client, request->wait_on_last_task, response->success, response->message);
    return true;
}

bool AirsimROSWrapper::auv_move_on_path_srv_cb(const std::shared_ptr<furosim_interfaces::srv::MoveOnPath::Request> request, const std::shared_ptr<furosim_interfaces::srv::MoveOnPath::Response> response, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    std::vector<msr::airlib::Vector3r> path;
    path.reserve(request->path.size());
    for (const auto& p : request->path)
        path.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    auto* client = get_auv_client();
    client->moveOnPathAsync(path, request->velocity, auv_timeout(request->timeout_sec),
                            auv_yaw_mode(request->yaw_fixed, request->yaw_deg), vehicle_name);
    auv_finish_move(client, request->wait_on_last_task, response->success, response->message);
    return true;
}

bool AirsimROSWrapper::auv_move_to_z_srv_cb(const std::shared_ptr<furosim_interfaces::srv::MoveToZ::Request> request, const std::shared_ptr<furosim_interfaces::srv::MoveToZ::Response> response, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    auto* client = get_auv_client();
    client->moveToZAsync(request->z, request->velocity, auv_timeout(request->timeout_sec),
                         auv_yaw_mode(request->yaw_fixed, request->yaw_deg), vehicle_name);
    auv_finish_move(client, request->wait_on_last_task, response->success, response->message);
    return true;
}

bool AirsimROSWrapper::auv_hover_srv_cb(const std::shared_ptr<furosim_interfaces::srv::Hover::Request> request, const std::shared_ptr<furosim_interfaces::srv::Hover::Response> response, const std::string& vehicle_name)
{
    unused(request);
    std::lock_guard<std::mutex> guard(control_mutex_);
    get_auv_client()->hoverAsync(vehicle_name)->waitOnLastTask(&response->success);
    response->message = "station keeping";
    return true;
}

msr::airlib::AuvRpcLibClient* AirsimROSWrapper::get_auv_client()
{
    return static_cast<msr::airlib::AuvRpcLibClient*>(airsim_client_.get());
}

msr::airlib::Pose AirsimROSWrapper::get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& airlib_quat) const
{
    return msr::airlib::Pose(msr::airlib::Vector3r(x, y, z), airlib_quat);
}

void AirsimROSWrapper::vel_cmd_body_frame_cb(const furosim_interfaces::msg::VelCmd::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    drone->vel_cmd_ = get_airlib_body_vel_cmd(*msg, drone->curr_drone_state_.kinematics_estimated.pose.orientation);
    drone->has_vel_cmd_ = true;
}

void AirsimROSWrapper::vel_cmd_group_body_frame_cb(const furosim_interfaces::msg::VelCmdGroup::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    for (const auto& vehicle_name : msg->vehicle_names) {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
        drone->vel_cmd_ = get_airlib_body_vel_cmd(msg->vel_cmd, drone->curr_drone_state_.kinematics_estimated.pose.orientation);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapper::vel_cmd_all_body_frame_cb(const furosim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_pair.second.get());
        drone->vel_cmd_ = get_airlib_body_vel_cmd(*msg, drone->curr_drone_state_.kinematics_estimated.pose.orientation);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapper::vel_cmd_world_frame_cb(const furosim_interfaces::msg::VelCmd::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    drone->vel_cmd_ = get_airlib_world_vel_cmd(*msg);
    drone->has_vel_cmd_ = true;
}

void AirsimROSWrapper::vel_cmd_group_world_frame_cb(const furosim_interfaces::msg::VelCmdGroup::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    for (const auto& vehicle_name : msg->vehicle_names) {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
        drone->vel_cmd_ = get_airlib_world_vel_cmd(msg->vel_cmd);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapper::vel_cmd_all_world_frame_cb(const furosim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_pair.second.get());
        drone->vel_cmd_ = get_airlib_world_vel_cmd(*msg);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapper::gimbal_angle_quat_cmd_cb(const furosim_interfaces::msg::GimbalAngleQuatCmd::SharedPtr gimbal_angle_quat_cmd_msg)
{
    tf2::Quaternion quat_control_cmd;
    try {
        tf2::convert(gimbal_angle_quat_cmd_msg->orientation, quat_control_cmd);
        quat_control_cmd.normalize();
        gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd);
        gimbal_cmd_.camera_name = gimbal_angle_quat_cmd_msg->camera_name;
        gimbal_cmd_.vehicle_name = gimbal_angle_quat_cmd_msg->vehicle_name;
        has_gimbal_cmd_ = true;
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
    }
}

void AirsimROSWrapper::gimbal_angle_euler_cmd_cb(const furosim_interfaces::msg::GimbalAngleEulerCmd::SharedPtr gimbal_angle_euler_cmd_msg)
{
    try {
        tf2::Quaternion quat_control_cmd;
        quat_control_cmd.setRPY(math_common::deg2rad(gimbal_angle_euler_cmd_msg->roll), math_common::deg2rad(gimbal_angle_euler_cmd_msg->pitch), math_common::deg2rad(gimbal_angle_euler_cmd_msg->yaw));
        quat_control_cmd.normalize();
        gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd);
        gimbal_cmd_.camera_name = gimbal_angle_euler_cmd_msg->camera_name;
        gimbal_cmd_.vehicle_name = gimbal_angle_euler_cmd_msg->vehicle_name;
        has_gimbal_cmd_ = true;
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
    }
}

furosim_interfaces::msg::CarState AirsimROSWrapper::get_roscarstate_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    furosim_interfaces::msg::CarState state_msg;
    const auto odo = get_odom_msg_from_car_state(car_state);

    state_msg.pose = odo.pose;
    state_msg.twist = odo.twist;
    state_msg.speed = car_state.speed;
    state_msg.gear = car_state.gear;
    state_msg.rpm = car_state.rpm;
    state_msg.maxrpm = car_state.maxrpm;
    state_msg.handbrake = car_state.handbrake;
    state_msg.header.stamp = rclcpp::Time(car_state.timestamp);

    return state_msg;
}

nav_msgs::msg::Odometry AirsimROSWrapper::get_odom_msg_from_kinematic_state(const msr::airlib::Kinematics::State& kinematics_estimated) const
{
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.pose.pose.position.x = kinematics_estimated.pose.position.x();
    odom_msg.pose.pose.position.y = kinematics_estimated.pose.position.y();
    odom_msg.pose.pose.position.z = kinematics_estimated.pose.position.z();
    odom_msg.pose.pose.orientation.x = kinematics_estimated.pose.orientation.x();
    odom_msg.pose.pose.orientation.y = kinematics_estimated.pose.orientation.y();
    odom_msg.pose.pose.orientation.z = kinematics_estimated.pose.orientation.z();
    odom_msg.pose.pose.orientation.w = kinematics_estimated.pose.orientation.w();

    odom_msg.twist.twist.linear.x = kinematics_estimated.twist.linear.x();
    odom_msg.twist.twist.linear.y = kinematics_estimated.twist.linear.y();
    odom_msg.twist.twist.linear.z = kinematics_estimated.twist.linear.z();
    odom_msg.twist.twist.angular.x = kinematics_estimated.twist.angular.x();
    odom_msg.twist.twist.angular.y = kinematics_estimated.twist.angular.y();
    odom_msg.twist.twist.angular.z = kinematics_estimated.twist.angular.z();

    if (isENU_) {
        std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
        odom_msg.pose.pose.position.z = -odom_msg.pose.pose.position.z;
        std::swap(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y);
        odom_msg.pose.pose.orientation.z = -odom_msg.pose.pose.orientation.z;
        std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
        odom_msg.twist.twist.linear.z = -odom_msg.twist.twist.linear.z;
        std::swap(odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y);
        odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;
    }

    return odom_msg;
}

nav_msgs::msg::Odometry AirsimROSWrapper::get_odom_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    return get_odom_msg_from_kinematic_state(car_state.kinematics_estimated);
}

nav_msgs::msg::Odometry AirsimROSWrapper::get_odom_msg_from_multirotor_state(const msr::airlib::MultirotorState& drone_state) const
{
    return get_odom_msg_from_kinematic_state(drone_state.kinematics_estimated);
}

nav_msgs::msg::Odometry AirsimROSWrapper::get_odom_msg_from_auv_state(const msr::airlib::AuvApiBase::AuvState& auv_state) const
{
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.pose.pose.position.x = auv_state.getPosition().x();
    odom_msg.pose.pose.position.y = auv_state.getPosition().y();
    odom_msg.pose.pose.position.z = auv_state.getPosition().z();
    odom_msg.pose.pose.orientation.x = auv_state.getOrientation().x();
    odom_msg.pose.pose.orientation.y = auv_state.getOrientation().y();
    odom_msg.pose.pose.orientation.z = auv_state.getOrientation().z();
    odom_msg.pose.pose.orientation.w = auv_state.getOrientation().w();

    odom_msg.twist.twist.linear.x = auv_state.kinematics_estimated.twist.linear.x();
    odom_msg.twist.twist.linear.y = auv_state.kinematics_estimated.twist.linear.y();
    odom_msg.twist.twist.linear.z = auv_state.kinematics_estimated.twist.linear.z();
    odom_msg.twist.twist.angular.x = auv_state.kinematics_estimated.twist.angular.x();
    odom_msg.twist.twist.angular.y = auv_state.kinematics_estimated.twist.angular.y();
    odom_msg.twist.twist.angular.z = auv_state.kinematics_estimated.twist.angular.z();

    return odom_msg;
}

// https://docs.ros.org/jade/api/sensor_msgs/html/point__cloud__conversion_8h_source.html#l00066
sensor_msgs::msg::PointCloud2 AirsimROSWrapper::get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    sensor_msgs::msg::PointCloud2 lidar_msg;
    lidar_msg.header.stamp = rclcpp::Time(lidar_data.time_stamp);
    lidar_msg.header.frame_id = vehicle_name + "/" + sensor_name;

    if (lidar_data.point_cloud.size() > 3) {
        lidar_msg.height = 1;
        lidar_msg.width = lidar_data.point_cloud.size() / 3;

        lidar_msg.fields.resize(3);
        lidar_msg.fields[0].name = "x";
        lidar_msg.fields[1].name = "y";
        lidar_msg.fields[2].name = "z";

        int offset = 0;

        for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4) {
            lidar_msg.fields[d].offset = offset;
            lidar_msg.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
            lidar_msg.fields[d].count = 1;
        }

        lidar_msg.is_bigendian = false;
        lidar_msg.point_step = offset;
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

        lidar_msg.is_dense = true;
        std::vector<float> data_std = lidar_data.point_cloud;

        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data_std.data());
        std::vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
        lidar_msg.data = std::move(lidar_msg_data);

        if (isENU_) {
            try {
                sensor_msgs::msg::PointCloud2 lidar_msg_enu;
                auto transformStampedENU = tf_buffer_->lookupTransform(AIRSIM_FRAME_ID, vehicle_name, rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(1));
                tf2::doTransform(lidar_msg, lidar_msg_enu, transformStampedENU);

                lidar_msg_enu.header.stamp = lidar_msg.header.stamp;
                lidar_msg_enu.header.frame_id = lidar_msg.header.frame_id;

                lidar_msg = std::move(lidar_msg_enu);
            }
            catch (tf2::TransformException& ex) {
                RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
                rclcpp::Rate(1.0).sleep();
            }
        }
    }

    return lidar_msg;
}

std::shared_ptr<sensor_msgs::msg::Image> AirsimROSWrapper::get_gpu_sonar_img_msg_from_airsim(const msr::airlib::GpuSonarData& sonar_data, const std::string& vehicle_name, const std::string& sensor_name, int image_height, int image_width) const
{
    std::shared_ptr<sensor_msgs::msg::Image> sonar_img_msg;

    int expected_size = image_height * image_width;
    if (static_cast<int>(sonar_data.sonar_raw_data.size()) >= expected_size && expected_size > 0) {

        cv::Mat sonar_image(image_height, image_width, CV_8UC1, cv::Scalar(0));

        for (int i = 0; i < expected_size; i++) {
            int row = i / image_width;
            int col = i % image_width;
            float intensity = sonar_data.sonar_raw_data[i];
            sonar_image.at<uchar>(row, col) = static_cast<uchar>(std::min(intensity * 255.0f, 255.0f));
        }

        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = nh_->now();
        cv_image.header.frame_id = vehicle_name + "/" + sensor_name;
        cv_image.encoding = "mono8";
        cv_image.image = sonar_image;

        sonar_img_msg = cv_image.toImageMsg();
    }

    return sonar_img_msg;
}

std::shared_ptr<sensor_msgs::msg::Image> AirsimROSWrapper::get_gpu_sidescan_sonar_img_msg_from_airsim(const msr::airlib::GpuSidescanSonarData& sonar_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    std::shared_ptr<sensor_msgs::msg::Image> sonar_img_msg;

    int port_size = sonar_data.port_raw_data.size();
    int starboard_size = sonar_data.starboard_raw_data.size();

    if (port_size > 0 && starboard_size > 0) {
        int width = port_size + starboard_size;
        cv::Mat scanline(1, width, CV_8UC1, cv::Scalar(0));

        // port side (reversed, left half)
        for (int i = 0; i < port_size; i++) {
            float intensity = sonar_data.port_raw_data[i];
            scanline.at<uchar>(0, port_size - 1 - i) = static_cast<uchar>(std::min(intensity * 255.0f, 255.0f));
        }

        // starboard side (right half)
        for (int i = 0; i < starboard_size; i++) {
            float intensity = sonar_data.starboard_raw_data[i];
            scanline.at<uchar>(0, port_size + i) = static_cast<uchar>(std::min(intensity * 255.0f, 255.0f));
        }

        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = nh_->now();
        cv_image.header.frame_id = vehicle_name + "/" + sensor_name;
        cv_image.encoding = "mono8";
        cv_image.image = scanline;

        sonar_img_msg = cv_image.toImageMsg();
    }

    return sonar_img_msg;
}

furosim_interfaces::msg::Environment AirsimROSWrapper::get_environment_msg_from_airsim(const msr::airlib::Environment::State& env_data) const
{
    furosim_interfaces::msg::Environment env_msg;
    env_msg.position.x = env_data.position.x();
    env_msg.position.y = env_data.position.y();
    env_msg.position.z = env_data.position.z();
    env_msg.geo_point.latitude = env_data.geo_point.latitude;
    env_msg.geo_point.longitude = env_data.geo_point.longitude;
    env_msg.geo_point.altitude = env_data.geo_point.altitude;
    env_msg.gravity.x = env_data.gravity.x();
    env_msg.gravity.y = env_data.gravity.y();
    env_msg.gravity.z = env_data.gravity.z();
    env_msg.air_pressure = env_data.air_pressure;
    env_msg.temperature = env_data.temperature;
    env_msg.air_density = env_data.temperature;

    return env_msg;
}

sensor_msgs::msg::MagneticField AirsimROSWrapper::get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data) const
{
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.magnetic_field.x = mag_data.magnetic_field_body.x();
    mag_msg.magnetic_field.y = mag_data.magnetic_field_body.y();
    mag_msg.magnetic_field.z = mag_data.magnetic_field_body.z();
    std::copy(std::begin(mag_data.magnetic_field_covariance),
              std::end(mag_data.magnetic_field_covariance),
              std::begin(mag_msg.magnetic_field_covariance));
    mag_msg.header.stamp = rclcpp::Time(mag_data.time_stamp);

    return mag_msg;
}

sensor_msgs::msg::NavSatFix AirsimROSWrapper::get_gps_msg_from_airsim(const msr::airlib::GpsBase::Output& gps_data) const
{
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = rclcpp::Time(gps_data.time_stamp);
    gps_msg.latitude = gps_data.gnss.geo_point.latitude;
    gps_msg.longitude = gps_data.gnss.geo_point.longitude;
    gps_msg.altitude = gps_data.gnss.geo_point.altitude;
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
    gps_msg.status.status = gps_data.gnss.fix_type;

    return gps_msg;
}

sensor_msgs::msg::Range AirsimROSWrapper::get_range_from_airsim(const msr::airlib::DistanceSensorData& dist_data) const
{
    sensor_msgs::msg::Range dist_msg;
    dist_msg.header.stamp = rclcpp::Time(dist_data.time_stamp);
    dist_msg.range = dist_data.distance;
    dist_msg.min_range = dist_data.min_distance;
    dist_msg.max_range = dist_data.max_distance;

    return dist_msg;
}

furosim_interfaces::msg::Altimeter AirsimROSWrapper::get_altimeter_msg_from_airsim(const msr::airlib::BarometerBase::Output& alt_data) const
{
    furosim_interfaces::msg::Altimeter alt_msg;
    alt_msg.header.stamp = rclcpp::Time(alt_data.time_stamp);
    alt_msg.altitude = alt_data.altitude;
    alt_msg.pressure = alt_data.pressure;
    alt_msg.qnh = alt_data.qnh;

    return alt_msg;
}

furosim_interfaces::msg::Pressure AirsimROSWrapper::get_pressure_msg_from_airsim(const msr::airlib::PressureBase::Output& prs_data) const
{
    furosim_interfaces::msg::Pressure prs_msg;
    prs_msg.header.stamp = rclcpp::Time(prs_data.time_stamp);
    prs_msg.depth = prs_data.depth;
    prs_msg.pressure = prs_data.pressure;

    return prs_msg;
}

furosim_interfaces::msg::DVL AirsimROSWrapper::get_dvl_msg_from_airsim(const msr::airlib::DvlData& dvl_data) const
{
    furosim_interfaces::msg::DVL dvl_msg;
    dvl_msg.header.stamp = rclcpp::Time(dvl_data.time_stamp);
    dvl_msg.velocity.x = dvl_data.velocity.x();
    dvl_msg.velocity.y = dvl_data.velocity.y();
    dvl_msg.velocity.z = dvl_data.velocity.z();
    dvl_msg.estimated_pose.position.x = dvl_data.estimated_pose.position.x();
    dvl_msg.estimated_pose.position.y = dvl_data.estimated_pose.position.y();
    dvl_msg.estimated_pose.position.z = dvl_data.estimated_pose.position.z();
    dvl_msg.estimated_pose.orientation.x = dvl_data.estimated_pose.orientation.x();
    dvl_msg.estimated_pose.orientation.y = dvl_data.estimated_pose.orientation.y();
    dvl_msg.estimated_pose.orientation.z = dvl_data.estimated_pose.orientation.z();
    dvl_msg.estimated_pose.orientation.w = dvl_data.estimated_pose.orientation.w();
    dvl_msg.altitude = dvl_data.altitude;

    return dvl_msg;
}

sensor_msgs::msg::Imu AirsimROSWrapper::get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = rclcpp::Time(imu_data.time_stamp);
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();

    imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z();

    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();

    return imu_msg;
}

void AirsimROSWrapper::publish_odom_tf(const nav_msgs::msg::Odometry& odom_msg)
{
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = odom_msg.header;
    odom_tf.child_frame_id = odom_msg.child_frame_id;
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);
}

furosim_interfaces::msg::GPSYaw AirsimROSWrapper::get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    furosim_interfaces::msg::GPSYaw gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude;
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

sensor_msgs::msg::NavSatFix AirsimROSWrapper::get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude;
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

msr::airlib::GeoPoint AirsimROSWrapper::get_origin_geo_point() const
{
    msr::airlib::HomeGeoPoint geo_point = AirSimSettings::singleton().origin_geopoint;
    return geo_point.home_geo_point;
}

VelCmd AirsimROSWrapper::get_airlib_world_vel_cmd(const furosim_interfaces::msg::VelCmd& msg) const
{
    VelCmd vel_cmd;
    vel_cmd.x = msg.twist.linear.x;
    vel_cmd.y = msg.twist.linear.y;
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd.yaw_mode.is_rate = true;
    vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);
    return vel_cmd;
}

VelCmd AirsimROSWrapper::get_airlib_body_vel_cmd(const furosim_interfaces::msg::VelCmd& msg, const msr::airlib::Quaternionr& airlib_quat) const
{
    VelCmd vel_cmd;
    double roll, pitch, yaw;
    tf2::Matrix3x3(get_tf2_quat(airlib_quat)).getRPY(roll, pitch, yaw);

    vel_cmd.x = (msg.twist.linear.x * cos(yaw)) - (msg.twist.linear.y * sin(yaw));
    vel_cmd.y = (msg.twist.linear.x * sin(yaw)) + (msg.twist.linear.y * cos(yaw));
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd.yaw_mode.is_rate = true;
    vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);

    return vel_cmd;
}

geometry_msgs::msg::Transform AirsimROSWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    tf2::Quaternion quat;
    quat.setRPY(rotation.roll, rotation.pitch, rotation.yaw);
    transform.rotation.x = quat.x();
    transform.rotation.y = quat.y();
    transform.rotation.z = quat.z();
    transform.rotation.w = quat.w();

    return transform;
}

geometry_msgs::msg::Transform AirsimROSWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    transform.rotation.x = quaternion.x();
    transform.rotation.y = quaternion.y();
    transform.rotation.z = quaternion.z();
    transform.rotation.w = quaternion.w();

    return transform;
}

void AirsimROSWrapper::drone_state_timer_cb()
{
    try {
        origin_geo_point_pub_->publish(origin_geo_point_msg_);

        const auto now = update_state();

        if (!airsim_client_->simIsPaused()) {
            ros_clock_.clock = now;
        }
        if (publish_clock_) {
            clock_pub_->publish(ros_clock_);
        }

        publish_vehicle_state();

        update_commands();
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
    }
}

void AirsimROSWrapper::update_and_publish_static_transforms(VehicleROS* vehicle_ros)
{
    if (vehicle_ros && !vehicle_ros->static_tf_msg_vec_.empty()) {
        for (auto& static_tf_msg : vehicle_ros->static_tf_msg_vec_) {
            static_tf_msg.header.stamp = vehicle_ros->stamp_;
            static_tf_pub_->sendTransform(static_tf_msg);
        }
    }
}

rclcpp::Time AirsimROSWrapper::update_state()
{
    bool got_sim_time = false;
    rclcpp::Time curr_ros_time = nh_->now();

    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        rclcpp::Time vehicle_time;
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        auto env_data = airsim_client_->simGetGroundTruthEnvironment(vehicle_ros->vehicle_name_);

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
            auto rpc = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
            drone->curr_drone_state_ = rpc->getMultirotorState(vehicle_ros->vehicle_name_);

            vehicle_time = rclcpp::Time(drone->curr_drone_state_.timestamp);
            if (!got_sim_time) {
                curr_ros_time = vehicle_time;
                got_sim_time = true;
            }

            vehicle_ros->gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(drone->curr_drone_state_.gps_location);
            vehicle_ros->gps_sensor_msg_.header.stamp = vehicle_time;

            vehicle_ros->curr_odom_ = get_odom_msg_from_multirotor_state(drone->curr_drone_state_);
        }
        else if (airsim_mode_ == AIRSIM_MODE::AUV) {
            auto auv = static_cast<AuvROS*>(vehicle_ros.get());
            auv->curr_auv_state_ = get_auv_client()->getAuvState(vehicle_ros->vehicle_name_);

            vehicle_time = rclcpp::Time(auv->curr_auv_state_.timestamp);
            if (!got_sim_time) {
                curr_ros_time = vehicle_time;
                got_sim_time = true;
            }

            vehicle_ros->gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(env_data.geo_point);
            vehicle_ros->gps_sensor_msg_.header.stamp = vehicle_time;

            vehicle_ros->curr_odom_ = get_odom_msg_from_auv_state(auv->curr_auv_state_);
        }
        else {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            auto rpc = static_cast<msr::airlib::CarRpcLibClient*>(airsim_client_.get());
            car->curr_car_state_ = rpc->getCarState(vehicle_ros->vehicle_name_);

            vehicle_time = rclcpp::Time(car->curr_car_state_.timestamp);
            if (!got_sim_time) {
                curr_ros_time = vehicle_time;
                got_sim_time = true;
            }

            vehicle_ros->gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(env_data.geo_point);
            vehicle_ros->gps_sensor_msg_.header.stamp = vehicle_time;

            vehicle_ros->curr_odom_ = get_odom_msg_from_car_state(car->curr_car_state_);

            furosim_interfaces::msg::CarState state_msg = get_roscarstate_msg_from_car_state(car->curr_car_state_);
            state_msg.header.frame_id = vehicle_ros->vehicle_name_;
            car->car_state_msg_ = state_msg;
        }

        vehicle_ros->stamp_ = vehicle_time;

        furosim_interfaces::msg::Environment env_msg = get_environment_msg_from_airsim(env_data);
        env_msg.header.frame_id = vehicle_ros->vehicle_name_;
        env_msg.header.stamp = vehicle_time;
        vehicle_ros->env_msg_ = env_msg;

        vehicle_ros->curr_odom_.header.frame_id = vehicle_ros->vehicle_name_;
        vehicle_ros->curr_odom_.child_frame_id = vehicle_ros->odom_frame_id_;
        vehicle_ros->curr_odom_.header.stamp = vehicle_time;
    }

    return curr_ros_time;
}

void AirsimROSWrapper::publish_vehicle_state()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        vehicle_ros->env_pub_->publish(vehicle_ros->env_msg_);

        if (airsim_mode_ == AIRSIM_MODE::CAR) {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            car->car_state_pub_->publish(car->car_state_msg_);
        }

        vehicle_ros->odom_local_pub_->publish(vehicle_ros->curr_odom_);
        publish_odom_tf(vehicle_ros->curr_odom_);

        vehicle_ros->global_gps_pub_->publish(vehicle_ros->gps_sensor_msg_);

        for (auto& sensor_publisher : vehicle_ros->barometer_pubs_) {
            auto baro_data = airsim_client_->getBarometerData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            furosim_interfaces::msg::Altimeter alt_msg = get_altimeter_msg_from_airsim(baro_data);
            alt_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(alt_msg);
        }

        for (auto& sensor_publisher : vehicle_ros->imu_pubs_) {
            auto imu_data = airsim_client_->getImuData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            sensor_msgs::msg::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
            imu_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(imu_msg);
        }

        for (auto& sensor_publisher : vehicle_ros->distance_pubs_) {
            auto distance_data = airsim_client_->getDistanceSensorData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            sensor_msgs::msg::Range dist_msg = get_range_from_airsim(distance_data);
            dist_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(dist_msg);
        }

        for (auto& sensor_publisher : vehicle_ros->gps_pubs_) {
            auto gps_data = airsim_client_->getGpsData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            sensor_msgs::msg::NavSatFix gps_msg = get_gps_msg_from_airsim(gps_data);
            gps_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(gps_msg);
        }

        for (auto& sensor_publisher : vehicle_ros->magnetometer_pubs_) {
            auto mag_data = airsim_client_->getMagnetometerData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            sensor_msgs::msg::MagneticField mag_msg = get_mag_msg_from_airsim(mag_data);
            mag_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(mag_msg);
        }

        for (auto& sensor_publisher : vehicle_ros->pressure_pubs_) {
            auto prs_data = airsim_client_->getPressureData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            furosim_interfaces::msg::Pressure prs_msg = get_pressure_msg_from_airsim(prs_data);
            prs_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(prs_msg);
        }

        for (auto& sensor_publisher : vehicle_ros->dvl_pubs_) {
            auto dvl_data = airsim_client_->getDvlData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            furosim_interfaces::msg::DVL dvl_msg = get_dvl_msg_from_airsim(dvl_data);
            dvl_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(dvl_msg);
        }

        update_and_publish_static_transforms(vehicle_ros.get());
    }
}

void AirsimROSWrapper::update_commands()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());

            if (drone->has_vel_cmd_) {
                std::lock_guard<std::mutex> guard(control_mutex_);
                static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByVelocityAsync(drone->vel_cmd_.x, drone->vel_cmd_.y, drone->vel_cmd_.z, vel_cmd_duration_, msr::airlib::DrivetrainType::MaxDegreeOfFreedom, drone->vel_cmd_.yaw_mode, drone->vehicle_name_);
            }
            drone->has_vel_cmd_ = false;
        }
        else if (airsim_mode_ == AIRSIM_MODE::AUV) {
            auto auv = static_cast<AuvROS*>(vehicle_ros.get());

            if (auv->has_force_cmd_) {
                std::lock_guard<std::mutex> guard(control_mutex_);
                get_auv_client()->setAuvControls(auv->force_cmd_.fx,
                                                 auv->force_cmd_.fy,
                                                 auv->force_cmd_.fz,
                                                 auv->force_cmd_.torque_roll,
                                                 auv->force_cmd_.torque_pitch,
                                                 auv->force_cmd_.torque_yaw,
                                                 vehicle_ros->vehicle_name_);
                auv->force_cmd_streaming_ = true;
            }
            else if (auv->force_cmd_streaming_) {
                // Zero the wrench once when the force_cmd stream stops. Sending it every cycle would also
                // cancel any waypoint/hover task started through the services (setAuvControls overrides the autopilot).
                std::lock_guard<std::mutex> guard(control_mutex_);
                get_auv_client()->setAuvControls(0, 0, 0, 0, 0, 0, vehicle_ros->vehicle_name_);
                auv->force_cmd_streaming_ = false;
            }
            auv->has_force_cmd_ = false;
        }
        else {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            if (car->has_car_cmd_) {
                std::lock_guard<std::mutex> guard(control_mutex_);
                static_cast<msr::airlib::CarRpcLibClient*>(airsim_client_.get())->setCarControls(car->car_cmd_, vehicle_ros->vehicle_name_);
            }
            car->has_car_cmd_ = false;
        }
    }

    if (has_gimbal_cmd_) {
        std::lock_guard<std::mutex> guard(control_mutex_);
        airsim_client_->simSetCameraPose(gimbal_cmd_.camera_name, get_airlib_pose(0, 0, 0, gimbal_cmd_.target_quat), gimbal_cmd_.vehicle_name);
    }

    has_gimbal_cmd_ = false;
}

void AirsimROSWrapper::gpu_sonar_timer_cb()
{
    try {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            if (!vehicle_name_ptr_pair.second->gpu_sonar_pubs_.empty()) {
                for (auto& sonar_publisher : vehicle_name_ptr_pair.second->gpu_sonar_pubs_) {
                    auto sonar_data = airsim_client_gpu_sonar_.getGpuSonarData(sonar_publisher.sensor_name, vehicle_name_ptr_pair.first);
                    auto sonar_msg = get_gpu_sonar_img_msg_from_airsim(sonar_data, vehicle_name_ptr_pair.first, sonar_publisher.sensor_name, sonar_publisher.image_height, sonar_publisher.image_width);
                    if (sonar_msg) {
                        sonar_publisher.publisher->publish(*sonar_msg);
                    }
                }
            }
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, didn't get FLS sonar response.\n%s", msg.c_str());
    }
}

void AirsimROSWrapper::gpu_sidescan_sonar_timer_cb()
{
    try {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            if (!vehicle_name_ptr_pair.second->gpu_sidescan_sonar_pubs_.empty()) {
                for (auto& sonar_publisher : vehicle_name_ptr_pair.second->gpu_sidescan_sonar_pubs_) {
                    auto sonar_data = airsim_client_gpu_sidescan_sonar_.getGpuSidescanSonarData(sonar_publisher.sensor_name, vehicle_name_ptr_pair.first);
                    auto sonar_msg = get_gpu_sidescan_sonar_img_msg_from_airsim(sonar_data, vehicle_name_ptr_pair.first, sonar_publisher.sensor_name);
                    if (sonar_msg) {
                        sonar_publisher.publisher->publish(*sonar_msg);
                    }
                }
            }
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, didn't get SSS sonar response.\n%s", msg.c_str());
    }
}

// airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS
void AirsimROSWrapper::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
{
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

void AirsimROSWrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
{
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


void AirsimROSWrapper::convert_tf_msg_to_enu(geometry_msgs::msg::TransformStamped& tf_msg)
{
    std::swap(tf_msg.transform.translation.x, tf_msg.transform.translation.y);
    std::swap(tf_msg.transform.rotation.x, tf_msg.transform.rotation.y);
    tf_msg.transform.translation.z = -tf_msg.transform.translation.z;
    tf_msg.transform.rotation.z = -tf_msg.transform.rotation.z;
}

geometry_msgs::msg::Transform AirsimROSWrapper::get_camera_optical_tf_from_body_tf(const geometry_msgs::msg::Transform& body_tf) const
{
    geometry_msgs::msg::Transform optical_tf = body_tf;
    auto opticalQ = msr::airlib::Quaternionr(optical_tf.rotation.w, optical_tf.rotation.x, optical_tf.rotation.y, optical_tf.rotation.z);
    if (isENU_)
        opticalQ *= msr::airlib::Quaternionr(0.7071068, -0.7071068, 0, 0);
    else
        opticalQ *= msr::airlib::Quaternionr(0.5, 0.5, 0.5, 0.5);
    optical_tf.rotation.w = opticalQ.w();
    optical_tf.rotation.x = opticalQ.x();
    optical_tf.rotation.y = opticalQ.y();
    optical_tf.rotation.z = opticalQ.z();
    return optical_tf;
}

void AirsimROSWrapper::append_static_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
{
    geometry_msgs::msg::TransformStamped vehicle_tf_msg;
    vehicle_tf_msg.header.frame_id = world_frame_id_;
    vehicle_tf_msg.header.stamp = nh_->now();
    vehicle_tf_msg.child_frame_id = vehicle_ros->vehicle_name_;
    vehicle_tf_msg.transform = get_transform_msg_from_airsim(vehicle_setting.position, vehicle_setting.rotation);

    if (isENU_) {
        convert_tf_msg_to_enu(vehicle_tf_msg);
    }

    vehicle_ros->static_tf_msg_vec_.emplace_back(vehicle_tf_msg);
}

void AirsimROSWrapper::append_static_lidar_tf(VehicleROS* vehicle_ros, const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_setting)
{
    geometry_msgs::msg::TransformStamped lidar_tf_msg;
    lidar_tf_msg.header.frame_id = vehicle_ros->vehicle_name_ + "/" + odom_frame_id_;
    lidar_tf_msg.child_frame_id = vehicle_ros->vehicle_name_ + "/" + lidar_name;
    lidar_tf_msg.transform = get_transform_msg_from_airsim(lidar_setting.relative_pose.position, lidar_setting.relative_pose.orientation);

    if (isENU_) {
        convert_tf_msg_to_enu(lidar_tf_msg);
    }

    vehicle_ros->static_tf_msg_vec_.emplace_back(lidar_tf_msg);
}

void AirsimROSWrapper::append_static_gpu_sonar_tf(VehicleROS* vehicle_ros, const std::string& sonar_name, const msr::airlib::GpuSonarSimpleParams& sonar_setting)
{
    geometry_msgs::msg::TransformStamped sonar_tf_msg;
    sonar_tf_msg.header.frame_id = vehicle_ros->vehicle_name_ + "/" + odom_frame_id_;
    sonar_tf_msg.child_frame_id = vehicle_ros->vehicle_name_ + "/" + sonar_name;
    sonar_tf_msg.transform = get_transform_msg_from_airsim(sonar_setting.relative_pose.position, sonar_setting.relative_pose.orientation);

    if (isENU_) {
        convert_tf_msg_to_enu(sonar_tf_msg);
    }

    vehicle_ros->static_tf_msg_vec_.emplace_back(sonar_tf_msg);
}

void AirsimROSWrapper::append_static_gpu_sidescan_sonar_tf(VehicleROS* vehicle_ros, const std::string& sonar_name, const msr::airlib::GpuSidescanSonarSimpleParams& sonar_setting)
{
    geometry_msgs::msg::TransformStamped sonar_tf_msg;
    sonar_tf_msg.header.frame_id = vehicle_ros->vehicle_name_ + "/" + odom_frame_id_;
    sonar_tf_msg.child_frame_id = vehicle_ros->vehicle_name_ + "/" + sonar_name;
    sonar_tf_msg.transform = get_transform_msg_from_airsim(sonar_setting.relative_pose.position, sonar_setting.relative_pose.orientation);

    if (isENU_) {
        convert_tf_msg_to_enu(sonar_tf_msg);
    }

    vehicle_ros->static_tf_msg_vec_.emplace_back(sonar_tf_msg);
}

void AirsimROSWrapper::append_static_camera_tf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting)
{
    geometry_msgs::msg::TransformStamped static_cam_tf_body_msg;
    static_cam_tf_body_msg.header.frame_id = vehicle_ros->vehicle_name_ + "/" + odom_frame_id_;
    static_cam_tf_body_msg.child_frame_id = vehicle_ros->vehicle_name_ + "/" + camera_name + "_body/static";
    static_cam_tf_body_msg.transform = get_transform_msg_from_airsim(camera_setting.position, camera_setting.rotation);

    if (isENU_) {
        convert_tf_msg_to_enu(static_cam_tf_body_msg);
    }

    geometry_msgs::msg::TransformStamped static_cam_tf_optical_msg = static_cam_tf_body_msg;
    static_cam_tf_optical_msg.child_frame_id = vehicle_ros->vehicle_name_ + "/" + camera_name + "_optical/static";
    static_cam_tf_optical_msg.child_frame_id = camera_name + "_optical/static";
    static_cam_tf_optical_msg.transform = get_camera_optical_tf_from_body_tf(static_cam_tf_body_msg.transform);

    vehicle_ros->static_tf_msg_vec_.emplace_back(static_cam_tf_body_msg);
    vehicle_ros->static_tf_msg_vec_.emplace_back(static_cam_tf_optical_msg);
}

void AirsimROSWrapper::img_response_timer_cb()
{
    try {
        int image_response_idx = 0;
        for (const auto& airsim_img_request_vehicle_name_pair : airsim_img_request_vehicle_name_pair_vec_) {
            const std::vector<ImageResponse>& img_response = airsim_client_images_.simGetImages(airsim_img_request_vehicle_name_pair.first, airsim_img_request_vehicle_name_pair.second);

            if (img_response.size() == airsim_img_request_vehicle_name_pair.first.size()) {
                process_and_publish_img_response(img_response, image_response_idx, airsim_img_request_vehicle_name_pair.second);
                image_response_idx += img_response.size();
            }
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, didn't get image response.\n%s", msg.c_str());
    }
}

void AirsimROSWrapper::lidar_timer_cb()
{
    try {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            if (!vehicle_name_ptr_pair.second->lidar_pubs_.empty()) {
                for (auto& lidar_publisher : vehicle_name_ptr_pair.second->lidar_pubs_) {
                    auto lidar_data = airsim_client_lidar_.getLidarData(lidar_publisher.sensor_name, vehicle_name_ptr_pair.first);
                    sensor_msgs::msg::PointCloud2 lidar_msg = get_lidar_msg_from_airsim(lidar_data, vehicle_name_ptr_pair.first, lidar_publisher.sensor_name);
                    lidar_publisher.publisher->publish(lidar_msg);
                }
            }
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, didn't get lidar response.\n%s", msg.c_str());
    }
}

std::shared_ptr<sensor_msgs::msg::Image> AirsimROSWrapper::get_img_msg_from_response(const ImageResponse& img_response,
                                                                                     const rclcpp::Time curr_ros_time,
                                                                                     const std::string frame_id)
{
    unused(curr_ros_time);
    std::shared_ptr<sensor_msgs::msg::Image> img_msg_ptr = std::make_shared<sensor_msgs::msg::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.image_data_uint8.size() / img_response.height;
    img_msg_ptr->header.stamp = rclcpp::Time(img_response.time_stamp);
    img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    if (is_vulkan_)
        img_msg_ptr->encoding = "rgb8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

std::shared_ptr<sensor_msgs::msg::Image> AirsimROSWrapper::get_depth_img_msg_from_response(const ImageResponse& img_response,
                                                                                           const rclcpp::Time curr_ros_time,
                                                                                           const std::string frame_id)
{
    unused(curr_ros_time);
    auto depth_img_msg = std::make_shared<sensor_msgs::msg::Image>();
    depth_img_msg->width = img_response.width;
    depth_img_msg->height = img_response.height;
    depth_img_msg->data.resize(img_response.image_data_float.size() * sizeof(float));
    memcpy(depth_img_msg->data.data(), img_response.image_data_float.data(), depth_img_msg->data.size());
    depth_img_msg->encoding = "32FC1";
    depth_img_msg->step = depth_img_msg->data.size() / img_response.height;
    depth_img_msg->is_bigendian = 0;
    depth_img_msg->header.stamp = rclcpp::Time(img_response.time_stamp);
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}

sensor_msgs::msg::CameraInfo AirsimROSWrapper::generate_cam_info(const std::string& camera_name,
                                                                 const CameraSetting& camera_setting,
                                                                 const CaptureSetting& capture_setting) const
{
    unused(camera_setting);
    sensor_msgs::msg::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "_optical";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    float f_x = (capture_setting.width / 2.0) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    cam_info_msg.k = { f_x, 0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 1.0 };
    cam_info_msg.p = { f_x, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    return cam_info_msg;
}

void AirsimROSWrapper::process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name)
{
    rclcpp::Time curr_ros_time = nh_->now();
    int img_response_idx_internal = img_response_idx;

    for (const auto& curr_img_response : img_response_vec) {
        publish_camera_tf(curr_img_response, curr_ros_time, vehicle_name, curr_img_response.camera_name);

        camera_info_msg_vec_[img_response_idx_internal].header.stamp = rclcpp::Time(curr_img_response.time_stamp);
        cam_info_pub_vec_[img_response_idx_internal]->publish(camera_info_msg_vec_[img_response_idx_internal]);

        if (curr_img_response.pixels_as_float) {
            image_pub_vec_[img_response_idx_internal].publish(get_depth_img_msg_from_response(curr_img_response,
                                                                                              curr_ros_time,
                                                                                              curr_img_response.camera_name + "_optical"));
        }
        else {
            image_pub_vec_[img_response_idx_internal].publish(get_img_msg_from_response(curr_img_response,
                                                                                        curr_ros_time,
                                                                                        curr_img_response.camera_name + "_optical"));
        }
        img_response_idx_internal++;
    }
}

void AirsimROSWrapper::publish_camera_tf(const ImageResponse& img_response, const rclcpp::Time& ros_time, const std::string& frame_id, const std::string& child_frame_id)
{
    unused(ros_time);
    geometry_msgs::msg::TransformStamped cam_tf_body_msg;
    cam_tf_body_msg.header.stamp = rclcpp::Time(img_response.time_stamp);
    cam_tf_body_msg.header.frame_id = frame_id;
    cam_tf_body_msg.child_frame_id = frame_id + "/" + child_frame_id + "_body";
    cam_tf_body_msg.transform = get_transform_msg_from_airsim(img_response.camera_position, img_response.camera_orientation);

    if (isENU_) {
        convert_tf_msg_to_enu(cam_tf_body_msg);
    }

    geometry_msgs::msg::TransformStamped cam_tf_optical_msg;
    cam_tf_optical_msg.header.stamp = rclcpp::Time(img_response.time_stamp);
    cam_tf_optical_msg.header.frame_id = frame_id;
    cam_tf_optical_msg.child_frame_id = frame_id + "/" + child_frame_id + "_optical";
    cam_tf_optical_msg.transform = get_camera_optical_tf_from_body_tf(cam_tf_body_msg.transform);

    tf_broadcaster_->sendTransform(cam_tf_body_msg);
    tf_broadcaster_->sendTransform(cam_tf_optical_msg);
}

void AirsimROSWrapper::convert_yaml_to_simple_mat(const YAML::Node& node, SimpleMatrix& m) const
{
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows * cols; ++i) {
        m.data[i] = data[i].as<double>();
    }
}

void AirsimROSWrapper::read_params_from_yaml_and_fill_cam_info_msg(const std::string& file_name, sensor_msgs::msg::CameraInfo& cam_info) const
{
    std::ifstream fin(file_name.c_str());
    YAML::Node doc = YAML::Load(fin);

    cam_info.width = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

    SimpleMatrix K_(3, 3, &cam_info.k[0]);
    convert_yaml_to_simple_mat(doc[K_YML_NAME], K_);
    SimpleMatrix R_(3, 3, &cam_info.r[0]);
    convert_yaml_to_simple_mat(doc[R_YML_NAME], R_);
    SimpleMatrix P_(3, 4, &cam_info.p[0]);
    convert_yaml_to_simple_mat(doc[P_YML_NAME], P_);

    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_rows = D_node["rows"].as<int>();
    D_cols = D_node["cols"].as<int>();
    const YAML::Node& D_data = D_node["data"];
    cam_info.d.resize(D_rows * D_cols);
    for (int i = 0; i < D_rows * D_cols; ++i) {
        cam_info.d[i] = D_data[i].as<float>();
    }
}
