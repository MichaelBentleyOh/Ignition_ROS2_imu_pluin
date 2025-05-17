// Source file template to make custom plugin for gazebo ignition

// Include Ignition Gazebo headers
// You could erase already includeed files in .hpp file.

// C++
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>

// Ignition System & Model entity headers
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>

// Igntion Components headers
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/AngularAcceleration.hh>

// Ignition Math headers
#include <ignition/math/Vector3.hh>
#include <ignition/math/RollingMean.hh>
#include <ignition/math/Helpers.hh>

// Ignition Plugin headers
#include <ignition/plugin/Register.hh>

// Include ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// package headers
#include "imu_plugin/Kinematics.hpp"

// Macro Pi constant
#define PI 3.1415926535897931

/// @brief Plugin Template for AstroROS in source file
// Notice namepace and class have to be same as header file.

namespace imu_plugin // Name your plugin namespace
{

class ImuPlugin // Name your plugin class
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate // these inheritances are fixed.
{
public:

  /// \brief Constructor
  ImuPlugin()
  {
    // We assume ROS has already been initialized externally, but if not:
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);  // or handle arguments properly
    }
    // Set up moving average filter for smoothing data for world frame data
    this->lin_vel_x_.SetWindowSize(10); this->lin_acc_x_.SetWindowSize(10);
    this->lin_vel_y_.SetWindowSize(10); this->lin_acc_y_.SetWindowSize(10);
    this->lin_vel_z_.SetWindowSize(10); this->lin_acc_z_.SetWindowSize(10);
    this->ang_vel_x_.SetWindowSize(10); this->ang_acc_x_.SetWindowSize(10);
    this->ang_vel_y_.SetWindowSize(10); this->ang_acc_y_.SetWindowSize(10);
    this->ang_vel_z_.SetWindowSize(10); this->ang_acc_z_.SetWindowSize(10);
    
    // Set up moving average filter for smoothing data for body frame data
    this->body_lin_vel_x_.SetWindowSize(10); this->body_lin_acc_x_.SetWindowSize(10);
    this->body_lin_vel_y_.SetWindowSize(10); this->body_lin_acc_y_.SetWindowSize(10);
    this->body_lin_vel_z_.SetWindowSize(10); this->body_lin_acc_z_.SetWindowSize(10);
    this->body_ang_vel_x_.SetWindowSize(10); this->body_ang_acc_x_.SetWindowSize(10);
    this->body_ang_vel_y_.SetWindowSize(10); this->body_ang_acc_y_.SetWindowSize(10);
    this->body_ang_vel_z_.SetWindowSize(10); this->body_ang_acc_z_.SetWindowSize(10);
  };

  /// \brief Destructor
  ~ImuPlugin() override
  {
    if (this->ros_spin_thread_.joinable())
    {
      this->ros_spin_thread_.join();
    }
    rclcpp::shutdown();
  }

  /// \brief Configure: Called once at startup
  /*
  _entity : objects in ignition.(world,gui,robot etc.) You will add this plugin into xacro file.
            So you could think this entity is robot "model" you want.
  _sdf    : sdf/xacro/urdf file. It means an object of sdf you wrote; the file that this plugin
            added.
  _ecm    : Entity manager for ignition entities. every entities are monitored by it.
  &       : If there is a factor in original function but do not use that factor, then "&" could be
            used without warning. (TBR)
  */
  void Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &) override
  {
    // Enter the model name for the plugin
    if (_sdf->HasElement("model_name")) {
      this->model_name_ = _sdf->Get<std::string>("model_name");
    } else {
      std::cerr<<"[ERROR] Missing <model_name> parameter in SDF. Check your SDF/Xacro/URDF file."<<std::endl;
      return;
    }
    
    // Crate a ROS 2 node for the plugin
    this->node_ = std::make_shared<rclcpp::Node>("imu_plugin_node");

    // Create a publisher for model data in world frame
    this->model_pos_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>(
                            this->model_name_+"/world_position", 10);
    this->model_vel_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>(
                            this->model_name_+"/world_velocity", 10);
    this->model_acc_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>(
                            this->model_name_+"/world_acceleration", 10);
    this->model_ang_pos_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>(
                                this->model_name_+"/euler_angle", 10); 
    this->model_ang_quat_pub_ = this->node_->create_publisher<geometry_msgs::msg::Quaternion>(
                                this->model_name_+"/quaternion", 10);
    this->model_ang_vel_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>(
                                this->model_name_+"/euler_rate", 10);
    this->model_ang_acc_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>(
                                this->model_name_+"/euler_acceleration", 10);
    
    // Create a publisher for model data in body frame
    this->model_body_lin_vel_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>(
                                      this->model_name_+"/body_velocity", 10);
    this->model_body_lin_acc_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>(
                                      this->model_name_+"/body_acceleration", 10);
    this->model_body_ang_vel_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>(
                                      this->model_name_+"/body_angle_rate", 10);
    this->model_body_ang_acc_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>(
                                      this->model_name_+"/body_angle_acceleration", 10);

    // Create a publisher timer and callback for model data publishing in world frame
    this->model_pos_timer_ = this->node_->create_wall_timer(
                              std::chrono::milliseconds(100),  // 10 Hz timer
                              std::bind(&ImuPlugin::ModelPosCallback, this));
    this->model_vel_timer_ = this->node_->create_wall_timer(
                              std::chrono::milliseconds(100),  // 10 Hz timer
                              std::bind(&ImuPlugin::ModelVelCallback, this));
    this->model_acc_timer_ = this->node_->create_wall_timer(
                              std::chrono::milliseconds(100),  // 10 Hz timer
                              std::bind(&ImuPlugin::ModelAccCallback, this));    
    this->model_ang_pos_timer_ = this->node_->create_wall_timer(
                                  std::chrono::milliseconds(100),     // 10 Hz timer
                                  std::bind(&ImuPlugin::ModelAngPoseCallback, this));
    this->model_ang_quat_timer_ = this->node_->create_wall_timer(
                                  std::chrono::milliseconds(100),     // 10 Hz timer
                                  std::bind(&ImuPlugin::ModelAngQuatCallback, this));
    this->model_ang_vel_timer_ = this->node_->create_wall_timer(
                                  std::chrono::milliseconds(100),     // 10 Hz timer
                                  std::bind(&ImuPlugin::ModelAngVelCallback, this));
    this->model_ang_acc_timer_ = this->node_->create_wall_timer(
                                  std::chrono::milliseconds(100),     // 10 Hz timer
                                  std::bind(&ImuPlugin::ModelAngAccCallback, this));
    
    // Create a publisher timer and callback for model data publishing in body frame
    this->model_body_lin_vel_timer_ = this->node_->create_wall_timer(
                                      std::chrono::milliseconds(100),     // 10 Hz timer
                                      std::bind(&ImuPlugin::ModelBodyLinVelCallback, this));
    this->model_body_lin_acc_timer_ = this->node_->create_wall_timer(
                                      std::chrono::milliseconds(100),     // 10 Hz timer
                                      std::bind(&ImuPlugin::ModelBodyLinAccCallback, this));
    this->model_body_ang_vel_timer_ = this->node_->create_wall_timer(
                                      std::chrono::milliseconds(100),     // 10 Hz timer
                                      std::bind(&ImuPlugin::ModelBodyAngVelCallback, this));
    this->model_body_ang_acc_timer_ = this->node_->create_wall_timer(
                                      std::chrono::milliseconds(100),     // 10 Hz timer
                                      std::bind(&ImuPlugin::ModelBodyAngAccCallback, this));
    
    // Create a shared multi-threaded executor to run both ROS 2 nodes concurrently.
  
    RCLCPP_INFO(this->node_->get_logger(), "[imu_plugin] Starting ROS 2 MultiThreadedExecutor.");

    // Run the executor in its own thread with proper exception handling.
    // This ensures that any exceptions thrown during spinning are caught and logged without crashing the simulation.
    this->ros_spin_thread_ = std::thread([this]() {
      this->executor_.add_node(this->node_);
      try {
        this->executor_.spin();
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->node_->get_logger(), "[imu_plugin] Exception in executor thread: %s", e.what());
      }
    });

    // Attempt to locate the model entity in ECM
    auto model = ignition::gazebo::Model(_entity);
    if (!model.Valid(_ecm)) {
      RCLCPP_ERROR(this->node_->get_logger(), "[imu_plugin] This plugin must be attached to a valid model.");
      return;
    }
    this->model_entity_ = model.Entity();
  }
  /// \brief PreUpdate: Called every simulation iteration before physics update
  /*
  _info : update data in ignition simulation; puasing, dt, simulation time etc.
  _ecm : entity component manager
  */
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override
  {
    // Do nothing if simulation is paused
    if (_info.paused) {
    return;  
    }
    // Get time step
    std::chrono::duration<double> dt = _info.dt;
    this->dt_ = dt.count();
    // Check if dt is valid
    if (dt.count() <= 0)
      return;
  }
  
  /// \brief PostUpdate: Called after physics update, so we can read final states
  /*
  _info : update data in ignition simulation; puasing, dt, simulation time etc.
  _ecm : entity component manager
  */
  void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                  const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    // Do nothing if simulation is paused
    if (_info.paused) {
      return;
    }

    //Get Pose Component
    auto model_pose = _ecm.Component<ignition::gazebo::components::Pose>(this->model_entity_);
    // Check if the model pose component is valid
    if (!model_pose) {
      RCLCPP_WARN(this->node_->get_logger(), "[imu_plugin] Model pose component not found.");
      return;
    }

    // mutually exclusive access to model data
    std::lock_guard<std::mutex> lock(this->data_mutex_);

    // Get the model pose data
    auto &model_pose_data = model_pose->Data(); // gz::math::Pose3d

    // If this is the first update, initialize the last pose and velocity
    if (this->is_init_)
    {
      this->last_pose_ = model_pose_data;
      this->last_vel_ = {0, 0, 0, 0, 0, 0};
      this->is_init_ = false;
      return;
    }

    // time step for calculation
    // We already have dt in PreUpdate but we need to use it here
    // You could use this->dt_
    std::chrono::duration<double> dt = _info.dt;
    if (dt.count() <= 0)
      return;

    // Ready to store model data in world frame
    this->model_data_.clear();

    // Get Orientation
    double qw = model_pose_data.Rot().W();
    double qx = model_pose_data.Rot().X();
    double qy = model_pose_data.Rot().Y();
    double qz = model_pose_data.Rot().Z();
    Vector4d quat;
    quat << qw, qx, qy, qz;

    // Compute position difference
    double dx = model_pose_data.Pos().X() - this->last_pose_.Pos().X();
    double dy = model_pose_data.Pos().Y() - this->last_pose_.Pos().Y();
    double dz = model_pose_data.Pos().Z() - this->last_pose_.Pos().Z();

    // Convert quaternion to rotation matrix for world to body frame
    this->R_wb_ = this->kinematics_.quat2rotm(quat); // Z-Y-X angle sequence
    this->conv_angle_ <<  atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)),
                          asin(2*(qw*qy - qz*qx)),
                          atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
    this->e2b_ << 1, 0, -sin(this->conv_angle_(1)),
                  0, cos(this->conv_angle_(0)), sin(this->conv_angle_(0))*cos(this->conv_angle_(1)),
                  0, -sin(this->conv_angle_(0)), cos(this->conv_angle_(0))*cos(this->conv_angle_(1));
    
    // Get euler angle
    double curr_roll = model_pose_data.Rot().Roll();
    double curr_pitch = model_pose_data.Rot().Pitch();
    double curr_yaw = model_pose_data.Rot().Yaw();
    
    // Get last euler angle
    double last_roll = this->last_pose_.Rot().Roll();
    double last_pitch = this->last_pose_.Rot().Pitch();
    double last_yaw = this->last_pose_.Rot().Yaw();

    // Calculate euler angle differences
    double droll = curr_roll - last_roll;
    double dpitch = curr_pitch - last_pitch;
    double dyaw = curr_yaw - last_yaw;

    // Normalize angle differences to [-π, π]
    while (droll > PI) droll -= 2.0 * PI;
    while (droll < -PI) droll += 2.0 * PI;
    while (dpitch > PI) dpitch -= 2.0 * PI;
    while (dpitch < -PI) dpitch += 2.0 * PI;
    while (dyaw > PI) dyaw -= 2.0 * PI;
    while (dyaw < -PI) dyaw += 2.0 * PI;

    // Compute linear & angular velocity in world frame
    double lin_vel_x = dx / dt.count();
    double lin_vel_y = dy / dt.count();
    double lin_vel_z = dz / dt.count();
    double roll_rate = droll / dt.count();
    double pitch_rate = dpitch / dt.count();
    double yaw_rate = dyaw / dt.count();
    
    // Convert from world linear velocity to body linear velocity
    Vector3d world_vel;
    world_vel << lin_vel_x, lin_vel_y, lin_vel_z;
    Vector3d body_vel = this->R_wb_.transpose() * world_vel;
    
    // Convert euler rate to body rate
    Vector3d angle_rate;
    Vector3d body_rate;
    angle_rate << roll_rate, pitch_rate, yaw_rate;
    body_rate = this->e2b_ * angle_rate;

    // Insert into mean average filter(RollingMean)
    // world frame data part
    this->lin_vel_x_.Push(lin_vel_x);
    this->lin_vel_y_.Push(lin_vel_y);
    this->lin_vel_z_.Push(lin_vel_z);
    this->ang_vel_x_.Push(roll_rate);
    this->ang_vel_y_.Push(pitch_rate);
    this->ang_vel_z_.Push(yaw_rate);
    // body frame data part
    this->body_lin_vel_x_.Push(body_vel(0));
    this->body_lin_vel_y_.Push(body_vel(1));
    this->body_lin_vel_z_.Push(body_vel(2));
    this->body_ang_vel_x_.Push(body_rate(0));
    this->body_ang_vel_y_.Push(body_rate(1));
    this->body_ang_vel_z_.Push(body_rate(2));
    
    // Store present velocity
    std::vector<double> new_vel = {this->lin_vel_x_.Mean(), this->lin_vel_y_.Mean(), this->lin_vel_z_.Mean(),
                                   this->ang_vel_x_.Mean(), this->ang_vel_y_.Mean(), this->ang_vel_z_.Mean(),
                                   this->body_lin_vel_x_.Mean(), this->body_lin_vel_y_.Mean(), this->body_lin_vel_z_.Mean(),
                                   this->body_ang_vel_x_.Mean(), this->body_ang_vel_y_.Mean(), this->body_ang_vel_z_.Mean()};

    // Compute linear acceleration in world frame
    double dvx = new_vel[0] - this->last_vel_[0];
    double dvy = new_vel[1] - this->last_vel_[1];
    double dvz = new_vel[2] - this->last_vel_[2];
    
    // Compute angular acceleration in world frame
    double dwx = new_vel[3] - this->last_vel_[3];
    double dwy = new_vel[4] - this->last_vel_[4];
    double dwz = new_vel[5] - this->last_vel_[5];
    
    // Compute body linear acceleration
    double body_dvx = new_vel[6] - this->last_vel_[6];
    double body_dvy = new_vel[7] - this->last_vel_[7];
    double body_dvz = new_vel[8] - this->last_vel_[8];
        
    // Compute body angular acceleration
    double body_dwx = new_vel[9] - this->last_vel_[11];
    double body_dwy = new_vel[10] - this->last_vel_[11];
    double body_dwz = new_vel[11] - this->last_vel_[11];

    // Normalize angle differences to [-π, π]
    while (dwx > PI) dwx -= 2.0 * PI;
    while (dwx < -PI) dwx += 2.0 * PI;
    while (dwy > PI) dwy -= 2.0 * PI;
    while (dwy < -PI) dwy += 2.0 * PI;
    while (dwz > PI) dwz -= 2.0 * PI;
    while (dwz < -PI) dwz += 2.0 * PI;
    while (body_dwx > PI) body_dwx -= 2.0 * PI;
    while (body_dwx < -PI) body_dwx += 2.0 * PI;
    while (body_dwy > PI) body_dwy -= 2.0 * PI;
    while (body_dwy < -PI) body_dwy += 2.0 * PI;
    while (body_dwz > PI) body_dwz -= 2.0 * PI;
    while (body_dwz < -PI) body_dwz += 2.0 * PI;

    // Compute linear & angular acceleration in world frame
    double lin_acc_x = dvx / dt.count();
    double lin_acc_y = dvy / dt.count();
    double lin_acc_z = dvz / dt.count();
    double ang_acc_x = dwx / dt.count();
    double ang_acc_y = dwy / dt.count();
    double ang_acc_z = dwz / dt.count();
    // Compute body linear acceleration
    double body_lin_acc_x = body_dvx / dt.count();
    double body_lin_acc_y = body_dvy / dt.count();
    double body_lin_acc_z = body_dvz / dt.count();
    // Compute body angular acceleration
    double body_ang_acc_x = body_dwx / dt.count();
    double body_ang_acc_y = body_dwy / dt.count();
    double body_ang_acc_z = body_dwz / dt.count();

    // Insert into mean average filter(RollingMean)
    this->lin_acc_x_.Push(lin_acc_x);
    this->lin_acc_y_.Push(lin_acc_y);
    this->lin_acc_z_.Push(lin_acc_z);
    this->ang_acc_x_.Push(ang_acc_x);
    this->ang_acc_y_.Push(ang_acc_y);
    this->ang_acc_z_.Push(ang_acc_z);
    this->body_lin_acc_x_.Push(body_lin_acc_x);
    this->body_lin_acc_y_.Push(body_lin_acc_y);
    this->body_lin_acc_z_.Push(body_lin_acc_z);
    this->body_ang_acc_x_.Push(body_ang_acc_x);
    this->body_ang_acc_y_.Push(body_ang_acc_y);
    this->body_ang_acc_z_.Push(body_ang_acc_z);

    // Store updated velocity and pose
    this->last_vel_ = new_vel;
    this->last_pose_ = model_pose_data;

    // model world data (ECI frame)
    this->model_data_.push_back(model_pose_data.Pos().X());
    this->model_data_.push_back(model_pose_data.Pos().Y());
    this->model_data_.push_back(model_pose_data.Pos().Z()); // X-Y-Z [m]

    this->model_data_.push_back(this->lin_vel_x_.Mean());
    this->model_data_.push_back(this->lin_vel_y_.Mean());
    this->model_data_.push_back(this->lin_vel_z_.Mean()); // X-Y-Z [m/s]

    this->model_data_.push_back(this->lin_acc_x_.Mean());
    this->model_data_.push_back(this->lin_acc_y_.Mean());
    this->model_data_.push_back(this->lin_acc_z_.Mean()); // X-Y-Z [m/s^2]
    
    this->model_data_.push_back(model_pose_data.Rot().Roll());
    this->model_data_.push_back(model_pose_data.Rot().Pitch());
    this->model_data_.push_back(model_pose_data.Rot().Yaw()); // Roll-Pitch-Yaw [rad]
    
    this->model_data_.push_back(model_pose_data.Rot().W());
    this->model_data_.push_back(model_pose_data.Rot().X());
    this->model_data_.push_back(model_pose_data.Rot().Y());
    this->model_data_.push_back(model_pose_data.Rot().Z()); // W-X-Y-Z [unit quaternion]

    this->model_data_.push_back(this->ang_vel_x_.Mean());
    this->model_data_.push_back(this->ang_vel_y_.Mean());
    this->model_data_.push_back(this->ang_vel_z_.Mean()); // X-Y-Z [rad/s]

    this->model_data_.push_back(this->ang_acc_x_.Mean());
    this->model_data_.push_back(this->ang_acc_y_.Mean());
    this->model_data_.push_back(this->ang_acc_z_.Mean()); // X-Y-Z [rad/s^2]

    // model body data (body frame)
    this->model_data_.push_back(this->body_lin_vel_x_.Mean());
    this->model_data_.push_back(this->body_lin_vel_y_.Mean());
    this->model_data_.push_back(this->body_lin_vel_z_.Mean()); // X-Y-Z [m/s]

    this->model_data_.push_back(this->body_lin_acc_x_.Mean());
    this->model_data_.push_back(this->body_lin_acc_y_.Mean());
    this->model_data_.push_back(this->body_lin_acc_z_.Mean()); // X-Y-Z [m/s^2]
    
    this->model_data_.push_back(this->body_ang_vel_x_.Mean());
    this->model_data_.push_back(this->body_ang_vel_y_.Mean());
    this->model_data_.push_back(this->body_ang_vel_z_.Mean()); // X-Y-Z [rad/s]
    
    this->model_data_.push_back(this->body_ang_acc_x_.Mean());
    this->model_data_.push_back(this->body_ang_acc_y_.Mean());
    this->model_data_.push_back(this->body_ang_acc_z_.Mean()); // X-Y-Z [rad/s^2]
  }

private:
  // Model members for the plugin
  std::string model_name_;
  ignition::gazebo::Entity model_entity_{ignition::gazebo::kNullEntity};
  std::vector<float> model_data_;
  ignition::math::Pose3d last_pose_;
  std::vector<double> last_vel_;

  // data members : mutually exclusive
  std::mutex data_mutex_;

  // ROS 2 Node, Executor, Thread
  std::thread ros_spin_thread_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Node::SharedPtr node_;

  // ROS 2 Publishers for world frame data
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_pos_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_acc_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_ang_pos_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr model_ang_quat_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_ang_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_ang_acc_pub_;

  // ROS 2 Publisher timers for world frame data
  rclcpp::TimerBase::SharedPtr model_pos_timer_;
  rclcpp::TimerBase::SharedPtr model_vel_timer_;
  rclcpp::TimerBase::SharedPtr model_acc_timer_;
  rclcpp::TimerBase::SharedPtr model_ang_pos_timer_;
  rclcpp::TimerBase::SharedPtr model_ang_quat_timer_;
  rclcpp::TimerBase::SharedPtr model_ang_vel_timer_;
  rclcpp::TimerBase::SharedPtr model_ang_acc_timer_;

  // ROS 2 Publishers for body frame data
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_body_lin_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_body_lin_acc_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_body_ang_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr model_body_ang_acc_pub_;

  // ROS 2 Publisher timers for body frame data
  rclcpp::TimerBase::SharedPtr  model_body_lin_vel_timer_;
  rclcpp::TimerBase::SharedPtr  model_body_lin_acc_timer_;
  rclcpp::TimerBase::SharedPtr  model_body_ang_vel_timer_;
  rclcpp::TimerBase::SharedPtr  model_body_ang_acc_timer_;
  
  // Time step
  double dt_{0.0};
  // Flag to check if the plugin is initialized
  bool is_init_{true};

  // Kinematics object
  Kinematics kinematics_;
  
  //Conversion matrices
  Matrix3d R_wb_;
  Matrix3d e2b_;

  // Conversion factors
  Vector4d curr_quat_;
  Vector3d conv_angle_;

  // Callbacks for model data publishing
  // world frame part
  void ModelPosCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->model_data_[0];
    msg.y = this->model_data_[1];
    msg.z = this->model_data_[2];

    this->model_pos_pub_->publish(msg);
  }
  void ModelVelCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->model_data_[3];
    msg.y = this->model_data_[4];
    msg.z = this->model_data_[5];

    this->model_vel_pub_->publish(msg);
  }
  void ModelAccCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->model_data_[6];
    msg.y = this->model_data_[7];
    msg.z = this->model_data_[8];

    this->model_acc_pub_->publish(msg);
  }
  void ModelAngPoseCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->model_data_[9];
    msg.y = this->model_data_[10];
    msg.z = this->model_data_[11];

    this->model_ang_pos_pub_->publish(msg);
  }
  void ModelAngQuatCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Quaternion msg;

    msg.w = this->model_data_[12];
    msg.x = this->model_data_[13];
    msg.y = this->model_data_[14];
    msg.z = this->model_data_[15];
    
    this->model_ang_quat_pub_->publish(msg);
  }
  void ModelAngVelCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->model_data_[16];
    msg.y = this->model_data_[17];
    msg.z = this->model_data_[18];

    this->model_ang_vel_pub_->publish(msg);
  }
  void ModelAngAccCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->model_data_[19];
    msg.y = this->model_data_[20];
    msg.z = this->model_data_[21];

    this->model_ang_acc_pub_->publish(msg);
  }
  // body frame part
  void ModelBodyLinVelCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->model_data_[22];
    msg.y = this->model_data_[23];
    msg.z = this->model_data_[24];

    this->model_body_lin_vel_pub_->publish(msg);
  }
  void ModelBodyLinAccCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->model_data_[25];
    msg.y = this->model_data_[26];
    msg.z = this->model_data_[27];

    this->model_body_lin_acc_pub_->publish(msg);
  }
  void ModelBodyAngVelCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->model_data_[28];
    msg.y = this->model_data_[29];
    msg.z = this->model_data_[30];

    this->model_body_ang_vel_pub_->publish(msg);
  }
  void ModelBodyAngAccCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->model_data_[31];
    msg.y = this->model_data_[32];
    msg.z = this->model_data_[33];

    this->model_body_ang_acc_pub_->publish(msg);
  }

  // calculation members for model data smoothing in world frame
  ignition::math::RollingMean lin_vel_x_, lin_vel_y_, lin_vel_z_;
  ignition::math::RollingMean lin_acc_x_, lin_acc_y_, lin_acc_z_;
  ignition::math::RollingMean ang_vel_x_, ang_vel_y_, ang_vel_z_;
  ignition::math::RollingMean ang_acc_x_, ang_acc_y_, ang_acc_z_;

  // calculation members for model data smoothing in world frame
  ignition::math::RollingMean body_lin_vel_x_, body_lin_vel_y_, body_lin_vel_z_;
  ignition::math::RollingMean body_lin_acc_x_, body_lin_acc_y_, body_lin_acc_z_;
  ignition::math::RollingMean body_ang_vel_x_, body_ang_vel_y_, body_ang_vel_z_;
  ignition::math::RollingMean body_ang_acc_x_, body_ang_acc_y_, body_ang_acc_z_;
}; // class ImuPlugin
}  // namespace imu_plugin

// Register the plugin with Ignition
IGNITION_ADD_PLUGIN(
  imu_plugin::ImuPlugin, // this is the registration name for this plugin
  ignition::gazebo::System,
  imu_plugin::ImuPlugin::ISystemConfigure,
  imu_plugin::ImuPlugin::ISystemPreUpdate,
  imu_plugin::ImuPlugin::ISystemPostUpdate
)