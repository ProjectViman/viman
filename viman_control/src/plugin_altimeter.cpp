#include "plugin_altimeter.h"

namespace gazebo {

// Contructor
Altimeter::Altimeter(){}

// Destructor
Altimeter::~Altimeter(){
  node_handle_->shutdown();
  delete node_handle_;
}

// Load the plugin
void Altimeter::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  
  world = _model->GetWorld();
  
  ROS_INFO("Loaded altimeter plugin.");
    
  link = _model->GetLink();
  link_name_ = link->GetName();
  
  if (_sdf->HasElement("bodyName")){
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = _model->GetLink(link_name_);
  }
  
  if (!link){
    ROS_FATAL("plugin_altimeter error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  namespace_.clear();
  frame_id_ = link->GetName();
  altimeter_topic_ = "/height";
  elevation_ = DEFAULT_ELEVATION;

  // load parameters
  if (_sdf->HasElement("robotNamespace"))     namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  if (_sdf->HasElement("frameId"))            frame_id_ = _sdf->GetElement("frameId")->Get<std::string>();
  if (_sdf->HasElement("altimeterTopicName")) altimeter_topic_ = _sdf->GetElement("altimeterTopicName")->Get<std::string>();
  if (_sdf->HasElement("elevation"))          elevation_ = _sdf->GetElement("elevation")->Get<double>();
  if (_sdf->HasElement("drift"))              drift_ = _sdf->GetElement("drift")->Get<double>();
  if (_sdf->HasElement("driftFrequency"))     driftFrequency_ = _sdf->GetElement("driftFrequency")->Get<double>();
  if (_sdf->HasElement("offset"))             offset_ = _sdf->GetElement("offset")->Get<double>();

  height_.header.frame_id = frame_id_;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()){
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  // advertise altimeter
  if(!altimeter_topic_.empty()){
    altimeter_publisher_ = node_handle_->advertise<geometry_msgs::PointStamped>(altimeter_topic_, 1024);
  }

  // connect Update function
  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Altimeter::Update, this));
}

void Altimeter::Reset(){}

// Update
void Altimeter::Update(){
#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time sim_time = world->SimTime();
#else
  common::Time sim_time = world->GetSimTime();
#endif
  double dt = (sim_time - last_time).Double();
  if (dt == 0.0) return;

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d pose = link->WorldPose();
  double height = pose.Pos().Z();
#else
  math::Pose pose = link->GetWorldPose();
  double height = pose.pos.z;
#endif

    height_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
    height_.point.z = height;
    altimeter_publisher_.publish(height_);
    
    // save last time stamp
    last_time = sim_time;  
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Altimeter)

} // namespace gazebo
