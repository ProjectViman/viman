#ifndef PLUGIN_ALTIMETER_H
#define PLUGIN_ALTIMETER_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math4/ignition/math.hh>

#define DEFAULT_ELEVATION 0.0

namespace gazebo{

class Altimeter : public ModelPlugin{
public:
  Altimeter();
  virtual ~Altimeter();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  physics::WorldPtr world;

  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::Publisher altimeter_publisher_;

  geometry_msgs::PointStamped height_;

  std::string namespace_;
  std::string altimeter_topic_;
  std::string link_name_;
  std::string frame_id_;

  double elevation_;
  double noise_;
  double drift_;
  double driftFrequency_;
  double offset_;
  
  common::Time last_time;
  event::ConnectionPtr updateConnection;
};

}

#endif // PLUGIN_ALTIMETER_H
