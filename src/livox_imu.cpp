// Livox built-in IMU for Gazebo Sim, published exactly as livox_ros_driver2 does
// (Lddc::InitImuMsg): sensor_msgs/Imu in the lidar's frame, angular velocity in
// rad/s, linear acceleration in g (not m/s^2; FAST-LIO etc. rescale it), no
// orientation.
//
// Attach inside an imu <sensor> placed at the IMU's position (see urdf/mid360.xacro):
//   <plugin filename="livox_gz_sim" name="livox_gz_sim::LivoxImu">
//     <topic>livox/imu</topic>  <frame_id>livox_frame</frame_id>
//   </plugin>
// Rate and noise come from the imu sensor itself.

#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/imu.pb.h>
// livox_lidar.cpp defines the library's plugin hook
#define GZ_PLUGIN_REGISTER_MORE_TRANS_UNITS
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace livox_gz_sim
{

constexpr double kGravity = 9.80665;  // m/s^2 per g

class LivoxImu : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    auto get = [&](const char *key, auto dflt) {
      return sdf->HasElement(key) ? sdf->Get<decltype(dflt)>(key) : dflt;
    };
    frame_id_ = get("frame_id", std::string("livox_frame"));
    std::string topic = get("topic", std::string("livox/imu"));

    auto sensor = ecm.Component<gz::sim::components::Imu>(entity);
    if (!sensor) {
      gzerr << "livox_gz_sim::LivoxImu must be attached to an imu sensor\n";
      return;
    }
    std::string gz_topic = sensor->Data().Topic();
    if (gz_topic.empty()) {
      gzerr << "livox_gz_sim::LivoxImu: give the imu sensor a <topic>\n";
      return;
    }

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    auto name_c = ecm.Component<gz::sim::components::Name>(entity);
    std::string name = name_c ? name_c->Data() : "livox_imu";
    node_ = std::make_shared<rclcpp::Node>("livox_sim_" + name);
    pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS().keep_last(50).reliable());

    if (!gz_node_.Subscribe(gz_topic, &LivoxImu::OnImu, this)) {
      gzerr << "livox_gz_sim::LivoxImu: cannot subscribe to [" << gz_topic << "]\n";
      return;
    }
    gzmsg << "Livox IMU [" << name << "]: on ROS [" << topic << "] in [" << frame_id_ << "]\n";
  }

private:
  void OnImu(const gz::msgs::IMU &in)
  {
    sensor_msgs::msg::Imu m;
    m.header.stamp = rclcpp::Time(int64_t(in.header().stamp().sec()) * 1000000000 + in.header().stamp().nsec());
    m.header.frame_id = frame_id_;
    m.angular_velocity.x = in.angular_velocity().x();
    m.angular_velocity.y = in.angular_velocity().y();
    m.angular_velocity.z = in.angular_velocity().z();
    m.linear_acceleration.x = in.linear_acceleration().x() / kGravity;
    m.linear_acceleration.y = in.linear_acceleration().y() / kGravity;
    m.linear_acceleration.z = in.linear_acceleration().z() / kGravity;
    pub_->publish(m);
  }

  std::string frame_id_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  gz::transport::Node gz_node_;  // last: unsubscribes before the state above goes away
};

}  // namespace livox_gz_sim

GZ_ADD_PLUGIN(livox_gz_sim::LivoxImu, gz::sim::System, livox_gz_sim::LivoxImu::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(livox_gz_sim::LivoxImu, "livox_gz_sim::LivoxImu")
