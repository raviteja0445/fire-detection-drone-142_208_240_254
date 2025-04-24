#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/double.pb.h>

namespace fire_sim
{
  class FireEmitterPlugin : public ignition::gazebo::System,
                            public ignition::gazebo::ISystemConfigure,
                            public ignition::gazebo::ISystemUpdate
  {
    ignition::transport::Node node;
    ignition::transport::Node::Publisher tempPub;

  public:
    void Configure(const ignition::gazebo::Entity &,
                   const std::shared_ptr<const sdf::Element> &,
                   ignition::gazebo::EntityComponentManager &,
                   ignition::gazebo::EventManager &) override
    {
      // Publish to custom topic for simulation (can adjust to match thermal sensor topic if needed)
      this->tempPub = node.Advertise<ignition::msgs::Double>("/fire/temperature");
    }

    void Update(const ignition::gazebo::UpdateInfo &,
                ignition::gazebo::EntityComponentManager &) override
    {
      ignition::msgs::Double msg;
      msg.set_data(1000.0); // Fire temperature in Celsius
      tempPub.Publish(msg);
    }
  };
}

IGNITION_ADD_PLUGIN(fire_sim::FireEmitterPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(fire_sim::FireEmitterPlugin, "FireEmitterPlugin")