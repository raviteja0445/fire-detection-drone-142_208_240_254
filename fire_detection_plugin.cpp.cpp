#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/double.pb.h>
#include <iostream>

namespace drone_sim
{
  class FireDetectionPlugin : public ignition::gazebo::System,
                              public ignition::gazebo::ISystemConfigure,
                              public ignition::gazebo::ISystemPreUpdate
  {
    ignition::transport::Node node;
    bool fireDetected = false;
    bool subscribed = false;
    double currentTemperature = 0.0;

  public:
    void Configure(const ignition::gazebo::Entity &,
                   const std::shared_ptr<const sdf::Element> &,
                   ignition::gazebo::EntityComponentManager &,
                   ignition::gazebo::EventManager &) override
    {
    }

    void PreUpdate(const ignition::gazebo::UpdateInfo &,
                   ignition::gazebo::EntityComponentManager &) override
    {
      if (!subscribed)
      {
        std::function<void(const ignition::msgs::Double &, const ignition::transport::MessageInfo &)> cb =
            [this](const ignition::msgs::Double &msg, const ignition::transport::MessageInfo &)
            {
              this->currentTemperature = msg.data();
            };

        node.Subscribe("/fire/temperature", cb);
        subscribed = true;
      }

      // Detect based on temperature threshold
      if (!fireDetected && currentTemperature > 300.0) // Example threshold
      {
        std::cout << "🔥 Fire detected by thermal sensor! Temperature: " << currentTemperature << "°C" << std::endl;
        fireDetected = true;
      }
    }
  };
}

IGNITION_ADD_PLUGIN(drone_sim::FireDetectionPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(drone_sim::FireDetectionPlugin, "FireDetectionPlugin")