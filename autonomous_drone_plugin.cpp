#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <iostream>

namespace drone_sim
{
  class AutonomousDrone
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemUpdate
  {
    private:
      ignition::gazebo::Model model;
      bool takeoff = false;
      bool movingUp = false;
      bool movingDown = false;
      int currentDirection = 1; // 1 = right, -1 = left

      // Movement Parameters
      double speed = 1.0;
      double altitude = 6.5;
      double stepSize = 1.0; // Row spacing

      // Grid Boundaries
      double X_min = 0.0, X_max = 10.0;
      double Y_min = 0.0, Y_max = 10.0;
      double currentRow = Y_min;

    public:
      void Configure(const ignition::gazebo::Entity &_entity,
                     const std::shared_ptr<const sdf::Element> &_sdf,
                     ignition::gazebo::EntityComponentManager &_ecm,
                     ignition::gazebo::EventManager &) override
      {
        this->model = ignition::gazebo::Model(_entity);
        if (!this->model.Valid(_ecm))
        {
          std::cerr << "[ERROR] AutonomousDrone plugin failed to configure." << std::endl;
          return;
        }
      }

      void Update(const ignition::gazebo::UpdateInfo &_info,
                  ignition::gazebo::EntityComponentManager &_ecm) override
      {
        if (_info.paused)
          return;

        auto poseComp = _ecm.Component<ignition::gazebo::components::Pose>(this->model.Entity());
        if (!poseComp)
          return;

        auto position = poseComp->Data().Pos();
        auto velocityComp = _ecm.Component<ignition::gazebo::components::LinearVelocityCmd>(this->model.Entity());

        if (!velocityComp)
        {
          _ecm.CreateComponent(this->model.Entity(),
                               ignition::gazebo::components::LinearVelocityCmd({0, 0, 0}));
          return;
        }

        //  Takeoff Logic
        if (!this->takeoff)
        {
          velocityComp->Data() = ignition::math::Vector3d(0, 0, speed);
          if (position.Z() >= altitude - 0.5)
            this->takeoff = true;
          return;
        }

        // Movement Logic: Covering Grid in a Lawn Mower Pattern
        if (movingDown)
        {
          velocityComp->Data() = ignition::math::Vector3d(0, -speed, 0);
          if (position.Y() <= Y_min + 0.5) // If drone has reached Y_min, start moving right
          {
            movingDown = false;
            movingUp = false;
            currentRow = Y_min;
            currentDirection = 1; // Reset to moving right
          }
          return;
        }

        if (!movingUp)
        {
          // Move Left or Right
          if ((currentDirection == 1 && position.X() >= X_max) || (currentDirection == -1 && position.X() <= X_min))
          {
            movingUp = true;  // Start moving up
          }
          else
          {
            velocityComp->Data() = ignition::math::Vector3d(currentDirection * speed, 0, 0);
          }
        }
        else
        {
          // Move Up
          if (position.Y() < Y_max)
          {
            velocityComp->Data() = ignition::math::Vector3d(0, speed, 0);
          }

          if (position.Y() >= currentRow + stepSize)
          {
            movingUp = false;
            currentRow = position.Y();
            currentDirection *= -1; // Switch left/right movement
          }
        }

        //  Restart Grid Scan When Reaching Y_max
        if (position.Y() >= Y_max - 0.5)
        {
          //std::cout << "[INFO] Reached max Y. Moving back to Y_min." << std::endl;
          movingDown = true;
        }

        // Debugging Log
        //std::cout << "[DEBUG] Position: X=" << position.X()
          //        << ", Y=" << position.Y()
            //      << ", Z=" << position.Z()
              //    << " | Moving Up: " << movingUp
                //  << " | Moving Down: " << movingDown
                  //<< " | Direction: " << ((currentDirection == 1) ? "Right" : "Left") 
              //    << std::endl;
      }
  };
}

// Register the plugin
IGNITION_ADD_PLUGIN(drone_sim::AutonomousDrone,
                    ignition::gazebo::System,
                    drone_sim::AutonomousDrone::ISystemConfigure,
                    drone_sim::AutonomousDrone::ISystemUpdate)
IGNITION_ADD_PLUGIN_ALIAS(drone_sim::AutonomousDrone, "AutonomousDrone")

