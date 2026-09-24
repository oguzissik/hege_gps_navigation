// Simulation actuator adapter: PX4 motor shaft speed -> rear wheel speed.
// Ideal velocity drive, not a calibrated motor/tyre/traction model.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace hege {
class WheelDrive : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    model = gz::sim::Model(entity);
    ratio = sdf->Get<double>("gear_ratio", 10.0).first;
    if (!std::isfinite(ratio) || ratio <= 0) return;
    auto joint = sdf->FindElement("joint_name");
    while (joint) {
      names.push_back(joint->Get<std::string>());
      joint = joint->GetNextElement("joint_name");
    }
    ready = node.Subscribe("/model/" + model.Name(ecm) + "/command/motor_speed",
                           &WheelDrive::OnCommand, this);
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (!ready || info.paused) return;
    // All configured joints must resolve before applying any command.
    std::vector<gz::sim::Entity> joints;
    for (const auto &name : names) {
      auto entity = model.JointByName(ecm, name);
      if (entity == gz::sim::kNullEntity) return;
      joints.push_back(entity);
    }
    const auto latest = sequence.load();
    if (latest != previousSequence) {
      previousSequence = latest;
      lastCommandTime = info.simTime;
    }
    // Stop the ideal drive if PX4 exits while Gazebo continues to run.
    const bool fresh = latest > 0 && info.simTime >= lastCommandTime &&
        info.simTime - lastCommandTime <= std::chrono::milliseconds(500);
    const double velocity = fresh ? shaftSpeed.load() / ratio : 0.0;
    for (auto entity : joints) {
      auto component = ecm.Component<gz::sim::components::JointVelocityCmd>(entity);
      if (!component) {
        ecm.CreateComponent(entity, gz::sim::components::JointVelocityCmd({velocity}));
      } else {
        ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(entity, {velocity});
      }
    }
  }

 private:
  void OnCommand(const gz::msgs::Actuators &message) {
    const double value = message.velocity_size() ? message.velocity(0) : 0;
    shaftSpeed.store(std::isfinite(value) ? value : 0);
    sequence.fetch_add(1);
  }
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::transport::Node node;
  std::vector<std::string> names;
  std::atomic<double> shaftSpeed{0};
  std::atomic<std::uint64_t> sequence{0};
  std::uint64_t previousSequence{0};
  std::chrono::steady_clock::duration lastCommandTime{};
  double ratio{10};
  bool ready{false};
};
}
GZ_ADD_PLUGIN(hege::WheelDrive, gz::sim::System,
              hege::WheelDrive::ISystemConfigure, hege::WheelDrive::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(hege::WheelDrive, "hege::WheelDrive")
