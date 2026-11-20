// Gió lên thân drone, tính MỖI BƯỚC VẬT LÝ (1 ms) - plugin gz-sim 8.
//
// Vì sao không dùng thứ có sẵn của Gazebo (đọc source gz-sim8, 20/11/2026):
//  - WindEffects: F = m * k * (v_gió - v_link), chỉ TUYẾN TÍNH, không có phần cản thân bậc hai.
//  - ApplyLinkWrench /wrench/persistent: lực CỘNG DỒN theo từng bản tin, /wrench/clear xoá luôn bản tin vừa
//    tới -> đổi lực 50 lần/s là chập chờn.
//  - LiftDrag của cánh TRỪ gió nền của world: world phải để gió nền 0 khi dùng plugin này, không thì
//    cánh quạt chịu gió hai lần.
//
// Mô hình = SITL ArduPilot (libraries/SITL/SIM_Motor.cpp + SIM_Frame.cpp), v = v_drone - v_gió (ngang):
//   cản cánh quạt (momentum drag) : F = -rotor_drag * v            rotor_drag = n * mdrag * sqrt(rho*A*T)
//   cản thân                       : F = -0.5 * rho * cda * |v| * v
// Gió (m/s, hệ world ENU) đọc từ topic gz.msgs.Vector3d `<topic>` - script ngoài sinh gió giật.
// Lực đặt ở TRỌNG TÂM link (Link::AddWorldForce): bỏ qua mô-men do tâm áp lệch trọng tâm.
#include <mutex>
#include <string>

#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace fire_drone
{
class WindDrag : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    const gz::sim::Model model(_entity);
    const auto name = _sdf->Get<std::string>("link_name", "base_link").first;
    this->link = gz::sim::Link(model.LinkByName(_ecm, name));
    if (!this->link.Valid(_ecm))
    {
      gzerr << "WindDrag: khong co link [" << name << "]\n";
      return;
    }
    this->link.EnableVelocityChecks(_ecm, true);
    this->rotorDrag = _sdf->Get<double>("rotor_drag", 1.1).first;
    this->cda = _sdf->Get<double>("cda", 0.08).first;
    this->rho = _sdf->Get<double>("air_density", 1.2041).first;
    const auto topic = _sdf->Get<std::string>("topic", "/wind").first;
    this->node.Subscribe(topic, &WindDrag::OnWind, this);
    gzmsg << "WindDrag: link " << name << ", rotor_drag " << this->rotorDrag << " N/(m/s), cda "
          << this->cda << " m2, gio tu " << topic << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->link.Valid(_ecm))
      return;
    const auto vel = this->link.WorldLinearVelocity(_ecm);
    if (!vel)
      return;
    gz::math::Vector3d w;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      w = this->wind;
    }
    // ponytail: chỉ phương ngang - phương đứng LiftDrag đã tính dòng vào cánh khi leo/hạ
    gz::math::Vector3d v(vel->X() - w.X(), vel->Y() - w.Y(), 0.0);
    const auto force = -(this->rotorDrag * v + 0.5 * this->rho * this->cda * v.Length() * v);
    this->link.AddWorldForce(_ecm, force);
  }

private:
  void OnWind(const gz::msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->wind.Set(_msg.x(), _msg.y(), _msg.z());
  }

  gz::sim::Link link;
  gz::transport::Node node;
  std::mutex mutex;
  gz::math::Vector3d wind{0, 0, 0};
  double rotorDrag{1.1}, cda{0.08}, rho{1.2041};
};
}  // namespace fire_drone

GZ_ADD_PLUGIN(fire_drone::WindDrag, gz::sim::System, fire_drone::WindDrag::ISystemConfigure,
              fire_drone::WindDrag::ISystemPreUpdate)
