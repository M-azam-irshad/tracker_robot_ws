// Visual-only tread animation for a tracked robot (Gazebo Harmonic / gz-sim 8).
//
// Moves N "lug" visuals along a closed rounded-triangle path (hull of 3 circles) in the belt link frame,
// driven by the angle of one drive joint.  Path construction == build_path() in generate_meshes.py.
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Element.hh>

namespace tracked_robot
{

class TrackAnimator
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm)) {
      gzerr << "[TrackAnimator] plugin must be attached to a model\n";
      return;
    }

    const std::string beltLink = _sdf->Get<std::string>("belt_link", "").first;
    const std::string driveJoint = _sdf->Get<std::string>("drive_joint", "").first;
    const std::string lugPrefix = _sdf->Get<std::string>("lug_prefix", "lug_").first;
    const int count = _sdf->Get<int>("lug_count", 0).first;
    this->driveRadius = _sdf->Get<double>("drive_radius", 0.08).first;
    this->direction = _sdf->Get<double>("direction", -1.0).first;
    const std::string circles = _sdf->Get<std::string>("circles", "").first;

    // circles: "x z r  x z r  x z r"
    std::vector<Circle> c;
    {
      std::istringstream iss(circles);
      double x, z, r;
      while (iss >> x >> z >> r) {c.push_back({x, z, r});}
    }
    if (count <= 0 || c.size() < 3) {
      gzerr << "[TrackAnimator] bad parameters (lug_count / circles)\n";
      return;
    }
    this->BuildPath(c);

    const gz::sim::Entity linkEnt = this->model.LinkByName(_ecm, beltLink);
    const gz::sim::Entity jointEnt = this->model.JointByName(_ecm, driveJoint);
    if (linkEnt == gz::sim::kNullEntity || jointEnt == gz::sim::kNullEntity) {
      gzerr << "[TrackAnimator] cannot find link '" << beltLink << "' or joint '" << driveJoint << "'\n";
      return;
    }
    this->joint = gz::sim::Joint(jointEnt);
    this->joint.EnablePositionCheck(_ecm, true);

    for (int i = 0; i < count; ++i) {
      const std::string name = lugPrefix + std::to_string(i);
      const gz::sim::Entity v = _ecm.EntityByComponents(
        gz::sim::components::ParentEntity(linkEnt), gz::sim::components::Name(name));
      if (v == gz::sim::kNullEntity) {
        gzwarn << "[TrackAnimator] visual '" << name << "' not found under link '" << beltLink << "'\n";
        continue;
      }
      this->lugs.push_back(v);
      this->lugOffset.push_back(this->total * i / count);
    }
    this->ready = !this->lugs.empty();
    gzmsg << "[TrackAnimator] " << beltLink << ": animating " << this->lugs.size()
          << " lugs, path length " << this->total << " m\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (!this->ready || _info.paused) {return;}
    const auto pos = this->joint.Position(_ecm);
    if (!pos || pos->empty()) {return;}

    const double s = this->direction * this->driveRadius * (*pos)[0];
    if (this->started && std::abs(s - this->lastS) < 1e-4) {return;}
    this->started = true;
    this->lastS = s;

    for (size_t i = 0; i < this->lugs.size(); ++i) {
      double u = std::fmod(this->lugOffset[i] + s, this->total);
      if (u < 0.0) {u += this->total;}
      double x, z, pitch;
      this->PointAt(u, x, z, pitch);
      _ecm.SetComponentData<gz::sim::components::Pose>(
        this->lugs[i], gz::math::Pose3d(x, 0.0, z, 0.0, pitch, 0.0));
    }
  }

private:
  struct Circle {double x, z, r;};

  // ---- path: hull of the circles, counter-clockwise, as a dense closed polyline ----
  void BuildPath(const std::vector<Circle> & c)
  {
    const size_t n = c.size();
    std::vector<std::array<double, 2>> norms(n);
    for (size_t i = 0; i < n; ++i) {
      const Circle & A = c[i];
      const Circle & B = c[(i + 1) % n];
      const double dx = B.x - A.x, dz = B.z - A.z;
      const double d = std::hypot(dx, dz);
      const double ux = dx / d, uz = dz / d;
      const double px = uz, pz = -ux;               // u rotated -90 deg
      const double k = (A.r - B.r) / d;
      const double sq = std::sqrt(std::max(0.0, 1.0 - k * k));
      norms[i] = {k * ux + sq * px, k * uz + sq * pz};
    }
    this->px.clear();
    this->pz.clear();
    for (size_t i = 0; i < n; ++i) {
      const auto & nin = norms[(i + n - 1) % n];
      const auto & nout = norms[i];
      const double a0 = std::atan2(nin[1], nin[0]);
      double a1 = std::atan2(nout[1], nout[0]);
      while (a1 < a0) {a1 += 2.0 * M_PI;}
      const int k = std::max(2, static_cast<int>((a1 - a0) * 180.0 / M_PI / 1.0));
      for (int j = 0; j <= k; ++j) {
        const double a = a0 + (a1 - a0) * j / k;
        this->px.push_back(c[i].x + c[i].r * std::cos(a));
        this->pz.push_back(c[i].z + c[i].r * std::sin(a));
      }
    }
    const size_t m = this->px.size();
    this->cum.assign(m + 1, 0.0);
    for (size_t i = 0; i < m; ++i) {
      const size_t j = (i + 1) % m;
      this->cum[i + 1] = this->cum[i] + std::hypot(this->px[j] - this->px[i], this->pz[j] - this->pz[i]);
    }
    this->total = this->cum[m];
  }

  void PointAt(double u, double & x, double & z, double & pitch) const
  {
    const size_t m = this->px.size();
    size_t i = static_cast<size_t>(
      std::upper_bound(this->cum.begin(), this->cum.end(), u) - this->cum.begin());
    i = (i == 0) ? 0 : i - 1;
    if (i >= m) {i = m - 1;}
    const size_t j = (i + 1) % m;
    const double seg = this->cum[i + 1] - this->cum[i];
    const double t = seg > 1e-12 ? (u - this->cum[i]) / seg : 0.0;
    x = this->px[i] + (this->px[j] - this->px[i]) * t;
    z = this->pz[i] + (this->pz[j] - this->pz[i]) * t;
    pitch = -std::atan2(this->pz[j] - this->pz[i], this->px[j] - this->px[i]);
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Joint joint{gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> lugs;
  std::vector<double> lugOffset;
  std::vector<double> px, pz, cum;
  double total{1.0};
  double driveRadius{0.08};
  double direction{-1.0};
  double lastS{0.0};
  bool started{false};
  bool ready{false};
};

}  // namespace tracked_robot

GZ_ADD_PLUGIN(
  tracked_robot::TrackAnimator,
  gz::sim::System,
  tracked_robot::TrackAnimator::ISystemConfigure,
  tracked_robot::TrackAnimator::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(tracked_robot::TrackAnimator, "tracked_robot::TrackAnimator")
