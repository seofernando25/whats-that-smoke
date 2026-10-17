// Reference TT gearmotor dynamics at the gearbox output; no hardware access.
#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <vector>
#include <sstream>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/transport/Node.hh>

namespace ugv
{
class TTMotorSystem final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemReset
{
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher telemetry;
  std::array<gz::sim::Entity,4> joints{};
  const std::array<std::string,4> names{{"front_left_wheel_joint",
      "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"}};
  std::mutex mutex;
  double linear{0}, angular{0}, radius{.03175}, track{.12065};
  double stall{.0784532}, omega0{26.17994}, feedback{.06};
  double integralGain{.6}, brakeDamping{.06}, brakeResistance{.01};
  std::array<double,4> wheelInertia{{1e-5,1e-5,1e-5,1e-5}};
  double pwmLeft{0}, pwmRight{0}, leftVoltageGain{1.475}, rightVoltageGain{1.475};
  bool pwmMode{false};
  std::chrono::steady_clock::time_point lastCommand{};
  std::array<double,4> integral{};
  std::array<std::vector<std::pair<double,double>>,9> pwmMap;
  std::array<double,9> riseTau{};
  std::array<double,4> filteredDuty{};
  std::chrono::steady_clock::duration lastPub{};

  public: void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    model = gz::sim::Model(entity);
    radius = sdf->Get<double>("wheel_radius", radius).first;
    track = sdf->Get<double>("wheel_separation", track).first;
    stall = sdf->Get<double>("stall_torque", stall).first;
    omega0 = sdf->Get<double>("no_load_speed", omega0).first;
    feedback = sdf->Get<double>("velocity_feedback", feedback).first;
    integralGain = sdf->Get<double>("velocity_integral", integralGain).first;
    brakeDamping=sdf->Get<double>("brake_damping",brakeDamping).first;
    brakeResistance=sdf->Get<double>("brake_resistance",brakeResistance).first;
    if (radius <= 0 || track <= 0 || stall <= 0 || omega0 <= 0 || brakeDamping<0 || brakeResistance<0)
      throw std::runtime_error("TT motor parameters must be positive");
    leftVoltageGain=sdf->Get<double>("pwm_left_voltage_gain",leftVoltageGain).first;
    rightVoltageGain=sdf->Get<double>("pwm_right_voltage_gain",rightVoltageGain).first;
    if(sdf->HasElement("empirical_pwm_map"))
    {
      std::istringstream rows(sdf->Get<std::string>("empirical_pwm_map"));
      int f,t;double pwm,effort;
      while(rows>>f>>t>>pwm>>effort)
      {
        if(f < -1 || f > 1 || t < -1 || t > 1 || pwm<0 || pwm>1800 || effort<0 || effort>1)
          throw std::runtime_error("Invalid empirical PWM row");
        pwmMap[(f+1)*3+t+1].push_back({pwm,effort});
      }
      for(auto &curve:pwmMap) std::sort(curve.begin(),curve.end());
    }
    if(sdf->HasElement("pwm_rise_tau"))
    {
      std::istringstream rows(sdf->Get<std::string>("pwm_rise_tau"));
      int f,t;double tau;
      while(rows>>f>>t>>tau)
      {
        if(f < -1 || f > 1 || t < -1 || t > 1 || tau<0 || tau>2)
          throw std::runtime_error("Invalid PWM rise lag");
        riseTau[(f+1)*3+t+1]=tau;
      }
    }
    node.Subscribe("/model/freenove/wheel_duty", &TTMotorSystem::PWM, this);
    node.Subscribe("/model/freenove/cmd_vel", &TTMotorSystem::Command, this);
    telemetry = node.Advertise<gz::msgs::Double_V>("/model/freenove/motor_state");
  }

  double EffectiveDuty(double left,double right)
  {
    auto sign=[](double x){return int(x>1e-9)-int(x < -1e-9);};
    int f=sign(left+right),t=sign(right-left);
    const auto &curve=pwmMap[(f+1)*3+t+1];
    double pwm=std::max(std::abs(left),std::abs(right))*4095.;
    if(curve.empty()) return -1;
    if(pwm<=curve.front().first) return curve.front().second;
    for(size_t i=1;i<curve.size();++i)
      if(pwm<=curve[i].first)
      {
        double fraction=(pwm-curve[i-1].first)/(curve[i].first-curve[i-1].first);
        return curve[i-1].second+fraction*(curve[i].second-curve[i-1].second);
      }
    return curve.back().second;
  }

  void PWM(const gz::msgs::Double_V &msg)
  {
    std::lock_guard<std::mutex> guard(mutex);
    pwmMode=true;lastCommand=std::chrono::steady_clock::now();
    pwmLeft=pwmRight=0;
    if(msg.data_size()!=2 || !std::isfinite(msg.data(0)) || !std::isfinite(msg.data(1))) return;
    pwmLeft=std::clamp(msg.data(0),-1800./4095.,1800./4095.);
    pwmRight=std::clamp(msg.data(1),-1800./4095.,1800./4095.);
  }

  void Command(const gz::msgs::Twist &msg)
  {
    std::lock_guard<std::mutex> guard(mutex);
    pwmMode=false;lastCommand=std::chrono::steady_clock::now();
    if (!std::isfinite(msg.linear().x()) || !std::isfinite(msg.angular().z()))
    { linear = angular = 0; return; }
    linear = std::clamp(msg.linear().x(), -.3, .3);
    angular = std::clamp(msg.angular().z(), -1., 1.);
  }

  public: void Reset(const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &ecm) override
  {
    std::lock_guard<std::mutex> guard(mutex);
    linear = angular = pwmLeft = pwmRight = 0;
    pwmMode=false;
    integral.fill(0);filteredDuty.fill(0);
    lastPub = {};
    for (auto j : joints)
      if (j != gz::sim::kNullEntity && ecm.HasEntity(j))
      {
        ecm.SetComponentData<gz::sim::components::JointForceCmd>(j, {0.});
        // Physics resets its velocity request bookkeeping. Recreate the
        // request next PreUpdate instead of retaining a stale zero component.
        ecm.RemoveComponent<gz::sim::components::JointVelocity>(j);
      }
    joints.fill(gz::sim::kNullEntity);
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused) return;
    double v,w,pl,pr;bool direct;
    { std::lock_guard<std::mutex> guard(mutex); v=linear;w=angular;pl=pwmLeft;pr=pwmRight;direct=pwmMode;
      if(std::chrono::steady_clock::now()-lastCommand > std::chrono::milliseconds(400))
        v=w=pl=pr=0; }
    gz::msgs::Double_V output;
    for (size_t i=0; i<joints.size(); ++i)
    {
      if (joints[i] == gz::sim::kNullEntity || !ecm.HasEntity(joints[i]))
      {
        joints[i] = model.JointByName(ecm, names[i]);
        if (joints[i] == gz::sim::kNullEntity) return;
        ecm.CreateComponent(joints[i], gz::sim::components::JointVelocity());
        auto wheel=model.LinkByName(ecm,names[i].substr(0,names[i].size()-6));
        auto inertial=ecm.Component<gz::sim::components::Inertial>(wheel);
        if(inertial) wheelInertia[i]=inertial->Data().MassMatrix().Iyy();
      }
      auto velocity = ecm.Component<gz::sim::components::JointVelocity>(joints[i]);
      if(!velocity || velocity->Data().empty())
      {
        // Missing feedback is not zero measured speed. Never drive blind.
        ecm.SetComponentData<gz::sim::components::JointForceCmd>(joints[i], {0.});
        for(int k=0;k<4;++k) output.add_data(0.);
        continue;
      }
      double omega = velocity->Data()[0];
      double side = i % 2 == 0 ? -1 : 1;
      double target = std::clamp((v + side*w*track/2)/radius, -omega0, omega0);
      // A velocity servo chooses effective voltage. The DC back-EMF relation
      // then produces finite output torque, decreasing with shaft speed.
      // At zero target, terminals are shorted (dynamic braking), not PID hold.
      double error = target-omega;
      double dt = std::clamp(std::chrono::duration<double>(info.dt).count(), 0., .01);
      if (target == 0) integral[i]=0;
      else
      {
        double voltage = target/omega0 + feedback*error + integral[i];
        if (std::abs(voltage) < 1 || error*voltage < 0)
          integral[i]=std::clamp(integral[i]+integralGain*error*dt, -1., 1.);
      }
      double duty = target == 0 ? 0 :
          std::clamp(target/omega0 + feedback*error + integral[i], -1., 1.);
      if(direct)
      {
        integral[i]=0;
        double effort=EffectiveDuty(pl,pr),magnitude=std::max(std::abs(pl),std::abs(pr));
        duty=effort>=0 ? (magnitude>1e-9 ? (i%2==0 ? pl : pr)/magnitude*effort : 0) :
            std::clamp(i%2==0 ? pl*leftVoltageGain : pr*rightVoltageGain,-1.,1.);
        auto sign=[](double x){return int(x>1e-9)-int(x < -1e-9);};
        double tau=riseTau[(sign(pl+pr)+1)*3+sign(pr-pl)+1];
        // Empirical spin-up lag; zero command still brakes immediately.
        if(duty==0 || tau<=0) filteredDuty[i]=duty;
        else filteredDuty[i]+=(1-std::exp(-dt/tau))*(duty-filteredDuty[i]);
        duty=filteredDuty[i];
        target=duty*omega0;
      }
      double torque = std::clamp(stall*(duty-omega/omega0), -stall, stall);
      bool braking=direct ? std::abs(i%2==0 ? pl : pr)<1e-9 : target==0;
      if(braking && dt>0)
      {
        // Real zero PWM is the H-bridge brake state, including the held inside
        // wheels of an arc. Model gearbox/brake drag with finite resistance.
        // The inertia cap prevents an explicit damping step reversing the wheel.
        double resistance=std::min({stall,brakeDamping*std::abs(omega)+brakeResistance,
                                   wheelInertia[i]*std::abs(omega)/dt});
        torque=-std::copysign(resistance,omega);
      }
      if (!std::isfinite(torque)) torque = 0;
      ecm.SetComponentData<gz::sim::components::JointForceCmd>(joints[i], {torque});
      output.add_data(omega);
      output.add_data(target);
      output.add_data(torque);
      output.add_data(duty);
    }
    if (info.simTime-lastPub >= std::chrono::milliseconds(20))
    { telemetry.Publish(output); lastPub=info.simTime; }
  }
};
}
GZ_ADD_PLUGIN(ugv::TTMotorSystem, gz::sim::System,
    ugv::TTMotorSystem::ISystemConfigure, ugv::TTMotorSystem::ISystemPreUpdate,
    ugv::TTMotorSystem::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(ugv::TTMotorSystem, "ugv::TTMotorSystem")
