#include <functional>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Plugin.hh"
#include <gazebo/common/common.hh>
#include "walker/ActorTurn.hh"

using namespace gazebo;

#define WALKING_ANIMATION "walking"
GZ_REGISTER_MODEL_PLUGIN(ActorTurn)

ActorTurn::ActorTurn()
{
}

void ActorTurn::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    this->sdf = _sdf;
    this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

    this->lastUpdate = 0;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ActorTurn::OnUpdate, this, std::placeholders::_1));

    this->Reset();

    if (_sdf->HasElement("turn_radius"))
        this->radius = _sdf->Get<double>("turn_radius");
    else
        this->radius = 2;

    if (_sdf->HasElement("animation_factor"))
        this->animationFactor = _sdf->Get<double>("animation_factor");
    else
        this->animationFactor = 4.1;

    if (_sdf->HasElement("linear_velocity"))
        this->linearVelocity = _sdf->Get<double>("linear_velocity");
    else
        this->linearVelocity = 1.13;
    
}

void ActorTurn::Reset()
{
    this->lastUpdate = 0;
    this->rotationCenter = ignition::math::Vector3d(0, 0, 1.05);

    auto skelAnims = this->actor->SkeletonAnimations();
    if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
    {
        gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
    }
    else
    {
        this->trajectoryInfo.reset(new physics::TrajectoryInfo());
        this->trajectoryInfo->type = WALKING_ANIMATION;
        this->trajectoryInfo->duration = 1.0;
        this->actor->SetCustomTrajectory(this->trajectoryInfo);
    }            

}

void ActorTurn::OnUpdate(const common::UpdateInfo &_info)
{
    double dt = (_info.simTime - this->lastUpdate).Double();

    ignition::math::Pose3d pose = this->actor->WorldPose();

    double linearDistance = this->linearVelocity * dt;

    // Radial direction with respect to rotation center
    double theta1 = atan2((pose.Pos()-this->rotationCenter).Y(),
                        (pose.Pos()-this->rotationCenter).X());

    // Angle step in this iteration
    double theta2 = linearDistance / this->radius;

    pose.Pos().X(radius * cos(theta1+theta2));
    pose.Pos().Y(radius * sin(theta1+theta2));
    
    ignition::math::Angle yaw = theta1+theta2 + M_PI;
    yaw.Normalize();

    pose.Rot() = ignition::math::Quaterniond(M_PI/2, 0, yaw.Radian());

    pose.Pos().Z(1.05);

    // Distance traveled is used to coordinate motion with the walking
    // animation
    double distanceTraveled = (pose.Pos() -
        this->actor->WorldPose().Pos()).Length();

    this->actor->SetWorldPose(pose, false, false);
    this->actor->SetScriptTime(this->actor->ScriptTime() +
        (distanceTraveled * this->animationFactor));
    this->lastUpdate = _info.simTime;
}