#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <limits>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Pose.hh>

namespace payload_demo
{
using ignition::gazebo::Entity;
using ignition::gazebo::EntityComponentManager;
using ignition::gazebo::ISystemConfigure;
using ignition::gazebo::ISystemPreUpdate;
using ignition::gazebo::Model;
using ignition::gazebo::System;
using ignition::gazebo::UpdateInfo;
using ignition::gazebo::World;
namespace components = ignition::gazebo::components;

struct PoseCommand
{
  std::string model;
  ignition::math::Pose3d pose;
};

class PayloadPoseReplaySystem:
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
  public: void Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &,
    ignition::gazebo::EventManager &) override
  {
    this->worldEntity = _entity;
    if (_sdf->HasElement("trajectory"))
      this->trajectoryPath = _sdf->Get<std::string>("trajectory");
    if (_sdf->HasElement("rate"))
      this->rate = std::max(1.0, _sdf->Get<double>("rate"));
    if (_sdf->HasElement("loop"))
      this->loop = _sdf->Get<bool>("loop");

    this->LoadTrajectory();
  }

  public: void PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->frames.empty())
      return;

    const double simTime =
      std::chrono::duration<double>(_info.simTime).count();
    std::size_t index = static_cast<std::size_t>(std::max(0.0, simTime) * this->rate);
    if (this->loop)
      index %= this->frames.size();
    else
      index = std::min(index, this->frames.size() - 1);
    if (index == this->lastFrameIndex)
      return;
    this->lastFrameIndex = index;

    World world(this->worldEntity);
    for (const auto &command : this->frames[index])
    {
      Entity entity = this->ResolveEntity(world, _ecm, command.model);
      if (entity == ignition::gazebo::kNullEntity)
        continue;

      Model model(entity);
      model.SetWorldPoseCmd(_ecm, command.pose);

      auto poseComponent = _ecm.Component<components::Pose>(entity);
      if (poseComponent)
      {
        poseComponent->SetData(
          command.pose,
          [](const ignition::math::Pose3d &_a,
             const ignition::math::Pose3d &_b)
          {
            return _a == _b;
          });
      }
      else
      {
        _ecm.CreateComponent(entity, components::Pose(command.pose));
      }
    }
  }

  private: Entity ResolveEntity(
    const World &_world,
    const EntityComponentManager &_ecm,
    const std::string &_name)
  {
    auto found = this->entityCache.find(_name);
    if (found != this->entityCache.end() &&
        found->second != ignition::gazebo::kNullEntity)
      return found->second;

    Entity entity = _world.ModelByName(_ecm, _name);
    this->entityCache[_name] = entity;
    return entity;
  }

  private: static std::vector<std::string> SplitCsvLine(const std::string &_line)
  {
    std::vector<std::string> fields;
    std::stringstream stream(_line);
    std::string field;
    while (std::getline(stream, field, ','))
      fields.push_back(field);
    return fields;
  }

  private: void LoadTrajectory()
  {
    if (this->trajectoryPath.empty())
    {
      std::cerr << "[PayloadPoseReplaySystem] missing <trajectory>" << std::endl;
      return;
    }

    std::ifstream input(this->trajectoryPath);
    if (!input)
    {
      std::cerr << "[PayloadPoseReplaySystem] cannot open "
                << this->trajectoryPath << std::endl;
      return;
    }

    std::string line;
    bool header = true;
    std::size_t rowCount = 0;
    while (std::getline(input, line))
    {
      if (line.empty())
        continue;
      if (header)
      {
        header = false;
        continue;
      }

      auto fields = SplitCsvLine(line);
      if (fields.size() != 9)
        continue;

      const std::size_t frame = static_cast<std::size_t>(std::stoul(fields[0]));
      if (this->frames.size() <= frame)
        this->frames.resize(frame + 1);

      const double x = std::stod(fields[2]);
      const double y = std::stod(fields[3]);
      const double z = std::stod(fields[4]);
      const double qx = std::stod(fields[5]);
      const double qy = std::stod(fields[6]);
      const double qz = std::stod(fields[7]);
      const double qw = std::stod(fields[8]);

      PoseCommand command;
      command.model = fields[1];
      command.pose = ignition::math::Pose3d(
        ignition::math::Vector3d(x, y, z),
        ignition::math::Quaterniond(qw, qx, qy, qz));
      this->frames[frame].push_back(std::move(command));
      ++rowCount;
    }

    std::cerr << "[PayloadPoseReplaySystem] loaded " << this->frames.size()
              << " frames, " << rowCount << " pose rows from "
              << this->trajectoryPath << std::endl;
  }

  private: Entity worldEntity{ignition::gazebo::kNullEntity};
  private: std::string trajectoryPath;
  private: double rate{90.0};
  private: bool loop{true};
  private: std::vector<std::vector<PoseCommand>> frames;
  private: std::unordered_map<std::string, Entity> entityCache;
  private: std::size_t lastFrameIndex{std::numeric_limits<std::size_t>::max()};
};
}

IGNITION_ADD_PLUGIN(
  payload_demo::PayloadPoseReplaySystem,
  ignition::gazebo::System,
  payload_demo::PayloadPoseReplaySystem::ISystemConfigure,
  payload_demo::PayloadPoseReplaySystem::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  payload_demo::PayloadPoseReplaySystem,
  "payload_demo::PayloadPoseReplaySystem")
