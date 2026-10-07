#include "g1_hardware_interface/gravity_compensator.hpp"

#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace g1_hardware_interface
{

namespace
{

constexpr std::array<const char*, kNumGravityJoints> kActiveJointNames = {
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",

    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",

    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
};

}  // namespace

struct GravityCompensator::Impl
{
    pinocchio::Model model;
    pinocchio::Data data;
    Eigen::VectorXd q;
    std::array<pinocchio::JointIndex, kNumGravityJoints> joint_ids{};

    explicit Impl(pinocchio::Model reduced_model)
        : model(std::move(reduced_model)),
          data(model),
          q(pinocchio::neutral(model))
    {
        if (model.nv != static_cast<int>(kNumGravityJoints))
        {
            throw std::runtime_error(
                "GravityCompensator reduced model must have exactly 17 DoF");
        }

        for (std::size_t i = 0; i < kActiveJointNames.size(); ++i)
        {
            const auto joint_id = model.getJointId(kActiveJointNames[i]);

            if (joint_id == 0)
            {
                throw std::runtime_error(
                    std::string("GravityCompensator joint not found: ") +
                    kActiveJointNames[i]);
            }

            joint_ids[i] = joint_id;
        }
    }
};

GravityCompensator::~GravityCompensator() = default;

GravityCompensator::GravityCompensator(const std::string& urdf_path)
{
    pinocchio::Model full_model;
    pinocchio::urdf::buildModel(urdf_path, full_model);

    const Eigen::VectorXd reference_configuration = pinocchio::neutral(full_model);

    const std::unordered_set<std::string> active_names(
        kActiveJointNames.begin(), kActiveJointNames.end());

    std::vector<pinocchio::JointIndex> joints_to_lock;

    for (pinocchio::JointIndex joint_id = 1;
         joint_id < static_cast<pinocchio::JointIndex>(full_model.njoints);
         ++joint_id)
    {
        if (!active_names.contains(full_model.names[joint_id]))
        {
            joints_to_lock.push_back(joint_id);
        }
    }

    pinocchio::Model reduced_model;
    pinocchio::buildReducedModel(
        full_model,
        joints_to_lock,
        reference_configuration,
        reduced_model);

    impl_ = std::make_unique<Impl>(std::move(reduced_model));
}

std::array<double, kNumArmJoints> GravityCompensator::compute(
    const std::array<double, 3>& waist_position,
    const std::array<double, kNumArmJoints>& arm_position)
{
    for (std::size_t i = 0; i < waist_position.size(); ++i)
    {
        const auto& joint = impl_->model.joints[impl_->joint_ids[i]];
        impl_->q[joint.idx_q()] = waist_position[i];
    }

    for (std::size_t i = 0; i < arm_position.size(); ++i)
    {
        const std::size_t gravity_index = i + waist_position.size();
        const auto& joint = impl_->model.joints[impl_->joint_ids[gravity_index]];
        impl_->q[joint.idx_q()] = arm_position[i];
    }

    const Eigen::VectorXd& gravity =
        pinocchio::computeGeneralizedGravity(
            impl_->model,
            impl_->data,
            impl_->q);

    std::array<double, kNumArmJoints> arm_torque{};

    for (std::size_t i = 0; i < arm_torque.size(); ++i)
    {
        const std::size_t gravity_index = i + waist_position.size();
        const auto& joint = impl_->model.joints[impl_->joint_ids[gravity_index]];
        arm_torque[i] = gravity[joint.idx_v()];
    }

    return arm_torque;
}

std::size_t GravityCompensator::dof() const
{
    return static_cast<std::size_t>(impl_->model.nv);
}

}  // namespace g1_hardware_interface
