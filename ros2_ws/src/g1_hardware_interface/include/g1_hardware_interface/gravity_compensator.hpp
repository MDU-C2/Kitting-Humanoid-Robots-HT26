#ifndef G1_HARDWARE_INTERFACE__GRAVITY_COMPENSATOR_HPP_
#define G1_HARDWARE_INTERFACE__GRAVITY_COMPENSATOR_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "g1_hardware_interface/arm_ramp_engine.hpp"

namespace g1_hardware_interface
{

inline constexpr std::size_t kNumGravityJoints = kNumArmJoints + 3;

class GravityCompensator
{
public:
    explicit GravityCompensator(const std::string& urdf_path);
    ~GravityCompensator();

    std::array<double, kNumArmJoints> compute(
        const std::array<double, 3>& waist_position,
        const std::array<double, kNumArmJoints>& arm_position);

    std::size_t dof() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__GRAVITY_COMPENSATOR_HPP_
