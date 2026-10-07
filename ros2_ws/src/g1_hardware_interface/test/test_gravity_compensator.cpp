#include <array>
#include <cmath>
#include <string>

#include <gmock/gmock.h>

#include "g1_hardware_interface/gravity_compensator.hpp"

namespace
{

constexpr double kTolerance = 1.0e-3;

TEST(GravityCompensator, MatchesReferenceGravityTorquesAtZeroPose)
{
    const std::string urdf_path =
        "/workspace/src/g1_description/urdf/"
        "g1_29dof_with_inspire_hand_ftp.urdf";

    g1_hardware_interface::GravityCompensator compensator(urdf_path);

    EXPECT_EQ(compensator.dof(), 17U);

    const std::array<double, 3> waist = {0.0, 0.0, 0.0};
    const std::array<double, g1_hardware_interface::kNumArmJoints> arms{};

    const auto tau = compensator.compute(waist, arms);

    EXPECT_NEAR(tau[0], -4.4795, kTolerance);  // left shoulder pitch
    EXPECT_NEAR(tau[3], -4.2541, kTolerance);  // left elbow
    EXPECT_NEAR(tau[5], -1.8148, kTolerance);  // left wrist pitch

    EXPECT_NEAR(tau[7], -4.4178, kTolerance);   // right shoulder pitch
    EXPECT_NEAR(tau[10], -4.1899, kTolerance); // right elbow
    EXPECT_NEAR(tau[12], -1.7506, kTolerance); // right wrist pitch
}

TEST(GravityCompensator, MatchesReferenceAtLeftShoulderMinus113Degrees)
{
    const std::string urdf_path =
        "/workspace/src/g1_description/urdf/"
        "g1_29dof_with_inspire_hand_ftp.urdf";

    g1_hardware_interface::GravityCompensator compensator(urdf_path);

    const std::array<double, 3> waist = {0.0, 0.0, 0.0};

    std::array<double, g1_hardware_interface::kNumArmJoints> arms{};
    arms[0] = -1.9738;

    const auto tau = compensator.compute(waist, arms);

    EXPECT_NEAR(tau[0], -3.6173, kTolerance);  // left shoulder pitch
    EXPECT_NEAR(tau[1], +2.0228, kTolerance);  // left shoulder roll
    EXPECT_NEAR(tau[2], +1.7004, kTolerance);  // left shoulder yaw
    EXPECT_NEAR(tau[3], +1.0693, kTolerance);  // left elbow
    EXPECT_NEAR(tau[5], +0.5561, kTolerance);  // left wrist pitch
    EXPECT_NEAR(tau[6], +0.4747, kTolerance);  // left wrist yaw
}

}  // namespace
