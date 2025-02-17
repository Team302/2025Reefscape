//====================================================================================================================================================
// Copyright 2025 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include <string>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"

// Team302 Includes
#include "chassis/states/PolarDrive.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"
#include "fielddata/DragonTargetFinder.h"

using frc::ChassisSpeeds;
using frc::Rotation2d;
using std::string;

PolarDrive::PolarDrive(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                                 m_robotDrive(robotDrive)
{
    auto finder = DragonTargetFinder::GetInstance();
    if (finder != nullptr)
    {
        auto info = finder->GetPose(DragonTargetFinderTarget::REEF_CENTER);
        m_reefCenter = get<1>(info.value());
    }
}

void PolarDrive::Init(ChassisMovement &chassismovement)
{
    if (m_chassis != nullptr)
    {
        frc::Pose2d currentPose = m_chassis->GetPose();
        units::length::meter_t xDiff = currentPose.X() - m_reefCenter.X();
        units::length::meter_t yDiff = currentPose.Y() - m_reefCenter.Y();

        m_radiusTarget = units::math::hypot(xDiff, yDiff);
    }
}

std::array<frc::SwerveModuleState, 4> PolarDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (m_chassis != nullptr)
    {
        auto chassisSpeeds = chassisMovement.chassisSpeeds;

        frc::Pose2d currentPose = m_chassis->GetPose();

        units::length::meter_t xDiff = currentPose.X() - m_reefCenter.X();
        units::length::meter_t yDiff = currentPose.Y() - m_reefCenter.Y();

        units::angle::degree_t angle = units::math::atan2(yDiff, xDiff);

        double radialVelocity = abs((chassisSpeeds.vx / m_chassis->GetMaxSpeed()).value()) > 0.25 ? chassisSpeeds.vx.value() : 0;
        m_radiusTarget += units::length::meter_t(radialVelocity * m_loopRate);

        double angularVelocity = chassisSpeeds.vy.value() / m_radiusTarget.value();
        angle += units::angle::degree_t(angularVelocity * m_loopRate);

        // Convert polar velocities back to Cartesian
        double vxNew = radialVelocity * units::math::cos(angle).value() - (m_radiusTarget.value() * angularVelocity * units::math::sin(angle).value());
        double vyNew = radialVelocity * units::math::sin(angle).value() + (m_radiusTarget.value() * angularVelocity * units::math::cos(angle).value());

        chassisSpeeds.vx = units::velocity::meters_per_second_t(-vxNew);
        chassisSpeeds.vy = units::velocity::meters_per_second_t(-vyNew);

        auto rot2d = Rotation2d(m_chassis->GetYaw());
        chassisMovement.chassisSpeeds = ChassisSpeeds::FromFieldRelativeSpeeds(chassisSpeeds.vx,
                                                                               chassisSpeeds.vy,
                                                                               chassisMovement.chassisSpeeds.omega,
                                                                               rot2d);
        return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("PolarDrive"), string("chassis"), string("nullptr"));
    }

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

std::string PolarDrive::GetDriveStateName() const
{
    return std::string("PolarDrive");
}
