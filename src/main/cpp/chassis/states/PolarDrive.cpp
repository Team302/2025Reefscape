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

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"

// Team302 Includes
#include "chassis/states/PolarDrive.h"
#include "utils/FMSData.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

using frc::ChassisSpeeds;
using frc::Rotation2d;
using std::string;

PolarDrive::PolarDrive(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                                 m_robotDrive(robotDrive)
{
    frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();
    m_reefCenter = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_REEF_CENTER)} /*load red reef*/ : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_CENTER)};
}

void PolarDrive::Init(ChassisMovement &chassismovement)
{
    if (m_chassis != nullptr)
    {
        frc::Pose2d currentPose = m_chassis->GetPose();
        units::length::meter_t xDiff = currentPose.X() - m_reefCenter.X();
        units::length::meter_t yDiff = currentPose.Y() - m_reefCenter.Y();

        units::length::meter_t m_radius = units::math::hypot(xDiff, yDiff);
    }
}

std::array<frc::SwerveModuleState, 4> PolarDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (m_chassis != nullptr)
    {
        auto chassisSpeeds = chassisMovement.chassisSpeeds;

        frc::Pose2d currentPose = m_chassis->GetPose();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Y", currentPose.Y().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "X", currentPose.X().value());
        units::length::meter_t xDiff = currentPose.X() - m_reefCenter.X();
        units::length::meter_t yDiff = currentPose.Y() - m_reefCenter.Y();

        units::angle::degree_t angle = units::math::atan2(yDiff, xDiff);

        // Radial velocity: Changes radius
        double radialVelocity = chassisSpeeds.vx.value(); // Forward/backward motion directly affects radius
        if (radialVelocity > 0.25)
        {
            m_radius += units::length::meter_t(radialVelocity * m_loopRate); // Negative since forward decreases radius
        }

        // Angular velocity: Changes angle
        double angularVelocity = chassisSpeeds.vy.value() / m_radius.value(); // Clockwise/counter-clockwise motion
        angle += units::angle::degree_t(angularVelocity * m_loopRate);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "radialVelocity", radialVelocity);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "angularVelocity", angularVelocity);

        // Convert polar velocities back to Cartesian
        double vxNew = radialVelocity * units::math::cos(angle).value() - (m_radius.value() * angularVelocity * units::math::sin(angle).value());
        double vyNew = radialVelocity * units::math::sin(angle).value() + (m_radius.value() * angularVelocity * units::math::cos(angle).value());

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Raidus", m_radius.value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Angle", angle.value());

        chassisSpeeds.vx = units::velocity::meters_per_second_t(-vxNew);
        chassisSpeeds.vy = units::velocity::meters_per_second_t(-vyNew);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "VxNew", vxNew);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "VyNew", vyNew);
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
