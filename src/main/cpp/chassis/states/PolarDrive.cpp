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
}

std::array<frc::SwerveModuleState, 4> PolarDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{

    /*
    frc::Pose3d reefCenter = FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kRed
                                 ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::RED_REEF_CENTER)}
                                 : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::BLUE_REEF_CENTER)};

    frc::Pose2d currentPose = m_chassis->GetPose();
    units::length::meter_t xDiff = currentPose.X() - reefCenter.X();  //Still useful for logging
    units::length::meter_t yDiff = currentPose.Y() - reefCenter.Y(); //Still useful for logging
    units::length::meter_t radius = units::math::hypot(xDiff, yDiff);
    units::angle::degree_t angle = units::math::atan2(yDiff, xDiff);

    auto chassisSpeeds = chassisMovement.chassisSpeeds;

    double radialInput = chassisSpeeds.vx.value();
    double radialVelocity = radialInput; // Scale as needed

    double angularInput = chassisSpeeds.vy.value();
    double angularVelocity = (radius.value() > 0.01) ? angularInput / radius.value() : 0.0;
    angle += units::angle::degree_t(angularVelocity * m_loopRate);

    // Calculate the *resulting* desired velocities in field coordinates
    chassisSpeeds.vx = units::velocity::meters_per_second_t(radialVelocity * units::math::cos(angle).value() - radius.value() * angularVelocity * units::math::sin(angle).value());
    chassisSpeeds.vy = units::velocity::meters_per_second_t(radialVelocity * units::math::sin(angle).value() + radius.value() * angularVelocity * units::math::cos(angle).value());


    auto rot2d = Rotation2d(m_chassis->GetYaw());
    chassisMovement.chassisSpeeds = ChassisSpeeds::FromFieldRelativeSpeeds(chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega, rot2d);
*/
    if (m_chassis != nullptr)
    {
        frc::Pose3d reefCenter = frc::Pose3d{};
        frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();
        reefCenter = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::RED_REEF_CENTER)} /*load red reef*/ : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::BLUE_REEF_CENTER)};
        auto chassisSpeeds = chassisMovement.chassisSpeeds;

        frc::Pose2d currentPose = m_chassis->GetPose();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Y", currentPose.Y().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "X", currentPose.X().value());
        units::length::meter_t xDiff = currentPose.X() - reefCenter.X();
        units::length::meter_t yDiff = currentPose.Y() - reefCenter.Y();

        units::length::meter_t radius = units::math::hypot(xDiff, yDiff);
        units::angle::degree_t angle = units::math::atan2(yDiff, xDiff);

        // Radial velocity: Changes radius
        double radialVelocity = chassisSpeeds.vx.value(); // Forward/backward motion directly affects radius
        if (radialVelocity > 0.1)
        {
            radius += units::length::meter_t(radialVelocity * m_loopRate * -1.0); // Negative since forward decreases radius
        }

        // Angular velocity: Changes angle
        double angularVelocity = chassisSpeeds.vy.value() / radius.value(); // Clockwise/counter-clockwise motion
        angle += units::angle::degree_t(angularVelocity * m_loopRate);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "radialVelocity", radialVelocity);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "angularVelocity", angularVelocity);

        // Convert polar velocities back to Cartesian
        double vxNew = radialVelocity * units::math::cos(angle).value() - (radius.value() * angularVelocity * units::math::sin(angle).value());
        double vyNew = radialVelocity * units::math::sin(angle).value() + (radius.value() * angularVelocity * units::math::cos(angle).value());

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Raidus", radius.value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Angle", angle.value());

        chassisSpeeds.vx = units::velocity::meters_per_second_t(vxNew);
        chassisSpeeds.vy = units::velocity::meters_per_second_t(vyNew);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "VxNew", vxNew);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "VyNew", vyNew);
        auto rot2d = Rotation2d(m_chassis->GetYaw());
        chassisMovement.chassisSpeeds = ChassisSpeeds::FromFieldRelativeSpeeds(chassisMovement.chassisSpeeds.vx,
                                                                               chassisMovement.chassisSpeeds.vy,
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

void PolarDrive::Init(ChassisMovement &chassisMovement)
{
}
