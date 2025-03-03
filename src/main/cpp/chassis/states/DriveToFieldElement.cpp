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
//=====================================================================================================================================================

// C++ Includes

// FRC Includes
#include <frc/geometry/Pose2d.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

// Team302 Includes
#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/states/DriveToFieldElement.h"
#include "chassis/SwerveChassis.h"
#include "vision/DragonVisionStructs.h"
#include "vision/DragonVisionStructLogger.h"
#include "fielddata/DragonTargetFinder.h"

#include "utils/logging/debug/Logger.h"
#include "utils/logging/debug/LoggerData.h"
#include "utils/logging/debug/LoggerEnums.h"

using namespace pathplanner;
using namespace std;

DriveToFieldElement::DriveToFieldElement(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                                                   m_robotDrive(robotDrive)
{
}

void DriveToFieldElement::Init(ChassisMovement &chassisMovement)
{
    InitChassisMovement(chassisMovement);
    auto info = DragonTargetFinder::GetInstance()->GetPose(GetDriveToTarget());
    m_endPose = std::nullopt;
    m_currentType = get<0>(info.value());

    if (m_chassis != nullptr)
    {
        m_translationPIDX.Reset(m_chassis->GetPose().X(), chassisMovement.chassisSpeeds.vx);
        m_translationPIDY.Reset(m_chassis->GetPose().Y(), chassisMovement.chassisSpeeds.vy);
    }
}

std::array<frc::SwerveModuleState, 4> DriveToFieldElement::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (m_chassis == nullptr)
    {
        auto chassisSpeeds = chassisMovement.chassisSpeeds;
        frc::Pose2d currentPose = m_chassis->GetPose();

        auto info = DragonTargetFinder::GetInstance()->GetPose(GetDriveToTarget());
        frc::Pose2d newEndPose = get<1>(info.value());
        auto regenerate = false;

        if (m_endPose.has_value())
        {
            regenerate = m_endPose.value().Translation().Distance(newEndPose.Translation()) > m_distanceThreshold;
        }

        if (info && (m_currentType == DragonTargetFinderData::ODOMETRY_BASED) && (get<0>(info.value()) == DragonTargetFinderData::VISION_BASED) && regenerate) // If we are in odometry but get vision based pose regenerate
        {
            m_endPose = newEndPose;
        }
        m_currentType = get<0>(info.value());

        m_translationPIDX.SetGoal(m_endPose.value().X());
        m_translationPIDY.SetGoal(m_endPose.value().Y());

        chassisSpeeds.vx = units::velocity::meters_per_second_t(m_translationPIDX.Calculate(currentPose.X(), m_endPose.value().X()));
        chassisSpeeds.vy = units::velocity::meters_per_second_t(m_translationPIDY.Calculate(currentPose.Y(), m_endPose.value().Y()));

        auto profiledVx = m_translationPIDX.GetSetpoint().velocity;
        auto profiledVy = m_translationPIDY.GetSetpoint().velocity;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Profiled Vx", profiledVx.value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Profiled Vy", profiledVy.value());

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "PID Calculated Vx", chassisSpeeds.vx.value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "PID Calculated Vy", chassisSpeeds.vy.value());

        auto rot2d = frc::Rotation2d(m_chassis->GetYaw());
        chassisMovement.chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassisSpeeds.vx,
                                                                                    chassisSpeeds.vy,
                                                                                    chassisMovement.chassisSpeeds.omega,
                                                                                    rot2d);
    }
    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void DriveToFieldElement::InitChassisMovement(ChassisMovement &chassisMovement)
{
    // initialize the same as holonomic drive
    chassisMovement.rawOmega = 0.0;
    chassisMovement.driveOption = GetDriveStateType();
    chassisMovement.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    chassisMovement.headingOption = ChassisOptionEnums::IGNORE;
    chassisMovement.pathplannerTrajectory = pathplanner::PathPlannerTrajectory();
    chassisMovement.centerOfRotationOffset = frc::Translation2d();
    chassisMovement.noMovementOption = ChassisOptionEnums::NoMovementOption::STOP;
    chassisMovement.pathnamegains = ChassisOptionEnums::PathGainsType::SHORT;
    auto chassis = ChassisConfigMgr::GetInstance()->GetCurrentConfig()->GetSwerveChassis();
    if (chassis != nullptr)
    {
        chassisMovement.yawAngle = chassis->GetYaw();
    }
    chassisMovement.checkTipping = false;
    chassisMovement.tippingTolerance = units::angle::degree_t(5.0);
    chassisMovement.tippingCorrection = -0.1;
    chassisMovement.targetPose = frc::Pose2d();
}

bool DriveToFieldElement::IsDone()
{
    if (m_endPose.has_value())
    {
        if (m_chassis != nullptr)
        {
            auto currentPose = m_chassis->GetPose();
            auto distance = currentPose.Translation().Distance(m_endPose.value().Translation());
            return (distance < m_distanceThreshold);
        }
    }
    return false;
}