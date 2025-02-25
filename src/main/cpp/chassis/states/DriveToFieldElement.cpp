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

DriveToFieldElement::DriveToFieldElement(
    RobotDrive *robotDrive,
    TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner) : TrajectoryDrivePathPlanner(robotDrive)
{
}

void DriveToFieldElement::Init(ChassisMovement &chassisMovement)
{
    InitChassisMovement(chassisMovement);
    auto info = DragonTargetFinder::GetInstance()->GetPose(GetDriveToTarget());
    m_endPose = std::nullopt;

    // if (!IsDone()) //TODO: don't generate if you are within a certain distance to the pose
    // {
    m_trajectory = CreateTrajectory(info);
    InitFromTrajectory(chassisMovement, m_trajectory);
    m_currentType = get<0>(info.value());
    // }
}

void DriveToFieldElement::InitFromTrajectory(ChassisMovement &chassisMovement, pathplanner::PathPlannerTrajectory trajectory)
{
    m_trajectory = trajectory;
    if (!m_trajectory.getStates().empty())
    {
        chassisMovement.pathplannerTrajectory = m_trajectory;
        chassisMovement.pathnamegains = ChassisOptionEnums::PathGainsType::SHORT;
        TrajectoryDrivePathPlanner::Init(chassisMovement);
    }
}

pathplanner::PathPlannerTrajectory DriveToFieldElement::CreateTrajectory(std::optional<std::tuple<DragonTargetFinderData, frc::Pose2d>> info)
{

    pathplanner::PathPlannerTrajectory trajectory;

    if (m_chassis != nullptr)
    {
        if (info.has_value())
        {
            m_endPose = std::get<frc::Pose2d>(info.value());
            trajectory = CreateDriveToFieldElementTrajectory(m_chassis->GetPose(), m_endPose.value()); // No need to check has_value since we just set it on the previous line
        }
    }
    return trajectory;
}

pathplanner::PathPlannerTrajectory DriveToFieldElement::CreateDriveToFieldElementTrajectory(frc::Pose2d currentPose2d, frc::Pose2d targetPose)
{
    PathPlannerTrajectory trajectory;

    auto endheading = GetModifiedHeadingValue(targetPose.Rotation().Degrees());
    frc::Pose2d endPose = frc::Pose2d(targetPose.Translation(), endheading);

    DragonVisionStructLogger::logPose2d("current pose", currentPose2d);
    DragonVisionStructLogger::logPose2d("target pose", endPose);

    pathplanner::PathConstraints constraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel);
    std::vector<frc::Pose2d> poses{currentPose2d, endPose};
    std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);
    shared_ptr<PathPlannerPath> path;

    path = std::make_shared<PathPlannerPath>(
        waypoints,
        constraints,
        std::nullopt,
        GoalEndState(0.0_mps, endPose.Rotation()), false);

    path->preventFlipping = true;

    trajectory = path.get()->generateTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation(), m_chassis->GetRobotConfig());
    return trajectory;
}

std::array<frc::SwerveModuleState, 4> DriveToFieldElement::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    auto info = DragonTargetFinder::GetInstance()->GetPose(GetDriveToTarget());
    frc::Pose2d newEndPose = get<1>(info.value());
    auto regenerate = false;
    if (m_endPose.has_value())
    {
        regenerate = m_endPose.value().Translation().Distance(newEndPose.Translation()) > m_distanceThreshold;
    }

    if (info && (m_currentType == DragonTargetFinderData::ODOMETRY_BASED) && (get<0>(info.value()) == DragonTargetFinderData::VISION_BASED) && regenerate) // If we are in odometry but get vision based pose regenerate
    {
        m_trajectory = CreateTrajectory(info);
        InitFromTrajectory(chassisMovement, m_trajectory);
    }
    m_currentType = get<0>(info.value());

    return TrajectoryDrivePathPlanner::UpdateSwerveModuleStates(chassisMovement);
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
