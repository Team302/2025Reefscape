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
#include <string>
#include <tuple>

// FRC Includes
#include <frc/geometry/Rotation2d.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include "vision/DragonVision.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/states/DriveToRightReefBranch.h"
#include "utils/FMSData.h"
#include "vision/DragonVisionStructs.h"
#include "vision/DragonVisionStructLogger.h"
#include "fielddata/DragonTargetFinder.h"
#include "chassis/SwerveChassis.h"

#include "utils/logging/debug/Logger.h"
#include "utils/logging/debug/LoggerData.h"
#include "utils/logging/debug/LoggerEnums.h"

using namespace pathplanner;
using namespace std;

DriveToRightReefBranch::DriveToRightReefBranch(RobotDrive *robotDrive, TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner)
    : TrajectoryDrivePathPlanner(robotDrive)
{
}

void DriveToRightReefBranch::Init(ChassisMovement &chassisMovement)
{

    m_trajectory = CreateDriveToRightReefBranch();
    InitFromTrajectory(chassisMovement, m_trajectory);
}

std::string DriveToRightReefBranch::GetDriveStateName() const
{
    return std::string("DriveToRightReefBranch");
}

void DriveToRightReefBranch::InitFromTrajectory(ChassisMovement &chassisMovement, pathplanner::PathPlannerTrajectory trajectory)
{
    m_trajectory = trajectory;
    if (!m_trajectory.getStates().empty())
    {
        chassisMovement.pathplannerTrajectory = m_trajectory;
        chassisMovement.pathnamegains = ChassisOptionEnums::PathGainsType::LONG;
        TrajectoryDrivePathPlanner::Init(chassisMovement);
    }
}

pathplanner::PathPlannerTrajectory DriveToRightReefBranch::CreateDriveToRightReefBranch()
{
    pathplanner::PathPlannerTrajectory trajectory;

    if (m_chassis != nullptr)
    {
        std::optional<std::tuple<DragonTargetFinderData, frc::Pose2d>> info = DragonTargetFinder::GetInstance()->GetPose(DragonTargetFinderTarget::CLOSEST_RIGHT_REEF_BRANCH);
        if (info && !IsDone())
        {
            m_endPose = std::get<frc::Pose2d>(info.value());
            trajectory = CreateDriveToRightReefBranchTrajectory(m_chassis->GetPose(), m_endPose);
        }
    }
    return trajectory;
}

pathplanner::PathPlannerTrajectory DriveToRightReefBranch::CreateDriveToRightReefBranchTrajectory(frc::Pose2d currentPose2d, frc::Pose2d targetPose)
{
    DragonVisionStructLogger::logPose2d("current pose", currentPose2d);
    DragonVisionStructLogger::logPose2d("coral pose", targetPose);

    pathplanner::PathConstraints constraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel);
    std::vector<frc::Pose2d> poses{currentPose2d, targetPose};
    std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);
    shared_ptr<PathPlannerPath> path;

    path = std::make_shared<PathPlannerPath>(
        waypoints,
        constraints,
        std::nullopt,
        GoalEndState(0.0_mps, frc::Rotation2d(m_chassis->GetStoredHeading())), false);

    path->preventFlipping = true;

    return path.get()->generateTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation(), m_chassis->GetRobotConfig());
}

bool DriveToRightReefBranch::IsDone()
{
    if (m_chassis != nullptr)
    {
        frc::Pose2d currentPose(m_chassis->GetPose());

        if (m_endPose.Translation().Distance(m_chassis->GetPose().Translation()) < units::inch_t(6))
            return true;
    }
    return false;
}
