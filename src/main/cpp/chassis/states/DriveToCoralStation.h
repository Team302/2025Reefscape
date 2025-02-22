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

#pragma once

// C++ Includes
#include <vector>

// FRC Includes
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include "chassis/states/RobotDrive.h"
#include "vision/DragonVision.h"
#include "fielddata/DragonTargetFinder.h"
#include "pathplanner/lib/trajectory/PathPlannerTrajectory.h"
#include "chassis/states/TrajectoryDrivePathPlanner.h"
#include "utils/FMSData.h"
#include "chassis/SwerveChassis.h"

class DriveToCoralStation : public TrajectoryDrivePathPlanner
{
public:
    DriveToCoralStation(RobotDrive *robotDrive, TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner);
    std::string GetDriveStateName() const override;

    pathplanner::PathPlannerTrajectory CreateTrajectory();

    void Init(ChassisMovement &chassisMovement) override;
    void InitFromTrajectory(ChassisMovement &chassisMovement, pathplanner::PathPlannerTrajectory trajectory) override;
    pathplanner::PathPlannerTrajectory GetTrajectory() const { return m_trajectory; }

    bool IsDone() override;

private:
    pathplanner::PathPlannerTrajectory CreateDriveToCoralStationTrajectory(frc::Pose2d currentPose, frc::Pose2d csaPose);

    pathplanner::PathPlannerTrajectory m_trajectory;
    frc::Pose2d m_endPose;
};