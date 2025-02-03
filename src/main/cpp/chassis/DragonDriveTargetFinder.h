
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

#include <map>
#include <tuple>

#include "chassis/ChassisMovement.h"
#include "chassis/SwerveChassis.h"
#include "frc/geometry/Pose2d.h"
#include "units/angle.h"
#include "vision/DragonVision.h"
#include "utils/FieldConstants.h"

class DragonDriveTargetFinder
{
public:
    // vision = 1
    // odometry = 10
    enum TARGET_INFO
    {
        NOT_FOUND,
        VISION_BASED = 1,
        ODOMETRY_BASED = 10,
        VISION_ODOMETRY_FUSED = 11
    };

    enum FINDER_OPTION
    {
        VISION_ONLY,
        ODOMETRY_ONLY,
        FUSE_IF_POSSIBLE
    };

    static DragonDriveTargetFinder *GetInstance();

    std::tuple<TARGET_INFO, frc::Pose2d> GetPose(DragonVision::VISION_ELEMENT item);
    std::tuple<TARGET_INFO, units::length::meter_t> GetDistance(FINDER_OPTION option, DragonVision::VISION_ELEMENT item);

    static void SetCorrection(ChassisMovement &chassisMovement,
                              SwerveChassis *chassis,
                              units::angle::degree_t target,
                              double kp);

private:
    DragonDriveTargetFinder() = default;
    ~DragonDriveTargetFinder() = default;
    static DragonDriveTargetFinder *m_instance;

    SwerveChassis *GetChassis();
    int GetAprilTag(DragonVision::VISION_ELEMENT item);
    frc::Pose2d GetAprilTagPose(DragonVision::VISION_ELEMENT item);
    units::angle::degree_t AdjustRobotRelativeAngleForIntake(units::angle::degree_t angle);

    const std::map<DragonVision::VISION_ELEMENT, FieldConstants::AprilTagIDs> blueMap = {
        {DragonVision::VISION_ELEMENT::REEF, FieldConstants::AprilTagIDs::FI_BLUE_SPEAKER},
        {DragonVision::VISION_ELEMENT::PROCESSOR, FieldConstants::AprilTagIDs::FI_BLUE_AMP},
        {DragonVision::VISION_ELEMENT::CAGE, FieldConstants::AprilTagIDs::FI_BLUE_STAGE_CENTER},
        {DragonVision::VISION_ELEMENT::CAGE, FieldConstants::AprilTagIDs::FI_BLUE_STAGE_LEFT},
        {DragonVision::VISION_ELEMENT::CAGE, FieldConstants::AprilTagIDs::FI_BLUE_STAGE_RIGHT}};

    const std::map<DragonVision::VISION_ELEMENT, AprilTagIDs> blueMap = {
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_SPEAKER},
        {DragonVision::VISION_ELEMENT::CORAL_STATION, AprilTagIDs::BLUE_SPEAKER},
        {DragonVision::VISION_ELEMENT::PROCESSOR, AprilTagIDs::BLUE_AMP},
        {DragonVision::VISION_ELEMENT::BARGE, AprilTagIDs::BLUE_STAGE_CENTER}};

    const std::map<DragonVision::VISION_ELEMENT, AprilTagIDs> redMap = {
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_SPEAKER},
        {DragonVision::VISION_ELEMENT::CORAL_STATION, AprilTagIDs::RED_SPEAKER},
        {DragonVision::VISION_ELEMENT::PROCESSOR, AprilTagIDs::RED_AMP},
        {DragonVision::VISION_ELEMENT::BARGE, AprilTagIDs::RED_STAGE_CENTER}};

    const units::length::meter_t m_fuseTol = units::length::meter_t(0.25);
};
