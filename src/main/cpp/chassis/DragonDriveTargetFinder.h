
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

#include "frc/geometry/Pose2d.h"
#include "units/angle.h"

#include "chassis/ChassisMovement.h"
#include "chassis/SwerveChassis.h"
#include "vision/DragonVision.h"

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

    enum AprilTagIDs
    {
        BLUE_REEF_J = 20,
        BLUE_REEF_I = 20,
        BLUE_REEF_H = 21,
        BLUE_REEF_G = 21,
        BLUE_REEF_F = 22,
        BLUE_REEF_E = 22,
        BLUE_REEF_D = 17,
        BLUE_REEF_C = 17,
        BLUE_REEF_B = 18,
        BLUE_REEF_A = 18,
        BLUE_REEF_L = 19,
        BLUE_REEF_K = 19,
        RED_REEF_D = 8,
        RED_REEF_C = 8,
        RED_REEF_B = 7,
        RED_REEF_A = 7,
        RED_REEF_L = 6,
        RED_REEF_K = 6,
        RED_REEF_J = 11,
        RED_REEF_I = 11,
        RED_REEF_H = 10,
        RED_REEF_G = 10,
        RED_REEF_F = 9,
        RED_REEF_E = 9,
        BLUE_LEFT_CAGE = 15,
        BLUE_LEFT_CAGE = 5,
        BLUE_MIDDLE_CAGE = 15,
        BLUE_MIDDLE_CAGE = 5,
        BLUE_RIGHT_CAGE = 15,
        BLUE_RIGHT_CAGE = 5,
        RED_LEFT_CAGE = 14,
        RED_LEFT_CAGE = 4,
        RED_MIDDLE_CAGE = 14,
        RED_MIDDLE_CAGE = 4,
        RED_RIGHT_CAGE = 14,
        RED_MIDDLE_CAGE = 4,
        BLUE_FAR_LEFT_HUMAN_PLAYER_ = 13,
        BLUE_FAR_RIGHT_HUMAN_PLAYER = 12,
        RED_FAR_RIGHT_HUMAN_PLAYER = 2,
        RED_FAR_LEFT_HUMAN_PLAYER = 1,
        BLUE_HUMAN_PROCESSOR = 16,
        RED_HUMAN_PROCESSOR = 13

    };

    const std::map<DragonVision::VISION_ELEMENT, AprilTagIDs> blueMap = {
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_REEF_J},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_REEF_I},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_REEF_H},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_REEF_G},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_REEF_F},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_REEF_E},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_REEF_D},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_REEF_A},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_REEF_L},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::BLUE_REEF_K},
        {DragonVision::VISION_ELEMENT::CAGE, AprilTagIDs::BLUE_LEFT_CAGE},
        {DragonVision::VISION_ELEMENT::CAGE, AprilTagIDs::BLUE_MIDDLE_CAGE},
        {DragonVision::VISION_ELEMENT::CAGE, AprilTagIDs::BLUE_RIGHT_CAGE},
        {DragonVision::VISION_ELEMENT::HUMAN_PLAYER, AprilTagIDs::BLUE_FAR_LEFT_HUMAN_PLAYER_},
        {DragonVision::VISION_ELEMENT::HUMAN_PLAYER, AprilTagIDs::BLUE_FAR_RIGHT_HUMAN_PLAYER},
        {DragonVision::VISION_ELEMENT::HUMAN_PROCESSOR, AprilTagIDs::BLUE_HUMAN_PROCESSOR}};

    const std::map<DragonVision::VISION_ELEMENT, AprilTagIDs> redMap = {
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_D},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_C},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_B},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_A},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_L},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_K},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_J},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_I},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_H},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_G},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_F},
        {DragonVision::VISION_ELEMENT::REEF, AprilTagIDs::RED_REEF_E},
        {DragonVision::VISION_ELEMENT::CAGE, AprilTagIDs::RED_LEFT_CAGE},
        {DragonVision::VISION_ELEMENT::CAGE, AprilTagIDs::RED_MIDDLE_CAGE},
        {DragonVision::VISION_ELEMENT::CAGE, AprilTagIDs::RED_RIGHT_CAGE},
        {DragonVision::VISION_ELEMENT::HUMAN_PLAYER, AprilTagIDs::RED_FAR_LEFT_HUMAN_PLAYER},
        {DragonVision::VISION_ELEMENT::HUMAN_PLAYER, AprilTagIDs::RED_FAR_RIGHT_HUMAN_PLAYER},
        {DragonVision::VISION_ELEMENT::HUMAN_PROCESSOR, AprilTagIDs::RED_HUMAN_PROCESSOR}};
    const units::length::meter_t m_fuseTol = units::length::meter_t(0.25);
};
