
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

// thrid party includes

#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Pose2d.h"

// 302 includes

#include "vision/DragonVision.h"
#include "chassis/SwerveChassis.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "vision/DragonQuest.h"

class DragonPoseEstimator
{
public:
    DragonPoseEstimator *GetInstance();
    frc::Pose3d GetEstimatedPosition();

private:
    DragonPoseEstimator() = default;
    ~DragonPoseEstimator() = default;

    frc::Pose3d GetVisonPose();
    frc::Pose2d GetChassisPose();
    frc::Pose3d GetQuestPose();

    static DragonPoseEstimator *m_instance;
    DragonVision *m_vision;
    SwerveChassis *m_chassis;
    DragonQuest *m_quest;

    frc::Pose3d m_chassisPose;
    frc::Pose3d m_visionPose;
    frc::Pose3d m_QuestPose;
    frc::Pose3d m_estimatedPose;

    double m_visionWeight = .4;
    double m_chassisWeight = .1;
    double m_questWeight = .5;
};