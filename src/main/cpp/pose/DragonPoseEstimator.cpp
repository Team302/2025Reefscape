
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

#include "DragonPoseEstimator.h"

DragonPoseEstimator *DragonPoseEstimator::m_instance = nullptr;

DragonPoseEstimator *DragonPoseEstimator::GetInstance()
{
    if (DragonPoseEstimator::m_instance == nullptr)
    {
        DragonPoseEstimator::m_instance = new DragonPoseEstimator();
    }
    return DragonPoseEstimator::m_instance;
}

DragonPoseEstimator::DragonPoseEstimator() : m_vision(DragonVision::GetDragonVision()),
                                             m_chassis()
{
    auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = chassisConfig != nullptr ? chassisConfig->GetSwerveChassis() : nullptr;
}

frc::Pose3d DragonPoseEstimator::GetEstimatedPosition()
{
    return frc::Pose3d();
}

frc::Pose3d DragonPoseEstimator::GetVisonPose()
{
    return frc::Pose3d();
}

frc::Pose2d DragonPoseEstimator::GetChassisPose()
{
    return m_chassis->GetPose();
}

frc::Pose3d DragonPoseEstimator::GetQuestPose()
{
    return frc::Pose3d();
}
