
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

#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "fielddata/ProcessorHelper.h"
#include "frc/DriverStation.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"
#include "frc/Filesystem.h"

ProcessorHelper *ProcessorHelper::m_instance = nullptr;
ProcessorHelper *ProcessorHelper::GetInstance()
{
    if (ProcessorHelper::m_instance == nullptr)
    {
        ProcessorHelper::m_instance = new ProcessorHelper();
    }
    return ProcessorHelper::m_instance;
}

ProcessorHelper::ProcessorHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetCurrentChassis()),
                                     m_fieldConstants(FieldConstants::GetInstance())
{
}

frc::Pose2d ProcessorHelper::CalcProcessorPose()
{
    auto allianceColor = FMSData::GetInstance()->GetAllianceColor();
    frc::Pose2d pose2d{};
    if (allianceColor == frc::DriverStation::Alliance::kRed)
    {
        pose2d = m_fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_PROCESSOR).ToPose2d();
    }
    else
    {
        pose2d = m_fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_PROCESSOR).ToPose2d();
    }
    return pose2d;
}
std::optional<FieldConstants::AprilTagIDs> ProcessorHelper::GetAprilTag()
{
    int apriltagid;
    auto allianceColor = FMSData::GetInstance()->GetAllianceColor();
    if (allianceColor == frc::DriverStation::Alliance::kRed)
    {
        return FieldConstants::AprilTagIDs::RED_PROCESSOR_TAG;
    }
    else
    {
        return FieldConstants::AprilTagIDs::BLUE_PROCESSOR_TAG;
    }
}
