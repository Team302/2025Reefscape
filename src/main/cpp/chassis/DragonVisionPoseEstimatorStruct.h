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
#include <vector>

#include "frc/geometry/Pose2d.h"
#include "units/time.h"
#include "wpi/array.h"

struct DragonVisionPoseEstimatorStruct
{
public:
    DragonVisionPoseEstimatorStruct() : m_confidenceLevel(ConfidenceLevel::NONE),
                                        m_visionPose(frc::Pose2d{}),
                                        m_timeStamp(units::time::second_t(0.0)),
                                        m_stds(wpi::array(0.9, 0.9, 0.9)) {};
    ~DragonVisionPoseEstimatorStruct() = default;

    enum ConfidenceLevel
    {
        NONE,
        LOW,
        MEDIUM,
        HIGH
    };

FieldConstants::FieldConstants()
{
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_J];
    fieldConstantsPoseMap[BLUE_REEF_I] = m_BlueReef;
    fieldConstantsPoseMap[BLUE_REEF_H] = m_BlueReef;
    fieldConstantsPoseMap[BLUE_REEF_G] = m_BlueReef;
    fieldConstantsPoseMap[BLUE_REEF_F] = m_BlueReef;
    fieldConstantsPoseMap[BLUE_REEF_E] = m_BlueReef;
    fieldConstantsPoseMap[BLUE_REEF_D] = m_BlueReef;
    fieldConstantsPoseMap[BLUE_REEF_C] = m_BlueReef;
    fieldConstantsPoseMap[BLUE_REEF_B] = m_BlueReef;
    fieldConstantsPoseMap[BLUE_REEF_A] = m_BlueReef;
    fieldConstantsPoseMap[BLUE_REEF_L] = m_BlueReef;
    fieldConstantsPoseMap[BLUE_REEF_K] = m_BlueReef;
    fieldConstantsPoseMap[RED_REEF_D] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_C] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_B] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_A] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_L] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_K] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_J] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_I] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_H] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_G] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_F] = m_RedReef;
    fieldConstantsPoseMap[RED_REEF_E] = m_RedReef;
    fieldConstantsPoseMap[BLUE_LEFT_CAGE] = m_BlueCage;
    fieldConstantsPoseMap[BLUE_MIDDLE_CAGE] = m_BlueCage;
    fieldConstantsPoseMap[BLUE_RIGHT_CAGE] = m_BlueCage;
    fieldConstantsPoseMap[RED_LEFT_CAGE] = m_RedCage;
    fieldConstantsPoseMap[RED_MIDDLE_CAGE] = m_RedCage;
    fieldConstantsPoseMap[RED_RIGHT_CAGE] = m_RedCage;
    fieldConstantsPoseMap[BLUE_FAR_LEFT_CORAL_STATION] = m_BlueCoralStation;
    fieldConstantsPoseMap[BLUE_FAR_RIGHT_CORAL_STATION] = m_BlueCoralStation;
    fieldConstantsPoseMap[RED_FAR_LEFT_CORAL_STATION] = m_RedCoralStation;
    fieldConstantsPoseMap[RED_FAR_RIGHT_CORAL_STATION] = m_RedCoralStation;
    fieldConstantsPoseMap[RED_HUMAN_PROCESSOR] = m_RedHumanProcessor;
    fieldConstantsPoseMap[RED_HUMAN_PROCESSOR] = m_RedHumanProcessor;
}

frc::Pose3d FieldConstants::GetFieldElement(FIELD_ELEMENT element)
{
    frc::Pose3d Pose3d = fieldConstantsPoseMap[element];
    return Pose3d;
}
    ConfidenceLevel m_confidenceLevel;
    frc::Pose2d m_visionPose;
    units::time::second_t m_timeStamp;
    wpi::array<double, 3> m_stds;
};
