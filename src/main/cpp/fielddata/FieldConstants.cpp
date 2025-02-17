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
#include "FieldConstants.h"

#include "FieldElementCalculator.h"


FieldConstants *FieldConstants::m_instance = nullptr;
FieldConstants *FieldConstants::GetInstance()
{
    if (FieldConstants::m_instance == nullptr)
    {
        FieldConstants::m_instance = new FieldConstants();
    }
    return FieldConstants::m_instance;
}

FieldConstants::FieldConstants()
{
    ReadFieldCalibrationData();

    // Blue AprilTag locations
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT] = m_aprilTag13;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT] = m_aprilTag12;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_PROCESSOR] = m_aprilTag16;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_BARGE_FRONT] = m_aprilTag14;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_BARGE_BACK] = m_aprilTag4;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_AB] = m_aprilTag18;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_CD] = m_aprilTag17;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_EF] = m_aprilTag22;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_GH] = m_aprilTag21;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_IJ] = m_aprilTag20;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_KL] = m_aprilTag19;

    // Blue Calculated Positions
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_ALLIANCE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_SIDEWALL] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_ALLIANCE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_SIDEWALL] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_RIGHT_CAGE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_LEFT_CAGE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_CENTER] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_A] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_B] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_C] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_D] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_E] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_F] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_G] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_H] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_I] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_J] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_K] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_L] = m_placeholder;

    // Red AprilTag locations
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_LEFT] = m_aprilTag1;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_RIGHT] = m_aprilTag2;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_PROCESSOR] = m_aprilTag3;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_BARGE_FRONT] = m_aprilTag5;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_BARGE_BACK] = m_aprilTag15;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_AB] = m_aprilTag7;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_CD] = m_aprilTag8;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_EF] = m_aprilTag9;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_GH] = m_aprilTag10;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_IJ] = m_aprilTag11;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_KL] = m_aprilTag6;

    // Red Calculated Positions
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_LEFT_ALLIANCE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_LEFT_SIDEWALL] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_ALLIANCE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_SIDEWALL] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_RIGHT_CAGE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_LEFT_CAGE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_CENTER] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_A] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_B] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_C] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_D] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_E] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_F] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_G] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_H] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_I] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_J] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_K] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_L] = m_placeholder;

    aprilTagPoseMap[1] = m_aprilTag1;
    aprilTagPoseMap[2] = m_aprilTag2;
    aprilTagPoseMap[3] = m_aprilTag3;
    aprilTagPoseMap[4] = m_aprilTag4;
    aprilTagPoseMap[5] = m_aprilTag5;
    aprilTagPoseMap[6] = m_aprilTag6;
    aprilTagPoseMap[7] = m_aprilTag7;
    aprilTagPoseMap[8] = m_aprilTag8;
    aprilTagPoseMap[9] = m_aprilTag9;
    aprilTagPoseMap[10] = m_aprilTag10;
    aprilTagPoseMap[11] = m_aprilTag11;
    aprilTagPoseMap[12] = m_aprilTag12;
    aprilTagPoseMap[13] = m_aprilTag13;
    aprilTagPoseMap[14] = m_aprilTag14;
    aprilTagPoseMap[15] = m_aprilTag15;
    aprilTagPoseMap[16] = m_aprilTag16;
    aprilTagPoseMap[17] = m_aprilTag17;
    aprilTagPoseMap[18] = m_aprilTag18;
    aprilTagPoseMap[19] = m_aprilTag19;
    aprilTagPoseMap[20] = m_aprilTag20;
    aprilTagPoseMap[21] = m_aprilTag21;
    aprilTagPoseMap[22] = m_aprilTag22;

    FieldElementCalculator fc;
    fc.CalcPositionsForField(fieldConstantsPoseMap);

}
frc::Pose3d FieldConstants::GetFieldElementPose(FIELD_ELEMENT element)
{
    frc::Pose3d Pose3d = fieldConstantsPoseMap[element];
    return Pose3d;
}

void FieldConstants::ReadFieldCalibrationData()
{
    if (std::filesystem::exists(m_feildFilePath))
    {
        m_aprilTagVector = frc::AprilTagFieldLayout(m_feildFilePath).GetTags();
        for (unsigned int i = 1; i >= m_aprilTagVector.size(); i++)
        {
            frc::AprilTag tag = m_aprilTagVector[i];
            aprilTagPoseMap[i] = tag.pose;
        }
    }
}

frc::Pose3d FieldConstants::GetAprilTagPose(AprilTagIDs tag)
{
    auto pose = aprilTagPoseMap[tag];
    return pose;
}
