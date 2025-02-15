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
#include "FieldElementCalculator.h"
#include "FieldConstantsPoseLogger.h"
#include "vision/DragonVisionStructLogger.h"
#include "utils/logging/Logger.h"

void FieldElementCalculator::CalcPositionsForField(std::map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap)
{
    InitializeTransforms();
    CalculateCenters(fieldConstantsPoseMap);

    // update all of the calculated values only
    for (auto &[key, translatedPose] : m_transformCalculatedMap)
    {
        fieldConstantsPoseMap[key] = fieldConstantsPoseMap[m_transformCalculatedMap[key].referencePose] + m_transformCalculatedMap[key].transform + m_halfRobotTransform;
    }

    // after transform the tags, if the tags are transformed first it doubly transforms the calculated values
    for (auto &[key, unusedValue] : m_transformTagsMap)
    {
        fieldConstantsPoseMap[key] = fieldConstantsPoseMap[key] + m_halfRobotTransform;
    }

#ifdef INCLUDE_FIELD_ELEMENT_POSE_LOGGER
    FieldConstantsPoseLogger fpl;
    fpl.LogFieldElementPoses(fieldConstantsPoseMap);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("PoseLogger"), std::string("Logging"), std::string(""));
#endif
}

frc::Pose3d FieldElementCalculator::CalcOffsetPositionForElement(frc::Pose3d &poseOfFaceTag, FieldConstants::FIELD_ELEMENT_OFFSETS offset)
{
    frc::Transform3d transformToApply = m_calcLeftStick;
    if (offset == FieldConstants::FIELD_ELEMENT_OFFSETS::RIGHT_STICK)
    {
        transformToApply = m_calcRightStick;
    }
    return poseOfFaceTag + transformToApply + m_halfRobotTransform;
}

void FieldElementCalculator::InitializeTransforms()
{

    // no transforms for april tags on blue side 
    // TODO: can these all be null and then not use the values in the 2nd formula?
    m_transformTagsMap[FieldConstants::BLUE_CORAL_STATION_LEFT];
    m_transformTagsMap[FieldConstants::BLUE_CORAL_STATION_RIGHT];
    m_transformTagsMap[FieldConstants::BLUE_PROCESSOR];
    m_transformTagsMap[FieldConstants::BLUE_BARGE_FRONT];
    m_transformTagsMap[FieldConstants::BLUE_BARGE_BACK];
    m_transformTagsMap[FieldConstants::BLUE_REEF_AB];
    m_transformTagsMap[FieldConstants::BLUE_REEF_CD];
    m_transformTagsMap[FieldConstants::BLUE_REEF_EF];
    m_transformTagsMap[FieldConstants::BLUE_REEF_GH];
    m_transformTagsMap[FieldConstants::BLUE_REEF_IJ];
    m_transformTagsMap[FieldConstants::BLUE_REEF_KL];

    // Blue Calculated Positions
    m_transformCalculatedMap[FieldConstants::BLUE_CORAL_STATION_LEFT_ALLIANCE] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_LEFT, m_calcCoralLeftAlliance);
    m_transformCalculatedMap[FieldConstants::BLUE_CORAL_STATION_LEFT_SIDEWALL] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_LEFT, m_calcCoralLeftSidewall);
    m_transformCalculatedMap[FieldConstants::BLUE_CORAL_STATION_RIGHT_ALLIANCE] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_RIGHT, m_calcCoralRightAlliance);
    m_transformCalculatedMap[FieldConstants::BLUE_CORAL_STATION_RIGHT_SIDEWALL] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_RIGHT, m_calcCoralRightSidewall);
    m_transformCalculatedMap[FieldConstants::BLUE_LEFT_CAGE] =
        TransformToPose(FieldConstants::BLUE_BARGE_FRONT, m_calcCageLeft);
    m_transformCalculatedMap[FieldConstants::BLUE_RIGHT_CAGE] =
        TransformToPose(FieldConstants::BLUE_BARGE_FRONT, m_calcCageRight);
    m_transformCalculatedMap[FieldConstants::BLUE_CENTER_CAGE] =
        TransformToPose(FieldConstants::BLUE_BARGE_FRONT, m_noTransform);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_A] =
        TransformToPose(FieldConstants::BLUE_REEF_AB, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_B] =
        TransformToPose(FieldConstants::BLUE_REEF_AB, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_C] =
        TransformToPose(FieldConstants::BLUE_REEF_CD, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_D] =
        TransformToPose(FieldConstants::BLUE_REEF_CD, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_E] =
        TransformToPose(FieldConstants::BLUE_REEF_EF, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_F] =
        TransformToPose(FieldConstants::BLUE_REEF_EF, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_G] =
        TransformToPose(FieldConstants::BLUE_REEF_GH, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_H] =
        TransformToPose(FieldConstants::BLUE_REEF_GH, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_I] =
        TransformToPose(FieldConstants::BLUE_REEF_IJ, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_J] =
        TransformToPose(FieldConstants::BLUE_REEF_IJ, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_K] =
        TransformToPose(FieldConstants::BLUE_REEF_KL, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::BLUE_REEF_L] =
        TransformToPose(FieldConstants::BLUE_REEF_KL, m_calcRightStick);


    // no transforms for april tags on red side
    m_transformTagsMap[FieldConstants::RED_CORAL_STATION_LEFT];
    m_transformTagsMap[FieldConstants::RED_CORAL_STATION_RIGHT];
    m_transformTagsMap[FieldConstants::RED_BARGE_FRONT];
    m_transformTagsMap[FieldConstants::RED_BARGE_BACK];
    m_transformTagsMap[FieldConstants::RED_REEF_AB];
    m_transformTagsMap[FieldConstants::RED_REEF_CD];
    m_transformTagsMap[FieldConstants::RED_REEF_EF];
    m_transformTagsMap[FieldConstants::RED_REEF_GH];
    m_transformTagsMap[FieldConstants::RED_REEF_IJ];
    m_transformTagsMap[FieldConstants::RED_REEF_KL];

    // Red Calculated Positions
    m_transformCalculatedMap[FieldConstants::RED_CORAL_STATION_LEFT_ALLIANCE] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_LEFT, m_calcCoralLeftAlliance);
    m_transformCalculatedMap[FieldConstants::RED_CORAL_STATION_LEFT_SIDEWALL] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_LEFT, m_calcCoralLeftSidewall);
    m_transformCalculatedMap[FieldConstants::RED_CORAL_STATION_RIGHT_ALLIANCE] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_RIGHT, m_calcCoralRightAlliance);
    m_transformCalculatedMap[FieldConstants::RED_CORAL_STATION_RIGHT_SIDEWALL] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_RIGHT, m_calcCoralRightSidewall);
    m_transformCalculatedMap[FieldConstants::RED_LEFT_CAGE] =
        TransformToPose(FieldConstants::RED_BARGE_FRONT, m_calcCageLeft);
    m_transformCalculatedMap[FieldConstants::RED_RIGHT_CAGE] =
        TransformToPose(FieldConstants::RED_BARGE_FRONT, m_calcCageRight);
    m_transformCalculatedMap[FieldConstants::RED_CENTER_CAGE] =
        TransformToPose(FieldConstants::RED_BARGE_FRONT, m_noTransform);
    m_transformCalculatedMap[FieldConstants::RED_REEF_A] =
        TransformToPose(FieldConstants::RED_REEF_AB, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_B] =
        TransformToPose(FieldConstants::RED_REEF_AB, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_C] =
        TransformToPose(FieldConstants::RED_REEF_CD, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_D] =
        TransformToPose(FieldConstants::RED_REEF_CD, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_E] =
        TransformToPose(FieldConstants::RED_REEF_EF, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_F] =
        TransformToPose(FieldConstants::RED_REEF_EF, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_G] =
        TransformToPose(FieldConstants::RED_REEF_GH, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_H] =
        TransformToPose(FieldConstants::RED_REEF_GH, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_I] =
        TransformToPose(FieldConstants::RED_REEF_IJ, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_J] =
        TransformToPose(FieldConstants::RED_REEF_IJ, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_K] =
        TransformToPose(FieldConstants::RED_REEF_KL, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::RED_REEF_L] =
        TransformToPose(FieldConstants::RED_REEF_KL, m_calcRightStick);

}

void FieldElementCalculator::CalculateCenters(std::map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap)
{
    fieldConstantsPoseMap[FieldConstants::RED_REEF_CENTER] = AverageHexagonPose(
        fieldConstantsPoseMap[FieldConstants::RED_REEF_AB],
        fieldConstantsPoseMap[FieldConstants::RED_REEF_CD],
        fieldConstantsPoseMap[FieldConstants::RED_REEF_EF],
        fieldConstantsPoseMap[FieldConstants::RED_REEF_GH],
        fieldConstantsPoseMap[FieldConstants::RED_REEF_IJ],
        fieldConstantsPoseMap[FieldConstants::RED_REEF_KL]);

    fieldConstantsPoseMap[FieldConstants::BLUE_REEF_CENTER] = AverageHexagonPose(
        fieldConstantsPoseMap[FieldConstants::BLUE_REEF_AB],
        fieldConstantsPoseMap[FieldConstants::BLUE_REEF_CD],
        fieldConstantsPoseMap[FieldConstants::BLUE_REEF_EF],
        fieldConstantsPoseMap[FieldConstants::BLUE_REEF_GH],
        fieldConstantsPoseMap[FieldConstants::BLUE_REEF_IJ],
        fieldConstantsPoseMap[FieldConstants::BLUE_REEF_KL]);
}

frc::Pose3d FieldElementCalculator::AverageHexagonPose(frc::Pose3d &pose1, frc::Pose3d &pose2, frc::Pose3d &pose3, frc::Pose3d &pose4, frc::Pose3d &pose5, frc::Pose3d &pose6)
{

    
    units::length::meter_t averageX = (pose1.X() + pose2.X() + pose3.X() + pose4.X() + pose5.X() + pose6.X()) / 6;
    units::length::meter_t averageY = (pose1.Y() + pose2.Y() + pose3.Y() + pose4.Y() + pose5.Y() + pose6.Y()) / 6;
    units::length::meter_t averageZ = (pose1.Z() + pose2.Z() + pose3.Z() + pose4.Z() + pose5.Z() + pose6.Z()) / 6;

    return frc::Pose3d(averageX, averageY, averageZ, frc::Rotation3d());
}