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

    // Iterate and update values
    for (auto &[key, translatedPose] : fieldConstantsPoseMap)
    {
        fieldConstantsPoseMap[key] = fieldConstantsPoseMap[m_transformConstantsMap[key].referencePose] + m_transformConstantsMap[key].transform;
    }

    #ifdef INCLUDE_FIELD_ELEMENT_POSE_LOGGER
    FieldConstantsPoseLogger fpl;
    fpl.LogFieldElementPoses(fieldConstantsPoseMap);
    #endif
}

void FieldElementCalculator::InitializeTransforms()
{

    // no transforms for april tags on blue side
    m_transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_LEFT] =
        TransformToPose{FieldConstants::BLUE_CORAL_STATION_LEFT, m_noTransform};
    m_transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_RIGHT] =
        TransformToPose{FieldConstants::BLUE_CORAL_STATION_RIGHT, m_noTransform};
    m_transformConstantsMap[FieldConstants::BLUE_PROCESSOR] =
        TransformToPose{FieldConstants::BLUE_PROCESSOR, m_noTransform};
    m_transformConstantsMap[FieldConstants::BLUE_BARGE_FRONT] =
        TransformToPose{FieldConstants::BLUE_BARGE_FRONT, m_noTransform};
    m_transformConstantsMap[FieldConstants::BLUE_BARGE_BACK] =
        TransformToPose{FieldConstants::BLUE_BARGE_BACK, m_noTransform};
    m_transformConstantsMap[FieldConstants::BLUE_REEF_AB] =
        TransformToPose{FieldConstants::BLUE_REEF_AB, m_noTransform};
    m_transformConstantsMap[FieldConstants::BLUE_REEF_CD] =
        TransformToPose{FieldConstants::BLUE_REEF_CD, m_noTransform};
    m_transformConstantsMap[FieldConstants::BLUE_REEF_EF] =
        TransformToPose{FieldConstants::BLUE_REEF_EF, m_noTransform};
    m_transformConstantsMap[FieldConstants::BLUE_REEF_GH] =
        TransformToPose{FieldConstants::BLUE_REEF_GH, m_noTransform};
    m_transformConstantsMap[FieldConstants::BLUE_REEF_IJ] =
        TransformToPose{FieldConstants::BLUE_REEF_IJ, m_noTransform};
    m_transformConstantsMap[FieldConstants::BLUE_REEF_KL] =
        TransformToPose{FieldConstants::BLUE_REEF_KL, m_noTransform};

    // Blue Calculated Positions
    m_transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_LEFT_ALLIANCE] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_LEFT, m_calcCoralLeftAlliance);
    m_transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_LEFT_SIDEWALL] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_LEFT, m_calcCoralLeftSidewall);
    m_transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_RIGHT_ALLIANCE] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_RIGHT, m_calcCoralRightAlliance);
    m_transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_RIGHT_SIDEWALL] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_RIGHT, m_calcCoralRightSidewall);
    m_transformConstantsMap[FieldConstants::BLUE_LEFT_CAGE] =
        TransformToPose(FieldConstants::BLUE_BARGE_FRONT, m_calcCageLeft);
    m_transformConstantsMap[FieldConstants::BLUE_RIGHT_CAGE] =
        TransformToPose(FieldConstants::BLUE_BARGE_FRONT, m_calcCageRight);
    m_transformConstantsMap[FieldConstants::BLUE_CENTER_CAGE] =
        TransformToPose(FieldConstants::BLUE_BARGE_FRONT, m_noTransform);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_A] =
        TransformToPose(FieldConstants::BLUE_REEF_AB, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_B] =
        TransformToPose(FieldConstants::BLUE_REEF_AB, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_C] =
        TransformToPose(FieldConstants::BLUE_REEF_CD, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_D] =
        TransformToPose(FieldConstants::BLUE_REEF_CD, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_E] =
        TransformToPose(FieldConstants::BLUE_REEF_EF, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_F] =
        TransformToPose(FieldConstants::BLUE_REEF_EF, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_G] =
        TransformToPose(FieldConstants::BLUE_REEF_GH, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_H] =
        TransformToPose(FieldConstants::BLUE_REEF_GH, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_I] =
        TransformToPose(FieldConstants::BLUE_REEF_IJ, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_J] =
        TransformToPose(FieldConstants::BLUE_REEF_IJ, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_K] =
        TransformToPose(FieldConstants::BLUE_REEF_KL, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_L] =
        TransformToPose(FieldConstants::BLUE_REEF_KL, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::BLUE_REEF_CENTER] =
        TransformToPose(FieldConstants::BLUE_REEF_CENTER, m_noTransform);

    // no transforms for april tags on red side
    m_transformConstantsMap[FieldConstants::RED_CORAL_STATION_LEFT] =
        TransformToPose{FieldConstants::RED_CORAL_STATION_LEFT, m_noTransform};
    m_transformConstantsMap[FieldConstants::RED_CORAL_STATION_RIGHT] =
        TransformToPose{FieldConstants::RED_CORAL_STATION_RIGHT, m_noTransform};
    m_transformConstantsMap[FieldConstants::RED_PROCESSOR] =
        TransformToPose{FieldConstants::RED_PROCESSOR, m_noTransform};
    m_transformConstantsMap[FieldConstants::RED_BARGE_FRONT] =
        TransformToPose{FieldConstants::RED_BARGE_FRONT, m_noTransform};
    m_transformConstantsMap[FieldConstants::RED_BARGE_BACK] =
        TransformToPose{FieldConstants::RED_BARGE_BACK, m_noTransform};
    m_transformConstantsMap[FieldConstants::RED_REEF_AB] =
        TransformToPose{FieldConstants::RED_REEF_AB, m_noTransform};
    m_transformConstantsMap[FieldConstants::RED_REEF_CD] =
        TransformToPose{FieldConstants::RED_REEF_CD, m_noTransform};
    m_transformConstantsMap[FieldConstants::RED_REEF_EF] =
        TransformToPose{FieldConstants::RED_REEF_EF, m_noTransform};
    m_transformConstantsMap[FieldConstants::RED_REEF_GH] =
        TransformToPose{FieldConstants::RED_REEF_GH, m_noTransform};
    m_transformConstantsMap[FieldConstants::RED_REEF_IJ] =
        TransformToPose{FieldConstants::RED_REEF_IJ, m_noTransform};
    m_transformConstantsMap[FieldConstants::RED_REEF_KL] =
        TransformToPose{FieldConstants::RED_REEF_KL, m_noTransform};

    // Red Calculated Positions
    m_transformConstantsMap[FieldConstants::RED_CORAL_STATION_LEFT_ALLIANCE] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_LEFT, m_calcCoralLeftAlliance);
    m_transformConstantsMap[FieldConstants::RED_CORAL_STATION_LEFT_SIDEWALL] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_LEFT, m_calcCoralLeftSidewall);
    m_transformConstantsMap[FieldConstants::RED_CORAL_STATION_RIGHT_ALLIANCE] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_RIGHT, m_calcCoralRightAlliance);
    m_transformConstantsMap[FieldConstants::RED_CORAL_STATION_RIGHT_SIDEWALL] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_RIGHT, m_calcCoralRightSidewall);
    m_transformConstantsMap[FieldConstants::RED_LEFT_CAGE] =
        TransformToPose(FieldConstants::RED_BARGE_FRONT, m_calcCageLeft);
    m_transformConstantsMap[FieldConstants::RED_RIGHT_CAGE] =
        TransformToPose(FieldConstants::RED_BARGE_FRONT, m_calcCageRight);
    m_transformConstantsMap[FieldConstants::RED_CENTER_CAGE] =
        TransformToPose(FieldConstants::RED_BARGE_FRONT, m_noTransform);
    m_transformConstantsMap[FieldConstants::RED_REEF_A] =
        TransformToPose(FieldConstants::RED_REEF_AB, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_B] =
        TransformToPose(FieldConstants::RED_REEF_AB, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_C] =
        TransformToPose(FieldConstants::RED_REEF_CD, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_D] =
        TransformToPose(FieldConstants::RED_REEF_CD, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_E] =
        TransformToPose(FieldConstants::RED_REEF_EF, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_F] =
        TransformToPose(FieldConstants::RED_REEF_EF, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_G] =
        TransformToPose(FieldConstants::RED_REEF_GH, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_H] =
        TransformToPose(FieldConstants::RED_REEF_GH, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_I] =
        TransformToPose(FieldConstants::RED_REEF_IJ, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_J] =
        TransformToPose(FieldConstants::RED_REEF_IJ, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_K] =
        TransformToPose(FieldConstants::RED_REEF_KL, m_calcLeftStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_L] =
        TransformToPose(FieldConstants::RED_REEF_KL, m_calcRightStick);
    m_transformConstantsMap[FieldConstants::RED_REEF_CENTER] =
        TransformToPose(FieldConstants::RED_REEF_CENTER, m_noTransform);
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