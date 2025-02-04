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

void FieldElementCalculator::CalcPositionsForField(std::map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap)
{
    InitializeTransforms();

    // Iterate and update values
    for (auto &[key, translatedPose] : fieldConstantsPoseMap)
    {
        fieldConstantsPoseMap[key] = fieldConstantsPoseMap[transformConstantsMap[key].referencePose] + transformConstantsMap[key].transform;
    }
}



void FieldElementCalculator::InitializeTransforms()
{

    //no transforms for april tags on blue side
    transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_LEFT] =
        TransformToPose{FieldConstants::BLUE_CORAL_STATION_LEFT, m_noTransform};
    transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_RIGHT] =
        TransformToPose{FieldConstants::BLUE_CORAL_STATION_RIGHT, m_noTransform};
    transformConstantsMap[FieldConstants::BLUE_PROCESSOR] =
        TransformToPose{FieldConstants::BLUE_PROCESSOR, m_noTransform};
    transformConstantsMap[FieldConstants::BLUE_BARGE_FRONT] =
        TransformToPose{FieldConstants::BLUE_BARGE_FRONT, m_noTransform};
    transformConstantsMap[FieldConstants::BLUE_BARGE_BACK] =
        TransformToPose{FieldConstants::BLUE_BARGE_BACK, m_noTransform};
    transformConstantsMap[FieldConstants::BLUE_REEF_AB] =
        TransformToPose{FieldConstants::BLUE_REEF_AB, m_noTransform};
    transformConstantsMap[FieldConstants::BLUE_REEF_CD] =
        TransformToPose{FieldConstants::BLUE_REEF_CD, m_noTransform};
    transformConstantsMap[FieldConstants::BLUE_REEF_EF] =
        TransformToPose{FieldConstants::BLUE_REEF_EF, m_noTransform};
    transformConstantsMap[FieldConstants::BLUE_REEF_GH] =
        TransformToPose{FieldConstants::BLUE_REEF_GH, m_noTransform};
    transformConstantsMap[FieldConstants::BLUE_REEF_IJ] =
        TransformToPose{FieldConstants::BLUE_REEF_IJ, m_noTransform};
    transformConstantsMap[FieldConstants::BLUE_REEF_KL] =
        TransformToPose{FieldConstants::BLUE_REEF_KL, m_noTransform};

    
    // Blue Calculated Positions
    transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_LEFT_ALLIANCE] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_LEFT, m_blueCalcCoralLeftAlliance);
    transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_LEFT_SIDEWALL] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_LEFT, m_blueCalcCoralLeftSidewall);
    transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_RIGHT_ALLIANCE] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_RIGHT, m_blueCalcCoralRightAlliance);
    transformConstantsMap[FieldConstants::BLUE_CORAL_STATION_RIGHT_SIDEWALL] =
        TransformToPose(FieldConstants::BLUE_CORAL_STATION_RIGHT, m_blueCalcCoralRightSidewall);
    transformConstantsMap[FieldConstants::BLUE_LEFT_CAGE] =
        TransformToPose(FieldConstants::BLUE_BARGE_FRONT, m_blueCalcCageLeft);
    transformConstantsMap[FieldConstants::BLUE_RIGHT_CAGE] =
        TransformToPose(FieldConstants::BLUE_BARGE_FRONT, m_blueCalcCageRight);
    transformConstantsMap[FieldConstants::BLUE_CENTER_CAGE] =
        TransformToPose(FieldConstants::BLUE_BARGE_FRONT, m_blueCalcCageCenter);
    transformConstantsMap[FieldConstants::BLUE_REEF_A] =
        TransformToPose(FieldConstants::BLUE_REEF_AB, m_blueCalcReefA);
    transformConstantsMap[FieldConstants::BLUE_REEF_B] =
        TransformToPose(FieldConstants::BLUE_REEF_AB, m_blueCalcReefB);
    transformConstantsMap[FieldConstants::BLUE_REEF_C] =
        TransformToPose(FieldConstants::BLUE_REEF_CD, m_blueCalcReefC);
    transformConstantsMap[FieldConstants::BLUE_REEF_D] =
        TransformToPose(FieldConstants::BLUE_REEF_CD, m_blueCalcReefD);
    transformConstantsMap[FieldConstants::BLUE_REEF_E] =
        TransformToPose(FieldConstants::BLUE_REEF_EF, m_blueCalcReefE);
    transformConstantsMap[FieldConstants::BLUE_REEF_F] =
        TransformToPose(FieldConstants::BLUE_REEF_EF, m_blueCalcReefF);
    transformConstantsMap[FieldConstants::BLUE_REEF_G] =
        TransformToPose(FieldConstants::BLUE_REEF_GH, m_blueCalcReefG);
    transformConstantsMap[FieldConstants::BLUE_REEF_H] =
        TransformToPose(FieldConstants::BLUE_REEF_GH, m_blueCalcReefH);
    transformConstantsMap[FieldConstants::BLUE_REEF_I] =
        TransformToPose(FieldConstants::BLUE_REEF_IJ, m_blueCalcReefI);
    transformConstantsMap[FieldConstants::BLUE_REEF_J] =
        TransformToPose(FieldConstants::BLUE_REEF_IJ, m_blueCalcReefJ);
    transformConstantsMap[FieldConstants::BLUE_REEF_K] =
        TransformToPose(FieldConstants::BLUE_REEF_KL, m_blueCalcReefK);
    transformConstantsMap[FieldConstants::BLUE_REEF_L] =
        TransformToPose(FieldConstants::BLUE_REEF_KL, m_blueCalcReefL);

    // no transforms for april tags on red side
    transformConstantsMap[FieldConstants::RED_CORAL_STATION_LEFT] =
        TransformToPose{FieldConstants::RED_CORAL_STATION_LEFT, m_noTransform};
    transformConstantsMap[FieldConstants::RED_CORAL_STATION_RIGHT] =
        TransformToPose{FieldConstants::RED_CORAL_STATION_RIGHT, m_noTransform};
    transformConstantsMap[FieldConstants::RED_PROCESSOR] =
        TransformToPose{FieldConstants::RED_PROCESSOR, m_noTransform};
    transformConstantsMap[FieldConstants::RED_BARGE_FRONT] =
        TransformToPose{FieldConstants::RED_BARGE_FRONT, m_noTransform};
    transformConstantsMap[FieldConstants::RED_BARGE_BACK] =
        TransformToPose{FieldConstants::RED_BARGE_BACK, m_noTransform};
    transformConstantsMap[FieldConstants::RED_REEF_AB] =
        TransformToPose{FieldConstants::RED_REEF_AB, m_noTransform};
    transformConstantsMap[FieldConstants::RED_REEF_CD] =
        TransformToPose{FieldConstants::RED_REEF_CD, m_noTransform};
    transformConstantsMap[FieldConstants::RED_REEF_EF] =
        TransformToPose{FieldConstants::RED_REEF_EF, m_noTransform};
    transformConstantsMap[FieldConstants::RED_REEF_GH] =
        TransformToPose{FieldConstants::RED_REEF_GH, m_noTransform};
    transformConstantsMap[FieldConstants::RED_REEF_IJ] =
        TransformToPose{FieldConstants::RED_REEF_IJ, m_noTransform};
    transformConstantsMap[FieldConstants::RED_REEF_KL] =
        TransformToPose{FieldConstants::RED_REEF_KL, m_noTransform};

    // Red Calculated Positions
    transformConstantsMap[FieldConstants::RED_CORAL_STATION_LEFT_ALLIANCE] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_LEFT, m_redCalcCoralLeftAlliance);
    transformConstantsMap[FieldConstants::RED_CORAL_STATION_LEFT_SIDEWALL] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_LEFT, m_redCalcCoralLeftSidewall);
    transformConstantsMap[FieldConstants::RED_CORAL_STATION_RIGHT_ALLIANCE] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_RIGHT, m_redCalcCoralRightAlliance);
    transformConstantsMap[FieldConstants::RED_CORAL_STATION_RIGHT_SIDEWALL] =
        TransformToPose(FieldConstants::RED_CORAL_STATION_RIGHT, m_redCalcCoralRightSidewall);
    transformConstantsMap[FieldConstants::RED_LEFT_CAGE] =
        TransformToPose(FieldConstants::RED_BARGE_FRONT, m_redCalcCageLeft);
    transformConstantsMap[FieldConstants::RED_RIGHT_CAGE] =
        TransformToPose(FieldConstants::RED_BARGE_FRONT, m_redCalcCageRight);
    transformConstantsMap[FieldConstants::RED_CENTER_CAGE] =
        TransformToPose(FieldConstants::RED_BARGE_FRONT, m_redCalcCageCenter);
    transformConstantsMap[FieldConstants::RED_REEF_A] =
        TransformToPose(FieldConstants::RED_REEF_AB, m_redCalcReefA);
    transformConstantsMap[FieldConstants::RED_REEF_B] =
        TransformToPose(FieldConstants::RED_REEF_AB, m_redCalcReefB);
    transformConstantsMap[FieldConstants::RED_REEF_C] =
        TransformToPose(FieldConstants::RED_REEF_CD, m_redCalcReefC);
    transformConstantsMap[FieldConstants::RED_REEF_D] =
        TransformToPose(FieldConstants::RED_REEF_CD, m_redCalcReefD);
    transformConstantsMap[FieldConstants::RED_REEF_E] =
        TransformToPose(FieldConstants::RED_REEF_EF, m_redCalcReefE);
    transformConstantsMap[FieldConstants::RED_REEF_F] =
        TransformToPose(FieldConstants::RED_REEF_EF, m_redCalcReefF);
    transformConstantsMap[FieldConstants::RED_REEF_G] =
        TransformToPose(FieldConstants::RED_REEF_GH, m_redCalcReefG);
    transformConstantsMap[FieldConstants::RED_REEF_H] =
        TransformToPose(FieldConstants::RED_REEF_GH, m_redCalcReefH);
    transformConstantsMap[FieldConstants::RED_REEF_I] =
        TransformToPose(FieldConstants::RED_REEF_IJ, m_redCalcReefI);
    transformConstantsMap[FieldConstants::RED_REEF_J] =
        TransformToPose(FieldConstants::RED_REEF_IJ, m_redCalcReefJ);
    transformConstantsMap[FieldConstants::RED_REEF_K] =
        TransformToPose(FieldConstants::RED_REEF_KL, m_redCalcReefK);
    transformConstantsMap[FieldConstants::RED_REEF_L] =
        TransformToPose(FieldConstants::RED_REEF_KL, m_redCalcReefL);

    
}
