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

    CalculateDerivedValues();

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
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_ALLIANCE] = m_blueCalcCoralLeftAlliance;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_SIDEWALL] = m_blueCalcCoralLeftSidewall;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_ALLIANCE] = m_blueCalcCoralRightAlliance;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_SIDEWALL] = m_blueCalcCoralRightSidewall;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_RIGHT_CAGE] = m_blueCalcRightCage;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_LEFT_CAGE] = m_blueCalcLeftCage;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_CENTER] = m_blueCalcReefCenter;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_A] = m_blueCalcReefA;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_B] = m_blueCalcReefB;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_C] = m_blueCalcReefC;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_D] = m_blueCalcReefD;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_E] = m_blueCalcReefE;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_F] = m_blueCalcReefF;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_G] = m_blueCalcReefG;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_H] = m_blueCalcReefH;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_I] = m_blueCalcReefI;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_J] = m_blueCalcReefJ;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_K] = m_blueCalcReefK;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_L] = m_blueCalcReefL;

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
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_LEFT_ALLIANCE] = m_redCalcCoralLeftAlliance;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_LEFT_SIDEWALL] = m_redCalcCoralLeftSidewall;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_ALLIANCE] = m_redCalcCoralRightAlliance;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_SIDEWALL] = m_redCalcCoralRightSidewall;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_RIGHT_CAGE] = m_redCalcRightCage;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_LEFT_CAGE] = m_redCalcLeftCage;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_CENTER] = m_redCalcReefCenter;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_A] = m_redCalcReefA;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_B] = m_redCalcReefB;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_C] = m_redCalcReefC;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_D] = m_redCalcReefD;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_E] = m_redCalcReefE;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_F] = m_redCalcReefF;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_G] = m_redCalcReefG;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_H] = m_redCalcReefH;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_I] = m_redCalcReefI;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_J] = m_redCalcReefJ;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_K] = m_redCalcReefK;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_L] = m_redCalcReefL;
}
frc::Pose3d FieldConstants::GetFieldElement(FIELD_ELEMENT element)
{
    frc::Pose3d Pose3d = fieldConstantsPoseMap[element];
    return Pose3d;
}

void FieldConstants::ReadFieldCalibrationData()
{
    // TODO: implement 242
}

void FieldConstants::CalculateDerivedValues()
{
    CalculateReefPositions();
    CalculateCSALocations();
    CalculateCageLocations();
}
void FieldConstants::CalculateReefPositions()
{
    // TODO: implement 249
}
void FieldConstants::CalculateCSALocations()
{
    // TODO: implement 250
}
void FieldConstants::CalculateCageLocations()
{
    // TODO: implement 254
}