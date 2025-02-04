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

#include <frc/geometry/Pose3d.h>
#include <fieldData/FieldConstants.h>

struct TranslationToPose
{
    FieldElementCalculator::TRANSLATION_MAP translationKey;
    frc::Translation3d translation;
};

class FieldElementCalculator
{

public:
    void CalcPositionsForField(std::map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap);
    

    enum TRANSLATION_MAP
    {
        // Blue Coral Stations translations
        BLUE_CORAL_STATION_LEFT_2_BLUE_CORAL_STATION_LEFT_ALLIANCE,
        BLUE_CORAL_STATION_LEFT_2_BLUE_CORAL_STATION_LEFT_SIDEWALL,
        BLUE_CORAL_STATION_RIGHT_2_BLUE_CORAL_STATION_RIGHT_ALLIANCE,
        BLUE_CORAL_STATION_RIGHT_2_BLUE_CORAL_STATION_RIGHT_SIDEWALL,

        // Blue Barge translations
        BLUE_BARGE_FRONT_2_BLUE_LEFT_CAGE,
        BLUE_BARGE_FRONT_2_BLUE_RIGHT_CAGE,
        BLUE_BARGE_FRONT_2_BLUE_CENTER_CAGE,

        // Blue reef translations
        BLUE_REEF_AB_2_BLUE_REEF_A,
        BLUE_REEF_AB_2_BLUE_REEF_B,
        BLUE_REEF_CD_2_BLUE_REEF_C,
        BLUE_REEF_CD_2_BLUE_REEF_D,
        BLUE_REEF_EF_2_BLUE_REEF_E,
        BLUE_REEF_EF_2_BLUE_REEF_F,
        BLUE_REEF_GH_2_BLUE_REEF_G,
        BLUE_REEF_GH_2_BLUE_REEF_H,
        BLUE_REEF_IJ_2_BLUE_REEF_I,
        BLUE_REEF_IJ_2_BLUE_REEF_J,
        BLUE_REEF_KL_2_BLUE_REEF_K,
        BLUE_REEF_KL_2_BLUE_REEF_L,

        // Red Coral Stations translations
        RED_CORAL_STATION_LEFT_2_RED_CORAL_STATION_LEFT_ALLIANCE,
        RED_CORAL_STATION_LEFT_2_RED_CORAL_STATION_LEFT_SIDEWALL,
        RED_CORAL_STATION_RIGHT_2_RED_CORAL_STATION_RIGHT_ALLIANCE,
        RED_CORAL_STATION_RIGHT_2_RED_CORAL_STATION_RIGHT_ALLIANCE,

        // Red Barge translations
        RED_BARGE_FRONT_2_RED_LEFT_CAGE,
        RED_BARGE_FRONT_2_RED_RIGHT_CAGE,
        RED_BARGE_FRONT_2_RED_CENTER_CAGE,

        // Red reef translations
        RED_REEF_AB_2_RED_REEF_A,
        RED_REEF_AB_2_RED_REEF_B,
        RED_REEF_CD_2_RED_REEF_C,
        RED_REEF_CD_2_RED_REEF_D,
        RED_REEF_EF_2_RED_REEF_E,
        RED_REEF_EF_2_RED_REEF_F,
        RED_REEF_GH_2_RED_REEF_G,
        RED_REEF_GH_2_RED_REEF_H,
        RED_REEF_IJ_2_RED_REEF_I,
        RED_REEF_IJ_2_RED_REEF_J,
        RED_REEF_KL_2_RED_REEF_K,
        RED_REEF_KL_2_RED_REEF_L

    };

private:
    void InitializeTranslations();

    frc::Translation3d m_blueCoralStationLeft2BlueCoralStationLeftAlliance = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueCoralStationLeft2BlueCoralStationLeftSidewall = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueCoralStationRight2BlueCoralStationRightAlliance = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueCoralStationRight2BlueCoralStationRightSidewall = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueBargeFront2BlueLeftCage = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueBargeFront2BlueRightCage = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueBargeFront2BlueCenterCage = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefAB2BlueReefA = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefAB2BlueReefB = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefCD2BlueReefC = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefCD2BlueReefD = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefEF2BlueReefE = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefEF2BlueReefF = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefGH2BlueReefG = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefGH2BlueReefH = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefIJ2BlueReefI = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));
    
    frc::Translation3d m_blueReefIJ2BlueReefJ = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefKL2BlueReefK = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_blueReefKL2BlueReefL = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redCoralStationLeft2RedCoralStationLeftAlliance = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redCoralStationLeft2RedCoralStationLeftSidewall = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redCoralStationRight2RedCoralStationRightAlliance = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redCoralStationRight2RedCoralStationRightSidewall = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redBargeFront2RedLeftCage = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redBargeFront2RedRightCage = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redBargeFront2RedCenterCage = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefAB2RedReefA = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefAB2RedReefB = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefCD2RedReefC = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefCD2RedReefD = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefEF2RedReefE = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefEF2RedReefF = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefGH2RedReefG = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefGH2RedReefH = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefIJ2RedReefI = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefIJ2RedReefJ = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefKL2RedReefK = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));

    frc::Translation3d m_redReefKL2RedReefL = frc::Translation3d(
        units::length::meter_t(0.0),
        units::length::meter_t(0.0),
        units::length::meter_t(0.0));
        

    std::map<FieldConstants::FIELD_ELEMENT, TranslationToPose> translationConstantsMap;
};

