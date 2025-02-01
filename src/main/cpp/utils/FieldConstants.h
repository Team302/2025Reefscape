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
#include <frc/geometry/Rotation3d.h>
#include "units/angle.h"
#include "units/base.h"
class FieldConstants
{
public:
    static FieldConstants *GetInstance();
    enum FIELD_ELEMENT
    {
        BLUE_REEF_J,
        BLUE_REEF_I,
        BLUE_REEF_H,
        BLUE_REEF_G,
        BLUE_REEF_F,
        BLUE_REEF_E,
        BLUE_REEF_D,
        BLUE_REEF_C,
        BLUE_REEF_B,
        BLUE_REEF_A,
        BLUE_REEF_L,
        BLUE_REEF_K,
        RED_REEF_D,
        RED_REEF_C,
        RED_REEF_B,
        RED_REEF_A,
        RED_REEF_L,
        RED_REEF_K,
        RED_REEF_J,
        RED_REEF_I,
        RED_REEF_H,
        RED_REEF_G,
        RED_REEF_F,
        RED_REEF_E,
        BLUE_LEFT_CAGE,
        BLUE_MIDDLE_CAGE,
        BLUE_RIGHT_CAGE,
        RED_LEFT_CAGE,
        RED_MIDDLE_CAGE,
        RED_RIGHT_CAGE,
        BLUE_FAR_LEFT_HUMAN_PLAYER_,
        BLUE_FAR_RIGHT_HUMAN_PLAYER,
        RED_FAR_RIGHT_HUMAN_PLAYER,
        RED_FAR_LEFT_HUMAN_PLAYER,
        BLUE_HUMAN_PROCESSOR,
        RED_HUMAN_PROCESSOR,
        ALGAE
    };
    frc::Pose3d GetFieldElement(FIELD_ELEMENT element);

private:
    // make a singleton
    static FieldConstants *m_instance;
    // make constructor private
    FieldConstants();
    // make singleton copy constructor private
    FieldConstants(const FieldConstants &) = delete;
    FieldConstants &operator=(const FieldConstants &) = delete;

    // specified here
    // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025LayoutMarkingDiagram.pdf

    // blue
    const frc::Pose3d m_BlueReef = frc::Pose3d(
        units::length::meter_t(15.63),
        units::length::meter_t(0.565),
        units::length::meter_t(1.36),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120)));
    const frc::Pose3d m_RedReef = frc::Pose3d(
        units::length::meter_t(1.84),
        units::length::meter_t(8.2),
        units::length::meter_t(1.36),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(-90.0)));
    const frc::Pose3d m_BlueCage = frc::Pose3d(
        units::length::meter_t(-.04),
        units::length::meter_t(5.55),
        units::length::meter_t(1.45),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));
    const frc::Pose3d m_RedCage = frc::Pose3d(
        units::length::meter_t(5.32),
        units::length::meter_t(4.11),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));
    const frc::Pose3d m_BlueHumanPlayer = frc::Pose3d(
        units::length::meter_t(4.64),
        units::length::meter_t(3.71),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(-120.0)));
    const frc::Pose3d m_RedHumanPlayer = frc::Pose3d(
        units::length::meter_t(4.64),
        units::length::meter_t(4.5),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120.0)));
    const frc::Pose3d m_BlueHumamProcessor = frc::Pose3d(
        units::length::meter_t(4.64),
        units::length::meter_t(3.71),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(-120.0)));
    const frc::Pose3d m_RedHumanProcessor = frc::Pose3d(
        units::length::meter_t(4.64),
        units::length::meter_t(4.5),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120.0)));
    std::map<FIELD_ELEMENT, frc::Pose3d>
        fieldConstantsPoseMap;
};
