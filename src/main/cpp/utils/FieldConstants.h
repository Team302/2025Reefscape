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
        //2024 old elements
        BLUE_SOURCE,
        BLUE_AMP,
        BLUE_SPEAKER,
        BLUE_CENTER_STAGE,
        BLUE_RIGHT_STAGE,
        BLUE_LEFT_STAGE,
        RED_SOURCE,
        RED_AMP,
        RED_SPEAKER,
        RED_CENTER_STAGE,
        RED_RIGHT_STAGE,
        RED_LEFT_STAGE,
        // 2025
        BLUE_CORAL_STATION,
        BLUE_PROCESSOR,
        BLUE_REEF,
        BLUE_CENTER_CAGE,
        BLUE_RIGHT_CAGE,
        BLUE_LEFT_CAGE,
        RED_CORAL_STATION,
        RED_PROCESSOR,
        RED_REEF,
        RED_CENTER_CAGE,
        RED_RIGHT_CAGE,
        RED_LEFT_CAGE
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
    //TODO: update the meter t values to the values from the field
    const frc::Pose3d m_BlueCoralStation = frc::Pose3d(
        units::length::meter_t(15.63),
        units::length::meter_t(0.565),
        units::length::meter_t(1.36),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120)));
    const frc::Pose3d m_BlueProcessor = frc::Pose3d(
        units::length::meter_t(1.84),
        units::length::meter_t(8.2),
        units::length::meter_t(1.36),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(-90.0)));
    const frc::Pose3d m_BlueReef = frc::Pose3d(
        units::length::meter_t(-.04),
        units::length::meter_t(5.55),
        units::length::meter_t(1.45),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));
    const frc::Pose3d m_BlueCenterCage = frc::Pose3d(
        units::length::meter_t(5.32),
        units::length::meter_t(4.11),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));
    const frc::Pose3d m_BlueRightCage = frc::Pose3d(
        units::length::meter_t(4.64),
        units::length::meter_t(3.71),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(-120.0)));
    const frc::Pose3d m_BlueLeftCage = frc::Pose3d(
        units::length::meter_t(4.64),
        units::length::meter_t(4.5),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120.0)));
    // red
    const frc::Pose3d m_RedCoralStation = frc::Pose3d(
        units::length::meter_t(.91),
        units::length::meter_t(.565),
        units::length::meter_t(1.36),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(60.0)));
    const frc::Pose3d m_RedProcessor = frc::Pose3d(
        units::length::meter_t(14.7),
        units::length::meter_t(8.2),
        units::length::meter_t(1.36),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(-90.0)));
    const frc::Pose3d m_RedReef = frc::Pose3d(
        units::length::meter_t(16.58),
        units::length::meter_t(5.55),
        units::length::meter_t(1.45),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(180.0)));
    const frc::Pose3d m_RedCenterCage = frc::Pose3d(
        units::length::meter_t(11.22),
        units::length::meter_t(4.11),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(180.0)));
    const frc::Pose3d m_RedRightCage = frc::Pose3d(
        units::length::meter_t(11.9),
        units::length::meter_t(3.71),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(60.0)));
    const frc::Pose3d m_RedLeftCage = frc::Pose3d(
        units::length::meter_t(11.9),
        units::length::meter_t(4.5),
        units::length::meter_t(1.32),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(-60.0)));
    std::map<FIELD_ELEMENT, frc::Pose3d> fieldConstantsPoseMap;
};