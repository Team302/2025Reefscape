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
#include <fielddata/FieldConstants.h>

struct TransformToPose
{
    FieldConstants::FIELD_ELEMENT referencePose;
    frc::Transform3d transform;
};

class FieldElementCalculator
{

public:
    void CalcPositionsForField(std::map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap);

private:
    void InitializeTransforms();
    void CalculateCenters(std::map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap);
    frc::Pose3d AverageHexagonPose(frc::Pose3d &pose1, frc::Pose3d &pose2, frc::Pose3d &pose3, frc::Pose3d &pose4, frc::Pose3d &pose5, frc::Pose3d &pose6);

    frc::Transform3d m_noTransform = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(0.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    // TODO: These need to be updated with the correct transforms to the physical field elements

    // the field position (0.0) is behind the blue right alliance wall
    // x runs down the lenght of the field
    // y runs across the field
    frc::Transform3d m_blueCalcCoralLeftAlliance = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-30.0),
            units::length::inch_t(-14.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcCoralLeftSidewall = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(30.0),
            units::length::inch_t(-14.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcCoralRightAlliance = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(30.0),
            units::length::inch_t(-14.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcCoralRightSidewall = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0),
            units::length::inch_t(-30.0),
            units::length::inch_t(-14.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcCageLeft = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-42.5),
            units::length::inch_t(-32.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcCageRight = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(42.5),
            units::length::inch_t(-32.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcCageCenter = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(0.0),
            units::length::inch_t(-32.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefCenter = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(0.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    //reef A is on blue driver station left stick
    frc::Transform3d m_blueCalcReefA = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    //reef b is on blue driver station right stick
    frc::Transform3d m_blueCalcReefB = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefC = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefD = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefE = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefF = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefG = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefH = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefI = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefJ = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefK = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_blueCalcReefL = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    // Red Calculated Positions
    frc::Transform3d m_redCalcCoralLeftAlliance = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-30.0),
            units::length::inch_t(-14.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcCoralLeftSidewall = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(30.0),
            units::length::inch_t(-14.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcCoralRightAlliance = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(30.0),
            units::length::inch_t(-14.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcCoralRightSidewall = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-30.0),
            units::length::inch_t(-14.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcCageLeft = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-42.5),
            units::length::inch_t(-32.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcCageRight = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(42.5),
            units::length::inch_t(-32)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcCageCenter = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(0.0),
            units::length::inch_t(-32)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefCenter = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(0.0),
            units::length::inch_t(48.0)),
        frc::Rotation3d());

    //a is on the red driver station left stick
    frc::Transform3d m_redCalcReefA = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    //b is on the red driver station right stick
    frc::Transform3d m_redCalcReefB = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefC = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefD = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefE = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefF = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefG = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefH = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefI = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefJ = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefK = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    frc::Transform3d m_redCalcReefL = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(22.0)),
        frc::Rotation3d());

    std::map<FieldConstants::FIELD_ELEMENT, TransformToPose> m_transformConstantsMap;
};
