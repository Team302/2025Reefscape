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

#include <fielddata/FieldConstantsPoseLogger.h>

#ifdef INCLUDE_FIELD_ELEMENT_POSE_LOGGER
#include "wpi/DataLog.h"
#include "frc/DataLogManager.h"
#include "frc/geometry/Pose3d.h"

void FieldConstantsPoseLogger::LogFieldElementPoses(std::map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap)
{
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_A] = "RED_REEF_A";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_B] = "RED_REEF_B";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_C] = "RED_REEF_C";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_D] = "RED_REEF_D";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_E] = "RED_REEF_E";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_F] = "RED_REEF_F";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_G] = "RED_REEF_G";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_H] = "RED_REEF_H";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_I] = "RED_REEF_I";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_J] = "RED_REEF_J";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_K] = "RED_REEF_K";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_L] = "RED_REEF_L";

    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_REEF_CENTER] = "RED_REEF_CENTER";

    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_LEFT_CAGE] = "RED_LEFT_CAGE";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_RIGHT_CAGE] = "RED_RIGHT_CAGE";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_CENTER_CAGE] = "RED_CENTER_CAGE";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT_SIDEWALL] = "RED_CORAL_STATION_LEFT_SIDEWALL";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_ALLIANCE] = "RED_CORAL_STATION_RIGHT_ALLIANCE";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_SIDEWALL] = "RED_CORAL_STATION_RIGHT_SIDEWALL";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT_ALLIANCE] = "RED_CORAL_STATION_LEFT_ALLIANCE";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT_SIDEWALL] = "RED_CORAL_STATION_LEFT_SIDEWALL";

    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_PROCESSOR] = "BLUE_PROCESSOR";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT] = "BLUE_BARGE_FRONT";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_BARGE_BACK] = "BLUE_BARGE_BACK";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_AB] = "BLUE_REEF_AB";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_CD] = "BLUE_REEF_CD";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_EF] = "BLUE_REEF_EF";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_GH] = "BLUE_REEF_GH";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_IJ] = "BLUE_REEF_IJ";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_KL] = "BLUE_REEF_KL";

    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_A] = "BLUE_REEF_A";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_B] = "BLUE_REEF_B";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_C] = "BLUE_REEF_C";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_D] = "BLUE_REEF_D";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_E] = "BLUE_REEF_E";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_F] = "BLUE_REEF_F";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_G] = "BLUE_REEF_G";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_H] = "BLUE_REEF_H";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_I] = "BLUE_REEF_I";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_J] = "BLUE_REEF_J";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_K] = "BLUE_REEF_K";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_L] = "BLUE_REEF_L";

    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_CENTER] = "BLUE_REEF_CENTER";

    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_LEFT_CAGE] = "BLUE_LEFT_CAGE";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_RIGHT_CAGE] = "BLUE_RIGHT_CAGE";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_CENTER_CAGE] = "BLUE_CENTER_CAGE";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_SIDEWALL] = "BLUE_CORAL_STATION_LEFT_SIDEWALL";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT] = "BLUE_CORAL_STATION_LEFT";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_ALLIANCE] = "BLUE_CORAL_STATION_LEFT_ALLIANCE";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_ALLIANCE] = "BLUE_CORAL_STATION_RIGHT_ALLIANCE";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT] = "BLUE_CORAL_STATION_RIGHT";
    m_fieldConstantsNameMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_SIDEWALL] = "BLUE_CORAL_STATION_RIGHT_SIDEWALL";

    frc::DataLogManager::Start("", "field_poses.wpilog");
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();

    for (auto &[key, pose] : fieldConstantsPoseMap)
    {
        auto poseLog = wpi::log::StructLogEntry<frc::Pose3d>(log, m_fieldConstantsNameMap[key]);
        poseLog.Append(pose);
    }

    log.Flush();
}

#endif
