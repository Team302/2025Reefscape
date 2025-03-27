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

#include <map>
#include <string>

#include "auton/AutonGrid.h"
#include "auton/ZoneParams.h"
#include "auton/ZoneParser.h"
#include "frc/Filesystem.h"
#include "mechanisms/DragonTale/DragonTale.h"
#include "mechanisms/IntakeManager/IntakeManager.h"
#include "auton/PrimitiveEnums.h"
#include "pugixml/pugixml.hpp"
#include "auton/ZoneEnums.h"

#include "utils/logging/debug/Logger.h"

using namespace std;
using namespace pugi;

ZoneParams *ZoneParser::ParseXML(string fulldirfile)
{
    auto hasError = false;

    static std::map<std::string, XGRID> X_xmlStringToGridEnumMap{
        {"1", XGRID::X_1},
        {"2", XGRID::X_2},
        {"3", XGRID::X_3},
        {"4", XGRID::X_4},
        {"5", XGRID::X_5},
        {"6", XGRID::X_6},
        {"7", XGRID::X_7},
        {"8", XGRID::X_8},
        {"9", XGRID::X_9},
        {"10", XGRID::X_10},
        {"11", XGRID::X_11},
        {"12", XGRID::X_12},
        {"13", XGRID::X_13},
        {"14", XGRID::X_14},
        {"15", XGRID::X_15},
        {"16", XGRID::X_16},
        {"17", XGRID::X_17},
        {"18", XGRID::X_18},
        {"19", XGRID::X_19},
        {"20", XGRID::X_20},
        {"21", XGRID::X_21},
        {"22", XGRID::X_22},
        {"23", XGRID::X_23},
        {"24", XGRID::X_24},
        {"25", XGRID::X_25},
        {"26", XGRID::X_26},
        {"27", XGRID::X_27},
        {"28", XGRID::X_28},
        {"29", XGRID::X_29},
        {"30", XGRID::X_30},
        {"31", XGRID::X_31},
        {"32", XGRID::X_32},
        {"33", XGRID::X_33},
        {"34", XGRID::X_34},
        {"35", XGRID::X_35},
        {"36", XGRID::X_36},
        {"37", XGRID::X_37},
        {"38", XGRID::X_38},
        {"39", XGRID::X_39},
        {"40", XGRID::X_40},
        {"41", XGRID::X_41},
        {"42", XGRID::X_42},
        {"43", XGRID::X_43},
        {"44", XGRID::X_44},
        {"45", XGRID::X_45},
        {"46", XGRID::X_46},
        {"47", XGRID::X_47},
        {"48", XGRID::X_48},
        {"49", XGRID::X_49},
        {"50", XGRID::X_50},
        {"51", XGRID::X_51},
        {"52", XGRID::X_52},
        {"53", XGRID::X_53},
        {"54", XGRID::X_54}}; // 1-54
    static std::map<std::string, YGRID> Y_xmlStringToGridEnumMap{
        {"1", YGRID::Y_1},
        {"2", YGRID::Y_2},
        {"3", YGRID::Y_3},
        {"4", YGRID::Y_4},
        {"5", YGRID::Y_5},
        {"6", YGRID::Y_6},
        {"7", YGRID::Y_7},
        {"8", YGRID::Y_8},
        {"9", YGRID::Y_9},
        {"10", YGRID::Y_10},
        {"11", YGRID::Y_11},
        {"12", YGRID::Y_12},
        {"13", YGRID::Y_13},
        {"14", YGRID::Y_14},
        {"15", YGRID::Y_15},
        {"16", YGRID::Y_16},
        {"17", YGRID::Y_17},
        {"18", YGRID::Y_18},
        {"19", YGRID::Y_19},
        {"20", YGRID::Y_20},
        {"21", YGRID::Y_21},
        {"22", YGRID::Y_22},
        {"23", YGRID::Y_23},
        {"24", YGRID::Y_24},
        {"25", YGRID::Y_25},
        {"26", YGRID::Y_26},
        {"27", YGRID::Y_27}};

    static std::map<std::string, ChassisOptionEnums::AutonChassisOptions> xmlStringToChassisOptionEnumMap{
        {"VISION_DRIVE_SPEAKER", ChassisOptionEnums::AutonChassisOptions::VISION_DRIVE_SPEAKER},
        {"NO_VISION", ChassisOptionEnums::AutonChassisOptions::NO_VISION},
    };

    static std::map<std::string, ChassisOptionEnums::HeadingOption> xmlStringToHeadingOptionEnumMap{

        {"MAINTAIN", ChassisOptionEnums::HeadingOption::MAINTAIN},
        {"SPECIFIED_ANGLE", ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE},
        {"FACE_GAME_PIECE", ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE},
        {"FACE_REEF_CENTER", ChassisOptionEnums::HeadingOption::FACE_REEF_CENTER},
        {"FACE_REEF_FACE", ChassisOptionEnums::HeadingOption::FACE_REEF_FACE},
        {"FACE_CORAL_STATION", ChassisOptionEnums::HeadingOption::FACE_CORAL_STATION},
        {"IGNORE", ChassisOptionEnums::HeadingOption::IGNORE}};

    static std::map<string, ChassisOptionEnums::DriveStateType> xmlStringToPathUpdateOptionMap{{"RIGHT_REEF_BRANCH", ChassisOptionEnums::DRIVE_TO_RIGHT_REEF_BRANCH},
                                                                                               {"LEFT_REEF_BRANCH", ChassisOptionEnums::DRIVE_TO_LEFT_REEF_BRANCH},
                                                                                               //    {"REEF_ALGAE", PATH_UPDATE_OPTION::REEF_ALGAE},
                                                                                               //    {"FLOOR_ALGAE", PATH_UPDATE_OPTION::FLOOR_ALGAE},
                                                                                               {"CORAL_STATION", ChassisOptionEnums::DRIVE_TO_CORAL_STATION},
                                                                                               //    {"PROCESSOR", PATH_UPDATE_OPTION::PROCESSOR},
                                                                                               {"BARGE", ChassisOptionEnums::DRIVE_TO_BARGE},
                                                                                               {"NOTHING", ChassisOptionEnums::STOP_DRIVE}};

    static std::map<std::string, ChassisOptionEnums::AutonAvoidOptions> xmlStringToAvoidOptionEnumMap{
        {"PODIUM", ChassisOptionEnums::AutonAvoidOptions::PODIUM},
        {"ROBOT_COLLISION", ChassisOptionEnums::AutonAvoidOptions::ROBOT_COLLISION},
        {"NO_AVOID_OPTION", ChassisOptionEnums::AutonAvoidOptions::NO_AVOID_OPTION},

    };

    auto deployDir = frc::filesystem::GetDeployDirectory();
    auto zonedir = deployDir + "/auton/zones/";

    string updfulldirfile = zonedir;
    updfulldirfile += fulldirfile;

    xml_document doc;
    xml_parse_result result = doc.load_file(updfulldirfile.c_str());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PrimitiveParser", "updated File", updfulldirfile.c_str());
    if (result)
    {
        xml_node auton = doc.root();
        for (xml_node zonenode = auton.first_child().first_child(); zonenode; zonenode = zonenode.next_sibling())
        {

            double radius = -1;
            double circleX = -1;
            double circleY = -1;

            XGRID xgrid1 = XGRID::NO_VALUE;
            YGRID ygrid1 = YGRID::NONE;
            XGRID xgrid2 = XGRID::NO_VALUE;
            YGRID ygrid2 = YGRID::NONE;

            units::length::meter_t xgrid1rect = units::length::meter_t(0.0);
            units::length::meter_t ygrid1rect = units::length::meter_t(0.0);
            units::length::meter_t xgrid2rect = units::length::meter_t(0.0);
            units::length::meter_t ygrid2rect = units::length::meter_t(0.0);

            ZoneMode zoneMode = ZoneMode::NOTHING;

            // TODO: add zoneType parsing and check

            ChassisOptionEnums::AutonChassisOptions chassisChosenOption = ChassisOptionEnums::AutonChassisOptions::NO_VISION;
            bool isTaleStateChanging = false;
            bool isIntakeStateChanging = false;
            DragonTale::STATE_NAMES taleChosenOption = DragonTale::STATE_NAMES::STATE_READY;
            IntakeManager::STATE_NAMES intakeChosenOption = IntakeManager::STATE_NAMES::STATE_OFF;
            ChassisOptionEnums::HeadingOption chosenHeadingOption = ChassisOptionEnums::HeadingOption::IGNORE;

            ChassisOptionEnums::DriveStateType chosenUpdateOption = ChassisOptionEnums::STOP_DRIVE;
            ChassisOptionEnums::AutonAvoidOptions avoidChosenOption = ChassisOptionEnums::AutonAvoidOptions::NO_AVOID_OPTION;

            auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();

            // looping through the zone xml attributes to define the location of a given zone (based on 2 sets grid coordinates)
            for (xml_attribute attr = zonenode.first_attribute(); attr; attr = attr.next_attribute())
            {

                if (strcmp(attr.name(), "xgrid1") == 0)
                {
                    auto itr = X_xmlStringToGridEnumMap.find(attr.value());
                    if (itr != X_xmlStringToGridEnumMap.end())
                    {
                        xgrid1 = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "ygrid1") == 0)
                {
                    auto itr = Y_xmlStringToGridEnumMap.find(attr.value());
                    if (itr != Y_xmlStringToGridEnumMap.end())
                    {
                        ygrid1 = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "xgrid2") == 0)
                {
                    auto itr = X_xmlStringToGridEnumMap.find(attr.value());
                    if (itr != X_xmlStringToGridEnumMap.end())
                    {
                        xgrid2 = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "ygrid2") == 0)
                {
                    auto itr = Y_xmlStringToGridEnumMap.find(attr.value());
                    if (itr != Y_xmlStringToGridEnumMap.end())
                    {
                        ygrid2 = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "circlex") == 0)
                {
                    zoneMode = ZoneMode::CIRCLE;
                    circleX = attr.as_double();
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "parsed circlex", circleX);
                }
                else if (strcmp(attr.name(), "circley") == 0)
                {

                    zoneMode = ZoneMode::CIRCLE;
                    circleY = attr.as_double();
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "parsed circley", circleX);
                }
                else if (strcmp(attr.name(), "radius") == 0)
                {
                    radius = attr.as_double();
                }
                if (strcmp(attr.name(), "x1_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    xgrid1rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "y1_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    ygrid1rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "x2_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    xgrid2rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "y2_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    ygrid2rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "intakeOption") == 0)
                {
                    auto itr = IntakeManager::stringToSTATE_NAMESEnumMap.find(attr.value());
                    if (config != nullptr && config->GetMechanism(MechanismTypes::INTAKE_MANAGER) != nullptr)
                    {

                        if (itr != IntakeManager::stringToSTATE_NAMESEnumMap.end())
                        {
                            intakeChosenOption = itr->second;
                            isIntakeStateChanging = true;
                        }
                        else
                        {
                            hasError = true;
                        }
                    }
                }
                else if (strcmp(attr.name(), "taleOption") == 0)
                {
                    auto itr = DragonTale::stringToSTATE_NAMESEnumMap.find(attr.value());
                    if (config != nullptr && config->GetMechanism(MechanismTypes::DRAGON_TALE) != nullptr)
                    {
                        if (itr != DragonTale::stringToSTATE_NAMESEnumMap.end())
                        {
                            taleChosenOption = itr->second;
                            isTaleStateChanging = true;
                        }
                        else
                        {
                            hasError = true;
                        }
                    }
                }

                else if (strcmp(attr.name(), "chassisOption") == 0)
                {
                    auto itr = xmlStringToChassisOptionEnumMap.find(attr.value());
                    if (itr != xmlStringToChassisOptionEnumMap.end())
                    {
                        chassisChosenOption = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }

                else if (strcmp(attr.name(), "headingOption") == 0)
                {
                    auto itr = xmlStringToHeadingOptionEnumMap.find(attr.value());
                    if (itr != xmlStringToHeadingOptionEnumMap.end())
                    {
                        chosenHeadingOption = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "pathUpdateOption") == 0)
                {
                    auto itr = xmlStringToPathUpdateOptionMap.find(attr.value());
                    if (itr != xmlStringToPathUpdateOptionMap.end())
                    {
                        chosenUpdateOption = itr->second;
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "Update Option Parsed", chosenUpdateOption);
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "avoidOption") == 0)
                {
                    auto itr = xmlStringToAvoidOptionEnumMap.find(attr.value());
                    if (itr != xmlStringToAvoidOptionEnumMap.end())
                    {
                        avoidChosenOption = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }
            }

            if (!hasError) // if no error returns the zone parameters
            {

                auto circlePose2d = frc::Pose2d(units::length::meter_t(circleX), units::length::meter_t(circleY), units::degree_t(0));
                return (new ZoneParams(xgrid1,
                                       ygrid1,
                                       xgrid2,
                                       ygrid2,
                                       circlePose2d,
                                       units::inch_t(radius),
                                       xgrid1rect,
                                       xgrid2rect,
                                       ygrid1rect,
                                       ygrid2rect,
                                       isTaleStateChanging,
                                       isIntakeStateChanging,
                                       intakeChosenOption,
                                       taleChosenOption,
                                       chassisChosenOption,
                                       chosenHeadingOption,
                                       chosenUpdateOption,
                                       avoidChosenOption,
                                       zoneMode));
            }

            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("ZoneParser"), string("ParseXML"), string("Has Error"));
        }
    }
    return nullptr; // if error, return nullptr
}
