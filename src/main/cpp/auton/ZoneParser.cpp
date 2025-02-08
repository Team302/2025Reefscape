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

#include "frc/Filesystem.h"

#include "auton/AutonGrid.h"
#include "auton/ZoneParams.h"
#include "auton/ZoneParser.h"
#include "utils/logging/Logger.h"
#include "mechanisms\DragonTale\DragonTale.h"
#include "mechanisms\IntakeManager\IntakeManager.h"

#include "pugixml/pugixml.hpp"

using namespace std;
using namespace pugi;

ZoneParams *ZoneParser::ParseXML(string fulldirfile)
{
    auto hasError = false;

    static std::map<std::string, AutonGrid::XGRID> X_xmlStringToGridEnumMap{
        {"1", AutonGrid::XGRID::X_1},
        {"2", AutonGrid::XGRID::X_2},
        {"3", AutonGrid::XGRID::X_3},
        {"4", AutonGrid::XGRID::X_4},
        {"5", AutonGrid::XGRID::X_5},
        {"6", AutonGrid::XGRID::X_6},
        {"7", AutonGrid::XGRID::X_7},
        {"8", AutonGrid::XGRID::X_8},
        {"9", AutonGrid::XGRID::X_9},
        {"10", AutonGrid::XGRID::X_10},
        {"11", AutonGrid::XGRID::X_11},
        {"12", AutonGrid::XGRID::X_12},
        {"13", AutonGrid::XGRID::X_13},
        {"14", AutonGrid::XGRID::X_14},
        {"15", AutonGrid::XGRID::X_15},
        {"16", AutonGrid::XGRID::X_16},
        {"17", AutonGrid::XGRID::X_17},
        {"18", AutonGrid::XGRID::X_18},
        {"19", AutonGrid::XGRID::X_19},
        {"20", AutonGrid::XGRID::X_20},
        {"21", AutonGrid::XGRID::X_21},
        {"22", AutonGrid::XGRID::X_22},
        {"23", AutonGrid::XGRID::X_23},
        {"24", AutonGrid::XGRID::X_24},
        {"25", AutonGrid::XGRID::X_25},
        {"26", AutonGrid::XGRID::X_26},
        {"27", AutonGrid::XGRID::X_27},
        {"28", AutonGrid::XGRID::X_28},
        {"29", AutonGrid::XGRID::X_29},
        {"30", AutonGrid::XGRID::X_30},
        {"31", AutonGrid::XGRID::X_31},
        {"32", AutonGrid::XGRID::X_32},
        {"33", AutonGrid::XGRID::X_33},
        {"34", AutonGrid::XGRID::X_34},
        {"35", AutonGrid::XGRID::X_35},
        {"36", AutonGrid::XGRID::X_36},
        {"37", AutonGrid::XGRID::X_37},
        {"38", AutonGrid::XGRID::X_38},
        {"39", AutonGrid::XGRID::X_39},
        {"40", AutonGrid::XGRID::X_40},
        {"41", AutonGrid::XGRID::X_41},
        {"42", AutonGrid::XGRID::X_42},
        {"43", AutonGrid::XGRID::X_43},
        {"44", AutonGrid::XGRID::X_44},
        {"45", AutonGrid::XGRID::X_45},
        {"46", AutonGrid::XGRID::X_46},
        {"47", AutonGrid::XGRID::X_47},
        {"48", AutonGrid::XGRID::X_48},
        {"49", AutonGrid::XGRID::X_49},
        {"50", AutonGrid::XGRID::X_50},
        {"51", AutonGrid::XGRID::X_51},
        {"52", AutonGrid::XGRID::X_52},
        {"53", AutonGrid::XGRID::X_53},
        {"54", AutonGrid::XGRID::X_54}}; // 1-54
    static std::map<std::string, AutonGrid::YGRID> Y_xmlStringToGridEnumMap{
        {"1", AutonGrid::YGRID::Y_1},
        {"2", AutonGrid::YGRID::Y_2},
        {"3", AutonGrid::YGRID::Y_3},
        {"4", AutonGrid::YGRID::Y_4},
        {"5", AutonGrid::YGRID::Y_5},
        {"6", AutonGrid::YGRID::Y_6},
        {"7", AutonGrid::YGRID::Y_7},
        {"8", AutonGrid::YGRID::Y_8},
        {"9", AutonGrid::YGRID::Y_9},
        {"10", AutonGrid::YGRID::Y_10},
        {"11", AutonGrid::YGRID::Y_11},
        {"12", AutonGrid::YGRID::Y_12},
        {"13", AutonGrid::YGRID::Y_13},
        {"14", AutonGrid::YGRID::Y_14},
        {"15", AutonGrid::YGRID::Y_15},
        {"16", AutonGrid::YGRID::Y_16},
        {"17", AutonGrid::YGRID::Y_17},
        {"18", AutonGrid::YGRID::Y_18},
        {"19", AutonGrid::YGRID::Y_19},
        {"20", AutonGrid::YGRID::Y_20},
        {"21", AutonGrid::YGRID::Y_21},
        {"22", AutonGrid::YGRID::Y_22},
        {"23", AutonGrid::YGRID::Y_23},
        {"24", AutonGrid::YGRID::Y_24},
        {"25", AutonGrid::YGRID::Y_25},
        {"26", AutonGrid::YGRID::Y_26},
        {"27", AutonGrid::YGRID::Y_27}};

    static std::map<std::string, ChassisOptionEnums::AutonChassisOptions> xmlStringToChassisOptionEnumMap{
        {"VISION_DRIVE_NOTE", ChassisOptionEnums::AutonChassisOptions::VISION_DRIVE_NOTE},
        {"VISION_DRIVE_SPEAKER", ChassisOptionEnums::AutonChassisOptions::VISION_DRIVE_SPEAKER},
        {"NO_VISION", ChassisOptionEnums::AutonChassisOptions::NO_VISION},
    };
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

            AutonGrid::XGRID xgrid1 = AutonGrid::XGRID::NO_VALUE;
            AutonGrid::YGRID ygrid1 = AutonGrid::YGRID::NONE;
            AutonGrid::XGRID xgrid2 = AutonGrid::XGRID::NO_VALUE;
            AutonGrid::YGRID ygrid2 = AutonGrid::YGRID::NONE;

            AutonGrid::ZoneMode zoneMode = AutonGrid::ZoneMode::NOTHING;

            ChassisOptionEnums::AutonChassisOptions chassisChosenOption = ChassisOptionEnums::AutonChassisOptions::NO_VISION;
            bool isTaleStateChanging = false;
            bool isIntakeStateChanging = false;
            DragonTale::STATE_NAMES taleChosenOption = DragonTale::STATE_NAMES::STATE_READY;
            IntakeManager::STATE_NAMES intakeChosenOption = IntakeManager::STATE_NAMES::STATE_OFF;
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
                        zoneMode = AutonGrid::RECTANGLE;
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
                    zoneMode = AutonGrid::CIRCLE;
                    circleX = attr.as_double();
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "parsed circlex", circleX);
                }
                else if (strcmp(attr.name(), "circley") == 0)
                {

                    zoneMode = AutonGrid::CIRCLE;
                    circleY = attr.as_double();
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "parsed circley", circleX);
                }
                else if (strcmp(attr.name(), "radius") == 0)
                {
                    radius = attr.as_double();
                }

                else if (strcmp(attr.name(), "intakeOption") == 0)
                {
                    auto itr = IntakeManager::stringToSTATE_NAMESEnumMap.find(attr.value());
                    if (config != nullptr && config->GetMechanism(MechanismTypes::INTAKE_MANAGER) != nullptr)
                    {

                        if (itr != IntakeManager::stringToSTATE_NAMESEnumMap.end())
                        {
                            intakeChosenOption = itr->second;
                            isIntakeStateChanging = false;
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
                            isTaleStateChanging = false;
                        }
                    }
                    else
                    {
                        hasError = true;
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
                                       isTaleStateChanging,
                                       isIntakeStateChanging,
                                       intakeChosenOption,
                                       taleChosenOption,
                                       chassisChosenOption,
                                       avoidChosenOption,
                                       zoneMode));
            }

            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("ZoneParser"), string("ParseXML"), string("Has Error"));
        }
    }
    return nullptr; // if error, return nullptr
}
