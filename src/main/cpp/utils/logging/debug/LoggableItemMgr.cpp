//====================================================================================================================================================
/// Copyright 2025 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
#include <vector>

// FRC includes

// Team 302 includes
#include "utils/logging/debug/LoggableItemMgr.h"
#include "utils/logging/debug/LoggableItem.h"

// Third Party Includes

LoggableItemMgr *LoggableItemMgr::m_instance = nullptr;
LoggableItemMgr *LoggableItemMgr::GetInstance()
{
    if (LoggableItemMgr::m_instance == nullptr)
    {
        LoggableItemMgr::m_instance = new LoggableItemMgr();
    }
    return LoggableItemMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
LoggableItemMgr::LoggableItemMgr() : m_loggableItems()
{
}
void LoggableItemMgr::RegisterLoggableItem(
    LoggableItem *item)
{
    m_loggableItems.emplace_back(item);
}
void LoggableItemMgr::LogData() const
{
    for (auto item : m_loggableItems)
    {
        item->LogInformation();
    }
}