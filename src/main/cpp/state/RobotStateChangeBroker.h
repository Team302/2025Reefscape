
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

#include <vector>
#include <state/RobotStateChanges.h>
#include <state/IRobotStateChangeSubscriber.h>

class RobotStateChangeBroker
{
public:
	RobotStateChangeBroker() = delete;
	RobotStateChangeBroker(RobotStateChanges::StateChange change);
	~RobotStateChangeBroker() = default;

	void AddSubscriber(IRobotStateChangeSubscriber *subscriber);

	void Notify(int value);
	void Notify(double value);
	void Notify(units::length::meter_t value);
	void Notify(units::angle::degree_t value);
	void Notify(units::velocity::meters_per_second_t value);
	void Notify(units::angular_velocity::degrees_per_second_t value);
	void Notify(frc::Pose2d value);

private:
	RobotStateChanges::StateChange m_change;
	std::vector<IRobotStateChangeSubscriber *> m_subscribers;
};