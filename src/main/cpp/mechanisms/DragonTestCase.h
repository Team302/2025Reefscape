#pragma once

#include <string>

using std::string;

class DragonTestCase
{
public:
    DragonTestCase(string testSuiteName, string testCaseName);
    virtual void SetUp() = 0;
    virtual bool Run() = 0;
    virtual void CompareAndReport() = 0;

private:
    string m_testCaseName;
};