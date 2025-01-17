#include "DragonTestCase.h"
#include "DragonTestSuiteManager.h"

DragonTestCase::DragonTestCase(string testSuiteName, string testCaseName) : m_testCaseName(testCaseName)
{
    DragonTestSuiteManager::GetInstance()->RegisterTest(testSuiteName, this);
}