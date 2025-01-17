#include "DragonTestSuiteManager.h"

DragonTestSuiteManager *DragonTestSuiteManager::m_instance = nullptr;

DragonTestSuiteManager *DragonTestSuiteManager::GetInstance()
{
    if (DragonTestSuiteManager::m_instance == nullptr)
    {
        DragonTestSuiteManager::m_instance = new DragonTestSuiteManager();
    }
    return DragonTestSuiteManager::m_instance;
}

DragonTestSuiteManager::DragonTestSuiteManager() : m_currTestSlot(0), m_currTestSuiteIndex(0), m_currTest(nullptr) {}

// assumes we will run through all tests
void DragonTestSuiteManager::Init()
{
    // initialize from first test suite and case
    m_currTestSlot = 0;
    m_currTestSuiteIndex = 0;

    GetNextTest();
}

void DragonTestSuiteManager::RegisterTest(string testSuiteName, DragonTestCase *tc)
{
    // if test suite already exists, add test case
    if (m_testSuites.count(testSuiteName) > 0)
    {
        m_testSuites.at(testSuiteName).emplace_back(tc);
    }
    // otherwise make new vector
    else
    {
        m_testSuites.emplace(std::vector<DragonTestCase *>{tc});
        m_testSuiteNames.emplace_back(testSuiteName);
    }
}

void DragonTestSuiteManager::Run()
{
    // return early if no test available
    if (m_currTest == nullptr)
        return;

    if (m_currTest->Run())
    {
        m_currTest->CompareAndReport();
        GetNextTest();
    }
}

void DragonTestSuiteManager::GetNextTest()
{
    // if all test suites have been iterated through, return early
    if (m_currTestSuiteIndex >= (int)m_testSuiteNames.size())
    {
        m_currTest = nullptr;
        return;
    }

    std::vector testCases = m_testSuites[m_testSuiteNames[m_currTestSuiteIndex]];
    if (m_currTestSlot >= (int)testCases.size())
    {
        m_currTestSlot = 0;
        m_currTestSuiteIndex++;

        if (m_currTestSuiteIndex >= (int)m_testSuiteNames.size())
        {
            m_currTest = nullptr;
            return;
        }

        testCases = m_testSuites[m_testSuiteNames[m_currTestSuiteIndex]];
    }

    m_currTest = testCases[m_currTestSlot];
    m_currTest->SetUp();
    m_currTestSlot++;
}