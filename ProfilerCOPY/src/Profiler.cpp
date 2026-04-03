#include "Profiler.h"
#include "Internal.h"

// Roman's additional includes
#include <cstdint>
#include <string>

//=============================================================================
// CONFIGURATION
//=============================================================================
static constexpr size_t INITIAL_EVENT_CAPACITY = 1024 * 1024;  // 1M events (~24MB)
//static constexpr size_t EVENT_GROWTH_FACTOR = 2;

Profiler* Profiler::m_instance = nullptr;
Profiler::Profiler()
    : m_events()
    , m_eventCount(0)
{
    m_events.resize(INITIAL_EVENT_CAPACITY);
}

//=============================================================================
// POST-PROCESSING STRUCTURES (Used only after profiling ends)
//=============================================================================

// Statistics per function address
//struct FunctionStats
//{
//    std::string name;
//    std::string module;
//    void* address = nullptr;
//    uint64_t    callCount = 0;
//    uint64_t    inclusiveCycles = 0;  // Time in function INCLUDING children
//    uint64_t    exclusiveCycles = 0;  // Time in function EXCLUDING children
//};

//// For reconstructing call tree during post-processing
//struct CallTreeNode
//{
//    void* address = nullptr;
//    uint64_t    startTime = 0;
//    uint64_t    endTime = 0;
//    uint64_t    childTime = 0;  // Accumulated time spent in children
//    CallTreeNode* parent = nullptr;
//    std::vector<CallTreeNode*> children;
//};

void Profiler::Enter(void* address, uint64_t timestamp)
{
    ProfileEvent& evt = m_events[m_eventCount++];
    evt.timestamp = timestamp;
    evt.address = address;
    evt.isExit = false;
}

void Profiler::Exit(void* address, uint64_t timestamp)
{
    ProfileEvent& evt = m_events[m_eventCount++];
    evt.timestamp = timestamp;
    evt.address = address;
    evt.isExit = true;
}

//=============================================================================
// INSTRUMENTATION HOOKS (Called by compiler-injected code)
//=============================================================================

#include <intrin.h> /*__rdtsc()*/

extern "C"
void EnterFunc(void* address)
{
    Profiler::GetProfiler()->Enter(address, __rdtsc());
}

extern "C"
void ExitFunc(void* address)
{
    Profiler::GetProfiler()->Exit(address, __rdtsc());
}
