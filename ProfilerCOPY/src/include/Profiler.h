#pragma once

#include "Profiler_API.h"
#include <vector>

// Public API for the Profiler library
//  - Do not put anything in this file that is not intended to be used by external code (e.g. the Testbed or other applications using the Profiler library)

class Profiler
{
	//=============================================================================
	// EVENT BUFFER STRUCTURE (Minimal overhead during profiling)
	//=============================================================================

	// Compact event - only 24 bytes per function call (enter or exit)
	struct ProfileEvent
	{
		uint64_t timestamp;
		void* address;
		// High bit of address encodes enter(0)/exit(1) - but addresses are typically 
		// 48-bit on x64, so we use a separate flag for clarity in teaching
		bool     isExit;
		uint8_t  padding[7]; // Align to 24 bytes for cache efficiency
	};

public:
	Profiler();
	~Profiler() = default;
	Profiler(Profiler const& rhs) = delete;
	Profiler& operator=(Profiler const& rhs) = delete;

	void Enter(void* address, uint64_t timestamp);
	void Exit(void* address, uint64_t timestamp);

private:
	std::vector<ProfileEvent> m_events;
	unsigned m_eventCount;

public:
	static Profiler* GetProfiler() {
		if (!m_instance) {
			m_instance = new Profiler;
		}
		return m_instance;
	}
	static Profiler* m_instance;
};
