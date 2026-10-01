/*
 * scheduler.h
 *
 *  Created on: Jul 21, 2026
 *      Author: anonymous
 */

#ifndef INC_FC_SCHEDULER_H_
#define INC_FC_SCHEDULER_H_

#include "types.h"

#ifndef FC_SCHEDULER_ENABLE_STATS
#define FC_SCHEDULER_ENABLE_STATS 1
#endif

extern "C" uint32_t MicrosecondClock_Now(void);

inline uint32_t HAL_GetMicros()
{
    return MicrosecondClock_Now();
}

class IScheduledTask
{
public:
	virtual ~IScheduledTask() = default;

	virtual void update(Microseconds dt) = 0;
};

template<size_t MaxTasks>
class Scheduler
{
public:

    using TaskID = int;

    struct SchedulerFault
    {
        TaskID task;

        Microseconds budget;
        Microseconds runtime;
        Microseconds overrun;
    };

    using FaultCallback = void (*)(void* context, const SchedulerFault&);

private:

    struct Entry
    {
        IScheduledTask* task = nullptr;

        Microseconds period{0};
        Microseconds nextRun{0};

        Microseconds lastRun{0};
        Microseconds dt{0};

        bool enabled = false;

        bool faulted = false;

#if FC_SCHEDULER_ENABLE_STATS

        uint32_t executions = 0;
        uint32_t overruns = 0;

        uint32_t lastRuntime_us = 0;
        uint32_t maxRuntime_us = 0;

#endif
    };

public:

    Scheduler() = default;

    TaskID add(
        IScheduledTask& task,
		Microseconds period,
		Microseconds startDelay = Microseconds{0})
    {
    	for(size_t i = 0; i < MaxTasks; i++)
		{
			if(entries[i].task == nullptr)
			{
				entries[i].task = &task;

				entries[i].period = period;

				const Microseconds now(HAL_GetMicros());

				entries[i].lastRun = now;
				entries[i].nextRun = now + startDelay;

				entries[i].dt = Microseconds{0};

				entries[i].enabled = true;

				return static_cast<TaskID>(i);
			}
		}

        return -1;
    }

    bool remove(TaskID id)
    {
        if(id < 0 || id >= static_cast<TaskID>(MaxTasks))
            return false;

        entries[id] = Entry();

        return true;
    }

    bool enable(TaskID id)
    {
        if(!valid(id))
            return false;

        entries[id].enabled = true;

        return true;
    }

    bool disable(TaskID id)
    {
        if(!valid(id))
            return false;

        entries[id].enabled = false;

        return true;
    }

    void tick()
    {
        const Microseconds now(HAL_GetMicros());

        for(size_t i = 0; i < MaxTasks; i++)
        {
            Entry& e = entries[i];

            if(e.task == nullptr)
                continue;

            if(!e.enabled)
                continue;

            if(static_cast<int32_t>(
                    (now - e.nextRun).value()) < 0)
                continue;

            e.dt = now - e.lastRun;
            e.lastRun = now;

#if FC_SCHEDULER_ENABLE_STATS

            uint32_t start = DWT->CYCCNT;

#endif

            e.task->update(e.dt);

#if FC_SCHEDULER_ENABLE_STATS

            uint32_t cycles = DWT->CYCCNT - start;

            uint32_t runtime =
                cycles / (SystemCoreClock / 1000000);

            Microseconds runtime_us(runtime);

            if(runtime_us > e.period)
            {
                if(!e.faulted)
                {
                    e.faulted = true;

                    if(faultCallback)
                    {
                        SchedulerFault fault = SchedulerFault{
                            .task = static_cast<TaskID>(i),
                            .budget = e.period,
                            .runtime = runtime_us,
                            .overrun = runtime_us - e.period
                        };

                        faultCallback(faultContext, fault);
                    }
                }
            }

            e.lastRuntime_us = runtime;

            if(runtime > e.maxRuntime_us)
                e.maxRuntime_us = runtime;

            e.executions++;

#endif

            Microseconds lateness = now - e.nextRun;

            if(lateness >= e.period)
            {
#if FC_SCHEDULER_ENABLE_STATS
                e.overruns++;
#endif
                e.nextRun = now + e.period;
            }
            else
            {
                e.nextRun += e.period;
            }
        }
    }

#if FC_SCHEDULER_ENABLE_STATS

    struct Stats
    {
        uint32_t executions;

        uint32_t overruns;

        uint32_t lastRuntime_us;

        uint32_t maxRuntime_us;
    };

    bool statistics(TaskID id, Stats& out)
    {
        if(!valid(id))
            return false;

        Entry& e = entries[id];

        out.executions = e.executions;
        out.overruns = e.overruns;
        out.lastRuntime_us = e.lastRuntime_us;
        out.maxRuntime_us = e.maxRuntime_us;

        return true;
    }

#endif

    void onFault(
        void* context,
        FaultCallback callback)
    {
        faultContext = context;
        faultCallback = callback;
    }

private:

    bool valid(TaskID id)
    {
        return
            id >= 0 &&
            id < static_cast<TaskID>(MaxTasks) &&
            entries[id].task != nullptr;
    }

    Entry entries[MaxTasks];

    FaultCallback faultCallback = nullptr;
    void* faultContext = nullptr;
};


#endif /* INC_FC_SCHEDULER_H_ */
