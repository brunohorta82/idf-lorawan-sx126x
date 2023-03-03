#include "timer.h"
#include "board.h"

Ticker timerTickers[10];
uint32_t timerTimes[10];
bool timerInUse[10] = {false, false, false, false, false, false, false, false, false, false};

// External functions
unsigned long millis()
{
	return (unsigned long)(esp_timer_get_time() / 1000ULL);
}
void TimerConfig(void)
{
	/// \todo Nothing to do here for ESP32
}

void TimerInit(TimerEvent_t *obj, void (*callback)(void))
{
	// Look for an available Ticker
	for (int idx = 0; idx < 10; idx++)
	{
		if (timerInUse[idx] == false)
		{
			timerInUse[idx] = true;
			obj->timerNum = idx;
			obj->Callback = callback;
			return;
		}
	}
	ESP_LOGI("TIM", "No more timers available!");
	/// \todo We run out of tickers, what do we do now???
}

void timerCallback(TimerEvent_t *obj)
{
	// Nothing to do here for the ESP32
}

void TimerStart(TimerEvent_t *obj)
{
	int idx = obj->timerNum;
	if (obj->oneShot)
	{
		timerTickers[idx].once_ms(timerTimes[idx], obj->Callback);
	}
	else
	{
		timerTickers[idx].attach_ms(timerTimes[idx], obj->Callback);
	}
}

void TimerStop(TimerEvent_t *obj)
{
	int idx = obj->timerNum;
	timerTickers[idx].detach();
}

void TimerReset(TimerEvent_t *obj)
{
	int idx = obj->timerNum;
	timerTickers[idx].detach();
	if (obj->oneShot)
	{
		timerTickers[idx].once_ms(timerTimes[idx], obj->Callback);
	}
	else
	{
		timerTickers[idx].attach_ms(timerTimes[idx], obj->Callback);
	}
}

void TimerSetValue(TimerEvent_t *obj, uint32_t value)
{
	int idx = obj->timerNum;
	timerTimes[idx] = value;
}

TimerTime_t TimerGetCurrentTime(void)
{
	return millis();
}

TimerTime_t TimerGetElapsedTime(TimerTime_t past)
{
	uint32_t nowInTicks = millis();
	uint32_t pastInTicks = past;
	TimerTime_t diff = nowInTicks - pastInTicks;

	return diff;
}
