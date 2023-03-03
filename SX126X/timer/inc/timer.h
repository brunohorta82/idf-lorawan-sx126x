#ifndef __TIMER_H__
#define __TIMER_H__

#include "stdbool.h"
#include "stdint.h"
#include <Ticker.h>
#define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))
#define delayMicroseconds(us) esp_rom_delay_us(us)
#define delay(ms) esp_rom_delay_us(ms * 1000)
unsigned long millis();
typedef void (*callbackType)(void);

/**@brief Timer object description
 */
typedef struct TimerEvent_s
{
  uint8_t timerNum;             /**< Used with ESP32 MCU 1 for TX, 2 for RX*/
  bool oneShot = true;          /**< True if it is a one shot timer */
  uint32_t Timestamp;           /**< Current timer value */
  uint32_t ReloadValue = 10000; /**< Timer delay value	*/
  bool IsRunning;               /**< Is the timer currently running	*/
  void (*Callback)(void);       /**< Timer IRQ callback function	*/
  struct TimerEvent_s *Next;    /**< Pointer to the next Timer object.	*/
} TimerEvent_t;

/**@brief Timer time variable definition
 */
#ifndef TimerTime_t
typedef uint32_t TimerTime_t;
#endif

/**@brief Initializes the RTC2 timer
 *
 * @details Set prescaler to 31 in order to have Fs=1kHz
 *			Enable CC interrupt
 *			Start RTC2
 */
void TimerConfig(void);

/**@brief Initializes the timer object
 *
 * @remark TimerSetValue function must be called before starting the timer.
 *         this function initializes timestamp and reload value at 0.
 *
 * @param  obj          Structure containing the timer object parameters
 * @param  callback     Function callback called at the end of the timeout
 */
void TimerInit(TimerEvent_t *obj, void (*callback)(void));

/**@brief Starts and adds the timer object to the list of timer events
 *
 * @param  obj Structure containing the timer object parameters
 */
void TimerStart(TimerEvent_t *obj);

/**@brief Stops and removes the timer object from the list of timer events
 *
 * @param  obj Structure containing the timer object parameters
 */
void TimerStop(TimerEvent_t *obj);

/**@brief Resets the timer object
 *
 * @param  obj Structure containing the timer object parameters
 */
void TimerReset(TimerEvent_t *obj);

/**@brief Set timer new timeout value
 *
 * @param  obj   Structure containing the timer object parameters
 *
 * @param  value New timer timeout value in ms
 */
void TimerSetValue(TimerEvent_t *obj, uint32_t value);

/**@brief Return the Time elapsed since a fix moment in Time
 *
 * @param  savedTime    fix moment in Time
 *
 * @retval time             returns elapsed time in ms
 */
TimerTime_t TimerGetElapsedTime(TimerTime_t savedTime);

/**@brief Read the current time ellapsed since the start (or restart) of RTC2
 *
 * @retval current time in ms
 */
TimerTime_t TimerGetCurrentTime(void);

void TimerHandleEvents(void);

#endif // __TIMER_H__
