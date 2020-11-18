#ifndef RTC_H
#define RTC_H

#include <stdint.h>
#include <time.h>

#define  DEBUG_SHOW_TIME    false

#define RTC_TEST_PIN        (12)

#define RTC_DEFAULT_YEAR    (2020-1900)
#define RTC_DEFAULT_MONTH   (11-1)
#define RTC_DEFAULT_DAY     (9)
#define RTC_DEFAULT_HOUR    (11)
#define RTC_DEFAULT_MIN     (20)
#define RTC_DEFAULT_S       (0)

#define RTC_TICK_MS_TO_S    (8)

void init_rtc(void);
void show_rtc(void);
void rtc_client_handle(uint8_t *packet);
void rtc_handle(void);

#endif /* RTC_H */
