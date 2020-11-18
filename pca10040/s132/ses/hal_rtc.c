#include "rtc.h"
#include <time.h>

volatile char rtc_buf[80];

// Handle every one second
void rtc_handle(struct tm *rtc_collector) {
  rtc_collector->tm_sec++;
  mktime(rtc_collector);
  strftime(rtc_buf, 80, "%x;%X", &rtc_tm);
}



