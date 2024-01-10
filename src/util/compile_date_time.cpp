
// program version revision
// compile_date_time.cpp

#include "compile_date_time.h"

static time_t get_build_timestamp() {
    struct tm tm_;
    tm_.tm_year = ((BUILD_YEAR_CH0 - '0') * 1000 + (BUILD_YEAR_CH1 - '0') * 100 + (BUILD_YEAR_CH2 - '0') * 10 + (BUILD_YEAR_CH3 - '0')) - 1900;
    tm_.tm_mon = ((BUILD_MONTH_CH0 - '0') * 10 + (BUILD_MONTH_CH1 - '0')) - 1;
    tm_.tm_mday = (BUILD_DAY_CH0 - '0') * 10 + (BUILD_DAY_CH1 - '0');
    tm_.tm_hour = (BUILD_HOUR_CH0 - '0') * 10 + (BUILD_HOUR_CH1 - '0');
    tm_.tm_min = (BUILD_MIN_CH0 - '0') * 10 + (BUILD_MIN_CH1 - '0');
    tm_.tm_sec = (BUILD_SEC_CH0 - '0') * 10 + (BUILD_SEC_CH1 - '0');
    tm_.tm_isdst = 0;
    return mktime(&tm_);
}

const char g_build_date_time[] = BUILD_DATE_TIME;
time_t g_build_timestamp = get_build_timestamp();