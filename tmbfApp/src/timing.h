static inline double toc(const struct timespec *timing_start)
{
    struct timespec duration;
    clock_gettime(CLOCK_MONOTONIC, &duration);
    duration.tv_sec  -= timing_start->tv_sec;
    duration.tv_nsec -= timing_start->tv_nsec;
    if (duration.tv_nsec < 0)
    {
        duration.tv_nsec += 1000000000;
        duration.tv_sec -= 1;
    }
    return duration.tv_sec + 1e-9 * duration.tv_nsec;
}

#define TIC() \
    struct timespec timing_start; \
    clock_gettime(CLOCK_MONOTONIC, &timing_start);
#define TOC() \
    toc(&timing_start)
