#include "common/profiler.h"

#include <chrono>
#include <ratio>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace common {
namespace {

#ifdef ENABLE_TIMERS

using std::chrono::high_resolution_clock;
using std::this_thread::sleep_for;
using std::vector;

// Simply confirm that the clock is giving us the resolution we want (ns).
GTEST_TEST(ProfilerTest, ConfirmHighResolution) {
  // The period should be 1 / 1e9 (i.e. nanosecond resolution).
  EXPECT_EQ(1000000000, high_resolution_clock::period::den);
}

// Tests the underlying Timer class.
GTEST_TEST(ProfilerTest, Timer) {
  Timer t;
  const std::chrono::duration<double, std::milli> sleep_time(250.);
  t.start();

  sleep_for(sleep_time);
  const double elapsed = t.elapsed<std::milli>();
  EXPECT_GE(elapsed, sleep_time.count());
  // This may be a fragile test; sleeping only guarantees that the thread will
  // be asleep *at least* the sleep time. I'm assuming it wont' double up.
  EXPECT_LT(elapsed, sleep_time.count() * 2);
}

// Tests the lap timer's behavior in tracking laps, reporting lap durations,
// and the average, all with common units.
GTEST_TEST(ProfilerTest, LapTimer) {
  const std::chrono::duration<double, std::milli> sleep_time(250.);
  const int kLapCount = 10;
  vector<double> lap_times(10, -1.);
  LapTimer timer;
  timer.start();
  for (int i = 0; i < kLapCount; ++i) {
    sleep_for(sleep_time);
    lap_times[i] = timer.lap<std::milli>();
  }
  EXPECT_EQ(kLapCount, timer.laps());
  double expected_mean_ms = 0;
  for (double t : lap_times) expected_mean_ms += t;
  expected_mean_ms /= kLapCount;
  EXPECT_NEAR(expected_mean_ms, timer.average<std::milli>(), 1);
  sleep_for(sleep_time);
  // Delaying an arbitrary amount of time *after* the last lap, will not change
  // the mean lap duration.
  EXPECT_NEAR(expected_mean_ms, timer.average<std::milli>(), 1);

  EXPECT_NEAR(expected_mean_ms * 1000, timer.average<std::micro>(), 1);
  EXPECT_NEAR(expected_mean_ms * 1000000, timer.average<std::nano>(), 1);
  EXPECT_NEAR(expected_mean_ms / 10, timer.average<std::centi>(), 1);
  // Note: the gtest macro does *not* deal well with the nested templated types.
  // This is why we evaluate mean seconds outside of the EXPECT_EQ.
  const double mean_seconds = timer.average<std::ratio<1, 1>>();
  EXPECT_NEAR(expected_mean_ms / 1000, mean_seconds, 1);
}

GTEST_TEST(ProfileTester, LapTimerForIntervals) {
  const std::chrono::duration<double, std::milli> sleep_time(250.);
  const int kLapCount = 10;
  vector<double> lap_times(10, -1.);
  LapTimer timer;
  timer.start();
  for (int i = 0; i < kLapCount; ++i) {
    sleep_for(sleep_time);
    lap_times[i] = timer.lap<std::milli>();
    sleep_for(sleep_time);
    timer.start();
  }

  EXPECT_EQ(kLapCount, timer.laps());

  double expected_mean_ms = 0;
  for (double t : lap_times) expected_mean_ms += t;
  expected_mean_ms /= kLapCount;
  EXPECT_NEAR(expected_mean_ms, timer.average<std::milli>(), 1);
  sleep_for(sleep_time);
  // Delaying an arbitrary amount of time *after* the last lap, will not change
  // the mean lap duration.
  EXPECT_NEAR(expected_mean_ms, timer.average<std::milli>(), 1);
}

#endif  // ENABLE_TIMERS

}  // namespace
}  // namespace common
}  // namespace drake
