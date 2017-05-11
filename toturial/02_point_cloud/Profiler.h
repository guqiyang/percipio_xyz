/*
 * Profiler.h
 *
 *  Created on: Sep 18, 2016
 *      Author: qxji
 */

#ifndef _PROFILER_H_
#define _PROFILER_H_

#ifdef __linux__
#include <sys/time.h>
#endif

class Profiler {
 public:
  Profiler();
  virtual ~Profiler();

  static void fps(const char* tag = nullptr)
  {
    static int ITER = 20;
    static int loops = 0;
    static long long _fps_start = 0;
    if (loops == 0)
      _fps_start = current_ms();
    if (loops++ < ITER)
      return;
    loops = 0;
#ifdef __linux__
    long long current = current_ms();
    printf("%s, fps = %lld\n", tag, 1000/((current-_fps_start)/ITER));
#endif
  }

  static long long current_ms()
  {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (tv.tv_sec*1000 + tv.tv_usec/1000);
  }


};

#endif /* _PROFILER_H_ */
