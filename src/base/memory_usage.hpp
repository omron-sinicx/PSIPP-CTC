/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#if defined _POSIX_VERSION || defined _POSIX2_VERSION || defined __linux__
#include <unistd.h>

#include <fstream>
#include <ios>
#include <iostream>

double getUsedMemory() {
  std::ifstream stat_stream("/proc/self/stat", std::ios_base::in);

  // dummy variables
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string O, itrealvalue, starttime;

  unsigned long vsize;
  long rss;

  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >>
      tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt >> utime >>
      stime >> cutime >> cstime >> priority >> nice >> O >> itrealvalue >>
      starttime >> vsize >> rss;
  stat_stream.close();

  return rss * (sysconf(_SC_PAGE_SIZE) / 1024.);
}
#else
double getUsedMemory() {
  fprintf(stderr, "Memory usage cannot be measured\n");
  return 0.0;
}
#endif
