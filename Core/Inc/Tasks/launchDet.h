#ifndef LAUNCHDET_H
#define LAUNCHDET_H

#include <stdbool.h>

void checkRecorded();
void checkFlightState();
void notifyTasks();
void satisfyWDG();
bool canardsActive(void); // true while the current rocket's canards phase is active

#endif //LAUNCHDET_H