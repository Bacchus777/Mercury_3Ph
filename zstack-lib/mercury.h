#ifndef MERCURY_H
#define MERCURY_H

#include "zcl_app.h"

#define MERCURY_INVALID_RESPONSE 0xFFFF



typedef void (*start_stop_data_t)(uint8 serial_num, uint8 cmd);
typedef bool (*check_ready_t)(void);

typedef void (*request_measure_t)(uint8 serial_num, uint8 cmd);
typedef current_values_t (*read_curr_values_t)(uint8 cmd);
typedef uint32 (*read_energy_t)(uint8 cmd);

typedef struct {
  start_stop_data_t StartStopData;
  check_ready_t CheckReady;

  request_measure_t RequestMeasure;
  read_curr_values_t ReadCurrentValues;
  read_energy_t ReadEnergy;
} zclMercury_t;

#endif //MERCURY_H