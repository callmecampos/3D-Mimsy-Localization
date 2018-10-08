#ifndef __LOCALIZATION_H
#define __LOCALIZATION_H

/**
\addtogroup AppUdp
\{
\addtogroup localization
\{
*/

#include "opentimers.h"
#include "openudp.h"
#include <stdint.h>

//=========================== define ==========================================

#define WRITE_PERIOD_MS 10
#define LOCALIZATION_PERIOD_MS 300
#define PULSE_TRACK_COUNT 5

#define MIN_SYNC_PERIOD_US 52
#define MAX_SYNC_PERIOD_US 138

#define PI 3.14159265f
#define SWEEP_PERIOD_US 8333.333333f
#define DIODE_WIDTH_CM 0.45f // FIXME: measure for PCB

#define CLOCK_SPEED_MHZ 32.0f

#define IMU_DATA_STRUCT_SIZE 36 // (42 w/ mag, 36 w/o)

//=========================== typedef =========================================

//=========================== variables =======================================

/*IMUData is a union data structure used to store imu data points. Access the struct
type of this union is used for accessing and setting the data. The uint32 array version
of the struct is used for reading and writing to flash*/
typedef union IMUData {

  struct {
  uint16_t accelX;//a X data
  uint16_t accelY;//a X data
  uint16_t accelZ;//a X data

  uint16_t gyroX;//gyro X data
  uint16_t gyroY;//gyro Y data
  uint16_t gyroZ;//gyro Z data

  //uint16_t magX;//magnetometer X data
  //uint16_t magY;//magnetometer Y data
  //uint16_t magZ;//magnetometer Z data

  uint32_t timestamp;
  uint32_t pulse_0;
  uint32_t pulse_1;
  uint32_t pulse_2;
  uint32_t pulse_3;
  uint32_t pulse_4;

} fields;
  struct {
  int16_t accelX;//a X data
  int16_t accelY;//a X data
  int16_t accelZ;//a X data

  int16_t gyroX;//gyro X data
  int16_t gyroY;//gyro Y data
  int16_t gyroZ;//gyro Z data

  //uint16_t magX;//magnetometer X data
  //uint16_t magY;//magnetometer Y data
  //uint16_t magZ;//magnetometer Z data

  int32_t timestamp;
  uint32_t pulse_0;
  uint32_t pulse_1;
  uint32_t pulse_2;
  uint32_t pulse_3;
  uint32_t pulse_4; // do I leave these unsigned?

} signedfields;
uint32_t bits[9];
}IMUData;

/*This struct is used to keep track of wherer data was written to. This struct
must be passed to flashWriteIMU where it is updated to include the flash location
of the data. A written data card is passed to flashReadIMU inorder to read the
data from that location*/
typedef struct IMUDataCard{
    uint32_t page;
    uint32_t startTime;
    uint32_t endTime;
} IMUDataCard;

extern void flashWriteIMU(IMUData data[],uint32_t size, uint32_t startPage, int wordsWritten);
extern void flashReadIMU(IMUDataCard card, IMUData *data, uint32_t size);
extern void flashReadIMUSection(IMUDataCard card, IMUData *data, uint32_t size,int wordsRead);

typedef struct {
   opentimers_id_t      timerId;  ///< periodic timer which triggers transmission
   uint16_t             counter;  ///< incrementing counter which is written into the packet
   uint16_t              period;  ///< localization packet sending period>
   udp_resource_desc_t     desc;  ///< resource descriptor for this module, used to register at UDP stack
} localization_vars_t;

typedef struct {
   uint32_t                time;
   uint32_t                rise;
   uint32_t                fall;
   int                     type; // -1 for unclassified, 0 for Sync, 1 for Horiz, 2 for Vert
} pulse_t;

typedef enum {
   Sync, Horiz, Vert,
} Pulses;

//=========================== prototypes ======================================

void localization_init(void);
void localization_sendDone(OpenQueueEntry_t* msg, owerror_t error);
void localization_receive(OpenQueueEntry_t* msg);
/**
\}
\}
*/

#endif
