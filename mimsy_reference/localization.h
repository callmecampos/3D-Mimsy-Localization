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

#include <stdint.h> // maybe...

//=========================== define ==========================================

#define LOCALIZATION_PERIOD_MS 66
#define PULSE_TRACK_COUNT 10

#define MIN_SYNC_PERIOD_US 52
#define MAX_SYNC_PERIOD_US 138

#define PI 3.14159265f
#define SWEEP_PERIOD_US 8333.333333f
#define DIODE_WIDTH_CM 0.45f // FIXME: measure for PCB

#define CLOCK_SPEED_MHZ 32.0f

#define PULSE_DATA_STRUCT_SIZE 60

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
   opentimers_id_t      timerId;  ///< periodic timer which triggers transmission
   uint16_t             counter;  ///< incrementing counter which is written into the packet
   uint16_t              period;  ///< localization packet sending period>
   udp_resource_desc_t     desc;  ///< resource descriptor for this module, used to register at UDP stack
} localization_vars_t;

typedef struct {
   uint32_t                rise;
   uint32_t                fall;
   int                     type; // -1 for unclassified, 0 for Sync, 1 for Horiz, 2 for Vert
} pulse_t;

typedef struct {
	float                    phi;
	float                  theta;
	float                 r_vert;
	float				      r_horiz;
	uint8_t               asn[5];
	int						  valid;
} location_t;

typedef enum {
   Sync, Horiz, Vert,
} Pulses;

/*PulseData is a union data structure used to store pulse data points. Access the struct
type of this union is used for accessing and setting the data. The uint32 array version
of the struct is used for reading and writing to flash*/
typedef union PulseData {
  struct {
  uint32_t pulse_0s;
  uint32_t pulse_0e;
  uint32_t pulse_1s;
  uint32_t pulse_1e;
  uint32_t pulse_2s;
  uint32_t pulse_2e;
  uint32_t pulse_3s;
  uint32_t pulse_3e;
  uint32_t pulse_4s;
  uint32_t pulse_4e;
  uint32_t pulse_5s;
  uint32_t pulse_5e;
  uint32_t pulse_6s;
  uint32_t pulse_6e;
  uint32_t pulse_7s;
  uint32_t pulse_7e;
  uint32_t pulse_8s;
  uint32_t pulse_8e;
  uint32_t pulse_9s;
  uint32_t pulse_9e;

} fields;
  struct {
  int32_t pulse_0s;
  int32_t pulse_0e;
  int32_t pulse_1s;
  int32_t pulse_1e;
  int32_t pulse_2s;
  int32_t pulse_2e;
  int32_t pulse_3s;
  int32_t pulse_3e;
  int32_t pulse_4s;
  int32_t pulse_4e;
  int32_t pulse_5s;
  int32_t pulse_5e;
  int32_t pulse_6s;
  int32_t pulse_6e;
  int32_t pulse_7s;
  int32_t pulse_7e;
  int32_t pulse_8s;
  int32_t pulse_8e;
  int32_t pulse_9s;
  int32_t pulse_9e;

} signedfields;
uint32_t bits[20];
}PulseData;

/*This struct is used to keep track of wherer data was written to. This struct
must be passed to flashWritePulse where it is updated to include the flash location
of the data. A written data card is passed to flashReadIMU inorder to read the
data from that location*/
typedef struct PulseDataCard{
    uint32_t page;
    uint32_t startTime;
    uint32_t endTime;
} PulseDataCard;

extern void flashWritePulse(PulseData data[],uint32_t size, uint32_t startPage, int wordsWritten);
extern void flashReadPulse(PulseDataCard card, PulseData *data, uint32_t size);
extern void flashReadPulseSection(PulseDataCard card, PulseData *data, uint32_t size,int wordsRead);

//=========================== prototypes ======================================

void localization_init(void);
void localization_sendDone(OpenQueueEntry_t* msg, owerror_t error);
void localization_receive(OpenQueueEntry_t* msg);
/**
\}
\}
*/

#endif
