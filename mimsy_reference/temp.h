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

//=========================== define ==========================================

#define LOCALIZATION_PERIOD_MS 1000
#define PULSE_TRACK_COUNT 5

#define MIN_SYNC_PERIOD_US 52.1f
#define MAX_SYNC_PERIOD_US 135.5f

#define PI 3.141592653589793f
#define SWEEP_PERIOD_US 8333.333333f
#define DIODE_WIDTH_CM 0.45f // FIXME: measure for PCB

#define CLOCK_SPEED_MHZ 32.0f

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
