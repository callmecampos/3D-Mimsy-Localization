#include "opendefs.h"
#include "localization.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "scheduler.h"
#include "IEEE802154E.h"
#include "idmanager.h"
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "source/gpio.h"
#include "headers/hw_rfcore_sfr.h"
#include "headers/hw_rfcore_sfr.h"
#include "headers/hw_rfcore_xreg.h"
#include <math.h>
#include <stdio.h>
#include "gptimer.h"
#include "sys_ctrl.h"
#include "headers/hw_gptimer.h"
#include "headers/hw_ints.h"
#include "gpio.h"
#include "interrupt.h"
#include "headers/hw_memmap.h"
#include "headers/hw_gpio.h"
#include "headers/hw_ioc.h"
#include "ioc.h"

//=========================== variables =======================================

localization_vars_t localization_vars;

static const uint8_t localization_payload[]    = "localization";
static const uint8_t localization_dst_addr[]   = {
   0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};

static const uint32_t gptmEdgeTimerBase = GPTIMER3_BASE;
static const uint32_t gptmFallingEdgeInt = INT_TIMER3B;
static const uint32_t gptmFallingEdgeEvent = GPTIMER_CAPB_EVENT;

static const uint32_t gptmTimer3AReg = 0x40033048;
static const uint32_t gptmTimer3BReg = 0x4003304C;

static const uint32_t timer_cnt_32 = 0xFFFFFFFF;
static const uint32_t timer_cnt_16 = 0xFFFF;
static const uint32_t timer_cnt_24 = 0xFFFFFF;

static const float sweep_velocity = PI / SWEEP_PERIOD_US;

volatile pulse_t valid_pulses[PULSE_TRACK_COUNT][PULSE_TRACK_COUNT];
volatile pulse_t pulses[PULSE_TRACK_COUNT];
volatile uint8_t modular_ptr;
volatile uint32_t pulse_count;
volatile uint32_t count;
volatile uint32_t wsn_count;

volatile bool testRan;

//=========================== prototypes ======================================

void localization_timer_cb(opentimers_id_t id);
void localization_task_cb(void);
void open_timer_init(void);
void precision_timers_init(void);
void input_edge_timers_init(void);
void mimsy_GPIO_falling_edge_handler(void);

bool localize_mimsy(float *r, float *theta, float *phi, pulse_t *pulses_local, uint8_t *ptr);

//=========================== public ==========================================

void localization_init(void) {

    // clear local variables
    memset(&localization_vars,0,sizeof(localization_vars_t));
    testRan = false;
    count = 0; modular_ptr = 0; pulse_count = 0; wsn_count = 0;
    // initialize edges
    unsigned short int i;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        pulses[i] = (pulse_t){.rise = 0, .fall = 0, .type = -1};
    }
    unsigned short int j;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        for (j = 0; j < PULSE_TRACK_COUNT; j++) {
            valid_pulses[i][j].rise = 0; valid_pulses[i][j].fall = 0; valid_pulses[i][j].type = -1;
        }
    }

    // register at UDP stack
    localization_vars.desc.port              = WKP_UDP_LOCALIZATION;
    localization_vars.desc.callbackReceive   = &localization_receive;
    localization_vars.desc.callbackSendDone  = &localization_sendDone;
    openudp_register(&localization_vars.desc);

    volatile uint32_t _i;

    //Delay to avoid pin floating problems
    for (_i = 0xFFFF; _i != 0; _i--);

    // configure_pins();
    precision_timers_init();
    open_timer_init();
}

void open_timer_init(void){
    localization_vars.period = LOCALIZATION_PERIOD_MS;
    // start periodic timer
    localization_vars.timerId = opentimers_create();
    opentimers_scheduleIn(
        localization_vars.timerId,
        LOCALIZATION_PERIOD_MS,
        TIME_MS,
        TIMER_PERIODIC,
        localization_timer_cb
    );
}

void precision_timers_init(void){
    // SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT0); // enables timer0 module
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT3); // enables timer3 module

    input_edge_timers_init();

    // TimerConfigure(gptmPeriodTimerBase, GPTIMER_CFG_PERIODIC_UP);
    // TimerLoadSet(gptmPeriodTimerBase,GPTIMER_A,timer_cnt_32);
    // TimerEnable(gptmPeriodTimerBase,GPTIMER_A);
}

// NOTE: route single gpio pin to both timers and see if that still works
void input_edge_timers_init(void) {
    GPIOPinTypeTimer(GPIO_A_BASE,GPIO_PIN_2); // enables hw muxing of pin inputs
    GPIOPinTypeTimer(GPIO_A_BASE,GPIO_PIN_5); // enables hw muxing of pin inputs

    TimerConfigure(gptmEdgeTimerBase, GPTIMER_CFG_SPLIT_PAIR |
          GPTIMER_CFG_A_CAP_TIME_UP | GPTIMER_CFG_B_CAP_TIME_UP); // configures timer3a/b as 16-bit edge timers

    TimerPrescaleSet(gptmEdgeTimerBase,GPTIMER_A,0); // add prescaler to timer3a (24-bit)
    TimerPrescaleSet(gptmEdgeTimerBase,GPTIMER_B,0); // add prescaler to timer3b (24-bit)

    TimerLoadSet(gptmEdgeTimerBase,GPTIMER_A,timer_cnt_16);
    TimerLoadSet(gptmEdgeTimerBase,GPTIMER_B,timer_cnt_16);

    // FIXME: can we use the same gpio pin for both??
    IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_2, IOC_GPT3OCP1); // map gpio pin output to timer3a
    IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_5, IOC_GPT3OCP2); // map gpio pin output to timer3b

    // NOTE: the TS3633-CM1 Prototyping Module inverts rising and falling edges when light pulses are received,
    // so negative edges correspond to rising edges, and positive edges correspond to falling edges
    TimerControlEvent(gptmEdgeTimerBase, GPTIMER_A, GPTIMER_EVENT_NEG_EDGE); // set timer3a to capture rising edges (inverted by PCB)
    TimerControlEvent(gptmEdgeTimerBase, GPTIMER_B, GPTIMER_EVENT_POS_EDGE); // set timer3b to capture falling edges (inverted by PCB)

    TimerIntDisable(gptmEdgeTimerBase, gptmFallingEdgeEvent);
    TimerIntClear(gptmEdgeTimerBase, gptmFallingEdgeEvent);

    // set up interrupt for falling edge timer
    TimerIntRegister(gptmEdgeTimerBase, GPTIMER_B, mimsy_GPIO_falling_edge_handler);
    TimerIntEnable(gptmEdgeTimerBase, gptmFallingEdgeEvent);
    // IntPrioritySet(gptmFallingEdgeInt, 7<<5);
    IntEnable(gptmFallingEdgeInt);

    ENABLE_INTERRUPTS();

    TimerEnable(gptmEdgeTimerBase,GPTIMER_BOTH);

    TimerSynchronize(gptmEdgeTimerBase, GPTIMER_3A_SYNC | GPTIMER_3B_SYNC);
}

void localization_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}

void localization_receive(OpenQueueEntry_t* pkt) {

   openqueue_freePacketBuffer(pkt);

   openserial_printError(
      COMPONENT_localization,
      ERR_RCVD_ECHO_REPLY,
      (errorparameter_t)0,
      (errorparameter_t)0
   );
}

//=========================== private =========================================

void mimsy_GPIO_falling_edge_handler(void) {
    TimerIntClear(gptmEdgeTimerBase, gptmFallingEdgeEvent);

    // uint32_t time = TimerValueGet(gptmPeriodTimerBase, GPTIMER_A);
    // uint32_t rise = HWREG(gptmTimer3AReg) & 0xFFFF;
    // uint32_t fall = HWREG(gptmTimer3BReg) & 0xFFFF;

    // shift previous pulses and write to struct
    pulses[modular_ptr].rise = (uint32_t)(HWREG(gptmTimer3AReg)); // TimerValueGet(gptmEdgeTimerBase, GPTIMER_A);
    pulses[modular_ptr].fall = (uint32_t)(HWREG(gptmTimer3BReg)); // TimerValueGet(gptmEdgeTimerBase, GPTIMER_B);
    modular_ptr++; if (modular_ptr == 5) modular_ptr = 0;

    pulse_count += 1;
}

/**
\note timer fired, but we don't want to execute task in ISR mode instead, push
   task to scheduler with CoAP priority, and let scheduler take care of it.
*/
void localization_timer_cb(opentimers_id_t id){
    // count += 1;
    scheduler_push_task(localization_task_cb,TASKPRIO_COAP);
    // SCHEDULER_WAKEUP();
}

void localization_task_cb(void) {
   OpenQueueEntry_t*    pkt;
   // uint8_t              asnArray[5];

   // don't run if not synch
   if (ieee154e_isSynch() == FALSE) return;

   // don't run on dagroot
   if (idmanager_getIsDAGroot()) {
      opentimers_destroy(localization_vars.timerId);
      return;
   }

   union {
      float flt;
      unsigned char bytes[4];
   } x;

   union {
      float flt;
      unsigned char bytes[4];
   } y;

   union {
      float flt;
      unsigned char bytes[4];
   } z;

   float *r; float *theta; float *phi;
   pulse_t pulses_copy[PULSE_TRACK_COUNT];
   uint8_t *ptr;

   wsn_count += 1;

   // perform localization calculations
   if (!localize_mimsy(r, theta, phi, pulses_copy, ptr)) return;

   //x.flt = *r * sinf(*theta) * cosf(*phi);
   //y.flt = *r * sinf(*theta) * sinf(*phi);
   //z.flt = *r * cosf(*theta);

   // x.flt = *phi; y.flt = *theta; z.flt = *r;
   x.flt = (float) pulse_count; y.flt = (float) wsn_count; z.flt = (float) count;

   // if you get here, send a packet

   // get a free packet buffer
   pkt = openqueue_getFreePacketBuffer(COMPONENT_localization);
   if (pkt==NULL) {
      openserial_printError(
         COMPONENT_localization,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)0,
         (errorparameter_t)0
      );
      return;
   }

   pkt->owner                         = COMPONENT_localization;
   pkt->creator                       = COMPONENT_localization;
   pkt->l4_protocol                   = IANA_UDP;
   pkt->l4_destination_port           = WKP_UDP_LOCALIZATION;
   pkt->l4_sourcePortORicmpv6Type     = WKP_UDP_LOCALIZATION;
   pkt->l3_destinationAdd.type        = ADDR_128B;
   memcpy(&pkt->l3_destinationAdd.addr_128b[0],localization_dst_addr,16);

   // add payload
   packetfunctions_reserveHeaderSize(pkt,sizeof(localization_payload)-1);
   memcpy(&pkt->payload[0],localization_payload,sizeof(localization_payload)-1);

   // add payload
   packetfunctions_reserveHeaderSize(pkt,3*sizeof(float));
   pkt->payload[0] = x.bytes[0];
   pkt->payload[1] = x.bytes[1];
   pkt->payload[2] = x.bytes[2];
   pkt->payload[3] = x.bytes[3];
   pkt->payload[4] = y.bytes[0];
   pkt->payload[5] = y.bytes[1];
   pkt->payload[6] = y.bytes[2];
   pkt->payload[7] = y.bytes[3];
   pkt->payload[8] = z.bytes[0];
   pkt->payload[9] = z.bytes[1];
   pkt->payload[10] = z.bytes[2];
   pkt->payload[11] = z.bytes[3];

   packetfunctions_reserveHeaderSize(pkt,2*PULSE_TRACK_COUNT*sizeof(uint32_t));
   unsigned int payload_ind = 0;
   unsigned int ind;
   for (ind = 0; ind < PULSE_TRACK_COUNT; ind++) {
      union {
         uint32_t _int;
         unsigned char bytes[4];
      } start;

      start._int = pulses_copy[ind].rise;

      unsigned int i;
      for (i = 0; i < 4; i++) {
         pkt->payload[payload_ind] = start.bytes[i];
         payload_ind++;
      }

      union {
         uint32_t _int;
         unsigned char bytes[4];
      } end;

      end._int = pulses_copy[ind].fall;

      for (i = 0; i < 4; i++) {
         pkt->payload[payload_ind] = end.bytes[i];
         payload_ind++;
      }
   }

   /* packetfunctions_reserveHeaderSize(pkt,sizeof(asn_t));
   ieee154e_getAsn(asnArray);
   pkt->payload[0] = asnArray[0];
   pkt->payload[1] = asnArray[1];
   pkt->payload[2] = asnArray[2];
   pkt->payload[3] = asnArray[3];
   pkt->payload[4] = asnArray[4]; */

   if ((openudp_send(pkt))==E_FAIL) {
      openqueue_freePacketBuffer(pkt);
   } else {
      count += 1;
   }
}

float get_period_us_32(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((float) (end + (timer_cnt_32 - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((float) (end - start)) / CLOCK_SPEED_MHZ;
    }
}

float get_period_us(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((float) (end + (timer_cnt_24 - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((float) (end - start)) / CLOCK_SPEED_MHZ;
    }
}

/** Returns a number defining our 3 information bits: skip, data, axis.
  Given by our pulse length in microseconds (us). */
unsigned short int sync_bits(float duration) {
  return (unsigned short int) (48*duration - 2501) / 500;
}

bool localize_mimsy(float *r, float *theta, float *phi, pulse_t *pulses_local, uint8_t *ptr) {
    *ptr = modular_ptr;
    unsigned short int i;
    for (i = *ptr; i < *ptr + PULSE_TRACK_COUNT; i++) {
      pulses_local[i-*ptr].rise = pulses[i%PULSE_TRACK_COUNT].rise;
      pulses_local[i-*ptr].fall = pulses[i%PULSE_TRACK_COUNT].fall;
      pulses_local[i-*ptr].type = pulses[i%PULSE_TRACK_COUNT].type;
    }

    return true;

    unsigned short int init_sync_index = PULSE_TRACK_COUNT;

    // loop through and classify our pulses
    Pulses valid_seq_a[4] = { Sync, Horiz, Sync, Vert };
    Pulses valid_seq_b[4] = { Sync, Vert, Sync, Horiz };
    unsigned short int sweep_axes_check = 0;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        float period = get_period_us(pulses_local[i].rise, pulses_local[i].fall);
        if (period < MIN_SYNC_PERIOD_US) { // sweep pulse
            if (init_sync_index != PULSE_TRACK_COUNT) {
                float parent_period = get_period_us(pulses_local[i-1].rise, pulses_local[i-1].fall);
                int axis = (sync_bits(parent_period) & 0b001) + 1;
                pulses_local[i].type = axis; // 1 if horizontal, 2 if vertical

                int ind = i - init_sync_index;
                if (axis == ((int) valid_seq_a[ind]) || axis == ((int) valid_seq_b[ind])) {
                    sweep_axes_check += axis;
                } else {
                    return false;
                }
            }
        } else if (period < MAX_SYNC_PERIOD_US) { // sync pulse
            if (init_sync_index == PULSE_TRACK_COUNT) {
            	init_sync_index = i; // set initial valid sync pulse index
            }
            pulses_local[i].type = (int) Sync;
        } else { // neither
            pulses_local[i].type = -1;
            return false;
        }
    }

    if (init_sync_index == PULSE_TRACK_COUNT || sweep_axes_check != 3) return false;

    bool r_init = false; bool phi_init = false; bool theta_init = false;

    for (i = init_sync_index; i < PULSE_TRACK_COUNT-1; i++) {
        pulse_t curr_pulse = pulses_local[i];
        pulse_t next_pulse = pulses_local[i+1];

        switch(next_pulse.type) {
            case ((int) Sync):
                // *r = DIODE_WIDTH_CM / (get_period_us(next_pulse.rise, next_pulse.fall) * sweep_velocity);
                *r = get_period_us(next_pulse.rise, next_pulse.fall);
                r_init = true;
                break;
            case ((int) Horiz):
                *phi = get_period_us(curr_pulse.fall, next_pulse.rise) * sweep_velocity;
                phi_init = true;
                break;
            case ((int) Vert):
                *theta = get_period_us(curr_pulse.fall, next_pulse.rise) * sweep_velocity;
                theta_init = true;
                break;
            default:
                return false;
                break;
        }
    }

    return r_init && phi_init && theta_init;
}
