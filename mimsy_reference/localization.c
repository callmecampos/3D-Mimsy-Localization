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
#include "ioc.h"

//=========================== variables =======================================

localization_vars_t localization_vars;

static const uint8_t localization_payload[]    = "localization";
static const uint8_t localization_dst_addr[]   = {
   0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};

static const uint32_t gptmTimerClkEnable = SYS_CTRL_PERIPH_GPT3;
static const uint32_t gptmTimerBase = GPTIMER3_BASE;
static const uint32_t timer_cnt = 32000000;

static const float sweep_velocity = PI / SWEEP_PERIOD_US;

volatile pulse_t pulses[PULSE_TRACK_COUNT];
volatile bool startSeen;
volatile uint32_t start;

//=========================== prototypes ======================================

void localization_timer_cb(opentimers_id_t id);
void localization_task_cb(void);
void open_timer_init(void);
void precision_timer_init(void);
void configure_pins(void);
void openmote_GPIO_A_Handler(void);

bool localize_mimsy(float *r, float *theta, float *phi);

//=========================== public ==========================================

void localization_init(void) {

    // clear local variables
    memset(&localization_vars,0,sizeof(localization_vars_t));
    startSeen = false;
    start = 0;
    // initialize edges array
    unsigned short int i;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        pulses[i].start = 0; pulses[i].end = 0; pulses[i].sync_sweep = -1;
    }

    // register at UDP stack
    localization_vars.desc.port              = WKP_UDP_LOCALIZATION;
    localization_vars.desc.callbackReceive   = &localization_receive;
    localization_vars.desc.callbackSendDone  = &localization_sendDone;
    openudp_register(&localization_vars.desc);

    configure_pins();
    precision_timer_init();
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

void precision_timer_init(void){
    SysCtrlPeripheralEnable(gptmTimerClkEnable); // enables timer module

    TimerConfigure(gptmTimerBase, GPTIMER_CFG_PERIODIC_UP); // configures timers
    TimerLoadSet(gptmTimerBase,GPTIMER_A,timer_cnt);

    TimerEnable(gptmTimerBase,GPTIMER_A);
}

void configure_pins(void) {
    volatile uint32_t i;

    //Delay to avoid pin floating problems
    for (i = 0xFFFF; i != 0; i--);

    // disable interrupts for PA2
    GPIOPinIntDisable(GPIO_A_BASE, GPIO_PIN_2);
    // clear the interrupt for PA2
    GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_2);

    // configures PA2 to be GPIO input
    GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_2);

    // input GPIO on rising and falling edges
    GPIOIntTypeSet(GPIO_A_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);

    // register the port level interrupt handler
    GPIOPortIntRegister(GPIO_A_BASE, openmote_GPIO_A_Handler);

    // nested vector interrupt controller (set priority for my interrupts to be higher)
    // IntPrioritySet(GPIO_A_BASE, 0<<5);

    // clear pin
    GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_2);
    // enable the interrupt (unmasks the interrupt bit)
    GPIOPinIntEnable(GPIO_A_BASE, GPIO_PIN_2);

    ENABLE_INTERRUPTS();
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

/**
 * Openmote-cc2538 AD4/DIO4 interrupt handler.
 * call the cb function specified.
 */
void openmote_GPIO_A_Handler(void) {
    // Disable interrupts
    // DISABLE_INTERRUPTS();

    // clear the interrupt!
    GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_2);

    uint32_t time = TimerValueGet(gptmTimerBase, GPTIMER_A);

    if ((GPIOPinRead(GPIO_A_BASE, GPIO_PIN_2) & GPIO_PIN_2) == 0) {
        start = time;
        startSeen = true;
    } else if (startSeen) {
        // shift previous pulses
        unsigned short int i;
        for (i = 0; i < PULSE_TRACK_COUNT-1; i++) {
            pulses[i] = pulses[i+1];
        }
        pulses[PULSE_TRACK_COUNT-1].start = start;  pulses[PULSE_TRACK_COUNT-1].end = time;
    }

    // Enable interrupts
    // ENABLE_INTERRUPTS();
}

/**
\note timer fired, but we don't want to execute task in ISR mode instead, push
   task to scheduler with CoAP priority, and let scheduler take care of it.
*/
void localization_timer_cb(opentimers_id_t id){
    scheduler_push_task(localization_task_cb,TASKPRIO_COAP);
    // SCHEDULER_WAKEUP();
}

void localization_task_cb(void) {
   OpenQueueEntry_t*    pkt;
   uint8_t              asnArray[5];

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
   pulse_t pulses[PULSE_TRACK_COUNT];

   // perform localization calculations
   if (!localize_mimsy(r, theta, phi, pulses)) return;

   //x.flt = *r * sinf(*theta) * cosf(*phi);
   //y.flt = *r * sinf(*theta) * sinf(*phi);
   //z.flt = *r * cosf(*theta);

   x.flt = *phi; y.flt = *theta; z.flt = *r;

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

   packetfunctions_reserveHeaderSize(pkt,10*sizeof(uint32_t));
   unsigned int payload_ind = 0;
   unsigned int ind;
   for (ind = 0; ind < PULSE_TRACK_COUNT; ind++) {
      union {
         uint32_t _int;
         unsigned char bytes[4];
      } start;

      start._int = pulses_copy[ind].start;

      unsigned int i;
      for (i = 0; i < 4; i++) {
         pkt.payload[payload_ind] = start.bytes[i];
         payload_ind++;
      }

      union {
         uint32_t _int;
         unsigned char bytes[4];
      } end;

      end._int = pulses_copy[ind].end;

      for (i = 0; i < 4; i++) {
         pkt.payload[payload_ind] = end.bytes[i];
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
   }
}

float get_period_us(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((float) (end + (0xFFFFFFFF - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((float) (end - start)) / CLOCK_SPEED_MHZ;
    }
}

/** Returns a number defining our 3 information bits: skip, data, axis.
  Given by our pulse length in microseconds (us). */
unsigned short int sync_pulse(float duration) {
  return (unsigned short int) (48*duration - 2501) / 500;
}

bool localize_mimsy(float *r, float *theta, float *phi, pulse_t *pulses_local) {
    unsigned short int i;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        pulses_local[i].start = pulses[i].start;
        pulses_local[i].end = pulses[i].end;
        pulses_local[i].sync_sweep = pulses[i].sync_sweep;
    }

    unsigned short int init_sync_index = PULSE_TRACK_COUNT;

    // loop through and classify our pulses
    Pulses valid_seq_a[4] = { Sync, Horiz, Sync, Vert };
    Pulses valid_seq_b[4] = { Sync, Vert, Sync, Horiz };
    unsigned short int sweep_axes_check = 0;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        float period = get_period_us(pulses_local[i].start, pulses_local[i].end);
        if (period < MIN_SYNC_PERIOD_US) { // sweep pulse
            if (init_sync_index != PULSE_TRACK_COUNT) {
                int axis = (sync_pulse(period) & 0b001) + 1;
                pulses_local[i].sync_sweep = axis; // 1 if horizontal, 2 if vertical

                int ind = i - init_sync_index;
                if (axis == ((int) valid_seq_a[ind]) || axis == ((int) valid_seq_b[ind])) {
                    sweep_axes_check += axis;
                } else {
                    return false;
                }
            }
        } else if (period < MAX_SYNC_PERIOD_US) { // sync pulse
            if ((sync_pulse(period) & 0b100) == 1) { // skip pulse
                if (i > 0 && i < PULSE_TRACK_COUNT-1) {
                    return false;
                }
            } else if (init_sync_index == PULSE_TRACK_COUNT) {
            	init_sync_index = i; // set initial valid sync pulse index
            }
            pulses_local[i].sync_sweep = (int) Sync;
        } else { // neither
            pulses_local[i].sync_sweep = -1;
            return false;
        }
    }

    if (init_sync_index == PULSE_TRACK_COUNT || sweep_axes_check != 3) return false;

    bool r_init = false; bool phi_init = false; bool theta_init = false;

    for (i = init_sync_index; i < PULSE_TRACK_COUNT-1; i++) {
        pulse_t curr_pulse = pulses_local[i];
        pulse_t next_pulse = pulses_local[i+1];

        switch(next_pulse.sync_sweep) {
            case ((int) Sync):
                // *r = DIODE_WIDTH_CM / (get_period_us(next_pulse.start, next_pulse.end) * sweep_velocity);
                *r = get_period_us(next_pulse.start, next_pulse.end);
                r_init = true;
                break;
            case ((int) Horiz):
                *phi = get_period_us(curr_pulse.end, next_pulse.start) * sweep_velocity;
                phi_init = true;
                break;
            case ((int) Vert):
                *theta = get_period_us(curr_pulse.end, next_pulse.start) * sweep_velocity;
                theta_init = true;
                break;
            default:
                return false;
                break;
        }
    }

    return r_init && phi_init && theta_init;
}
