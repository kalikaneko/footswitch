/* Name: main.c
 * Project: foot-switch
 * Author: Kali Kaneko
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usbdrv/usbdrv.h"
#include "usbdrv/oddebug.h"


/* We define two vendor-specific control messages */
#define USB_LED_OFF 0
#define USB_LED_ON  1
#define USB_DATA_OUT 2
#define USB_DATA_WRITE 3
#define USB_DATA_IN 4

#define LED_PIN (1<<PB0)

#define BUTTON_PORT PORTB       /* PORTx - register for button output */
#define BUTTON_PIN PINB         /* PINx - register for button input */
#define BUTTON_BIT PB3          /* bit for button input/output */

/*static uchar replyBuf[16] = "Hello, USB!";*/
/* static uchar dataReceived = 0, dataLength = 0; // for USB_DATA_IN */


int doModifier = 0;
static uchar mod = 0;

static uchar reportBuffer[2];   /* buffer for HID reports */
static uchar idleRate;
static uchar reportCount;

static uchar buttonState;
static uchar debounceTimeIsOver;

static uchar pedalKeyPress = 0x30; 
static uchar pedalKeyRelease = 0x2f;

static uchar winKey = 0xe3;
static uchar winKey2 = 0xe7;


/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */
static void timerPoll(void) {
	static unsigned int timerCnt;

    if(TIFR & (1 << TOV1)){
        TIFR = (1 << TOV1); /* clear overflow */
        if(++timerCnt >= 3){       // 3/63 sec delay for switch debouncing
			timerCnt = 0;
			debounceTimeIsOver = 1; 
        }
    }
}

static void buildReport(void) {
    uchar key = 0; //if not changed by the if-statement below, then send an empty report

    if(reportCount == 0){
        if (buttonState == 0){
			// if button is not pressed
		    // 	key = pedalKeyPress;
			mod = 0;
			//key = 0;
		} else {
			// pressed!
		    //key = pedalKeyRelease;
			//key = winKey;
			mod = winKey;
			key = 0;
    	}
    }

	reportCount++;

	reportBuffer[0] = mod;
	reportBuffer[1] = key;
}

static void checkButtonChange(void) {
	
	uchar tempButtonValue = bit_is_clear(BUTTON_PIN, BUTTON_BIT); //status of switch is stored in tempButtonValue 

	if (tempButtonValue != buttonState && debounceTimeIsOver == 1){ //if status has changed and the debounce-delay is over
		buttonState = tempButtonValue;	// change buttonState to new state
		debounceTimeIsOver = 0;	// debounce timer starts
		reportCount = 0; // start report 
	}
}

static void timerInit(void)
{
    TCCR1 = 0x0b;           /* select clock: 16.5M/1k -> overflow rate = 16.5M/256k = 62.94 Hz */
}

/* --------- interface to USB driver --------------------
 * ------------------------------------------------------
 */

// this gets called when custom control message is received
USB_PUBLIC uchar usbFunctionSetup(uchar data[8]) {
	usbRequest_t *rq = (void *)data; // cast data to correct type

    usbMsgPtr = reportBuffer;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            buildReport();
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
	} else {
		switch(rq->bRequest) { // custom command is in the bRequest field
		case USB_LED_ON:
			doModifier = 1;
			PORTB |= LED_PIN; // turn LED on
			return 0;
		case USB_LED_OFF: 
			doModifier = 0;
			PORTB &= ~LED_PIN; // turn LED off
			return 0;
		}
	}
    return 0; // should not get here
}


#define abs(x) ((x) > 0 ? (x) : (-x))

// Called by V-USB after device reset
void hadUsbReset() {
    int frameLength, targetLength = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
    int bestDeviation = 9999;
    uchar trialCal, bestCal, step, region;
	bestCal = 0;

    // do a binary search in regions 0-127 and 128-255 to get optimum OSCCAL
    for(region = 0; region <= 1; region++) {
        frameLength = 0;
        trialCal = (region == 0) ? 0 : 128;
        
        for(step = 64; step > 0; step >>= 1) { 
            if(frameLength < targetLength) // true for initial iteration
                trialCal += step; // frequency too low
            else
                trialCal -= step; // frequency too high
                
            OSCCAL = trialCal;
            frameLength = usbMeasureFrameLength();
            
            if(abs(frameLength-targetLength) < bestDeviation) {
                bestCal = trialCal; // new optimum found
                bestDeviation = abs(frameLength -targetLength);
            }
        }
    }

    OSCCAL = bestCal;
}

int main() {
    uchar i;

	DDRB = LED_PIN; // LED pin as output

    wdt_enable(WDTO_1S); // enable 1s watchdog timer

    usbInit();
        
    usbDeviceDisconnect(); // enforce re-enumeration
    for(i = 0; i<250; i++) { // wait 500 ms
        wdt_reset(); // keep the watchdog happy
        _delay_ms(2);
    }
    usbDeviceConnect();

	/* turn on internal pull-up resistor for the switch */
    BUTTON_PORT |= _BV(BUTTON_BIT);

	timerInit();
        
    sei(); // Enable interrupts after re-enumeration
        
    while(1) {
        wdt_reset(); // keep the watchdog happy
        usbPoll();
		if(usbInterruptIsReady() && reportCount < 2){ /* we can send another key */
			buildReport();
			usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
		}
		checkButtonChange();
		timerPoll();
    }

    return 0;
}

