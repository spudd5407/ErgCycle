
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

/* Some CT Microcontroller / Protocol Constants */

/* read timeouts in microseconds */
#define CT_READTIMEOUT    1000
#define CT_WRITETIMEOUT   2000

// message type
#define CT_SPEED        0x01
#define CT_POWER        0x02
#define CT_HEARTRATE    0x03
#define CT_CADENCE      0x06
#define CT_RRC          0x09
#define CT_SENSOR       0x0b

// buttons
#define CT_RESET        0x01
#define CT_F1           0x02
#define CT_F3           0x04
#define CT_PLUS         0x08
#define CT_F2           0x10
#define CT_MINUS        0x20
#define CT_SSS          0x40    // spinscan sync is not a button!
#define CT_NONE         0x80  //sets reset normally high when not pressed

/* Device operation mode */
#define CT_ERGOMODE    0x01
#define CT_SSMODE      0x02
#define CT_CALIBRATE   0x04

/* UI operation mode */
#define UI_MANUAL 0x01  // using +/- keys to adjust
#define UI_ERG    0x02  // running an erg file!

/* Control status */
#define CT_RUNNING 0x01
#define CT_PAUSED  0x02

/* default operation mode */
#define DEFAULT_MODE        CT_ERGOMODE
#define DEFAULT_LOAD        100.00
#define DEFAULT_GRADIENT    2.00
#define DEFAULT_SPEED		5.0		//mps
#define DEFAULT_POWER		50		//watts
#define DEFAULT_HEARTRATE	60
#define DEFAULT_CADENCE		60

#define MISSED_MSG_TIMEOUT  50                          // Timeout the session after 50 missed control messages (set minimum resistance etc)


 // Protocol encoding
    void prepareCommand(int mode, double value);  // sets up the command packet according to current settings
    int sendCommand();      // writes a command to the device
    int calcCRC(int value);     // calculates the checksum for the current command

    // Protocol decoding
    int getMessage();
    void unpackTelemetry(int &type, int &value8, int &value12);
