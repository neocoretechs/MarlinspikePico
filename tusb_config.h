#pragma once

#include "tusb_option.h"

// Enable 1 CDC interface (Marlinspike only needs one)
#define CFG_TUD_CDC 1

// Disable all other USB classes
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

// Buffer sizes
#define CFG_TUD_CDC_RX_BUFSIZE 1024
#define CFG_TUD_CDC_TX_BUFSIZE 1024

// RP2040 device mode
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE