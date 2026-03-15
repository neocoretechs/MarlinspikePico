#include "tusb.h"

// --------------------------------------------------------------------
// Device Descriptor
// --------------------------------------------------------------------

tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,        // Replace with your VID if you have one
    .idProduct          = 0x4001,        // Unique PID for Marlinspike
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const* tud_descriptor_device_cb(void)
{
    return (uint8_t const*) &desc_device;
}

// --------------------------------------------------------------------
// Configuration Descriptor (CDC Only)
// --------------------------------------------------------------------

enum
{
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

uint8_t const desc_configuration[] =
{
    // Configuration descriptor
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // CDC interface descriptor
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, 0x81, 8, 0x02, 0x82, 64)
};
uint8_t const* tud_descriptor_configuration_cb(uint8_t index)
{
    return desc_configuration;
}


// --------------------------------------------------------------------
// String Descriptors
// --------------------------------------------------------------------

char const* string_desc_arr[] =
{
    (const char[]) { 0x09, 0x04 }, // 0: English (0x0409)
    "NeoCoreTechs",                // 1: Manufacturer
    "Marlinspike Control Plane",   // 2: Product
    "MSP-001",                     // 3: Serial number
    "Marlinspike CDC Interface"    // 4: CDC interface name
};

static uint16_t _desc_str[32];
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void) langid;

    uint8_t chr_count;

    if (index == 0)
    {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }
    else
    {
        const char* str = string_desc_arr[index];
        chr_count = strlen(str);

        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = str[i];
    }

    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
}