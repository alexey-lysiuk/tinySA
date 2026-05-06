/*
 * USB Mass Storage Class (MSC) driver — Bulk-Only Transport (BOT).
 * Exposes the SD card as a removable USB drive when connected to a PC.
 *
 * Protocol overview:
 *   Host  --[CBW 31B]--> Device          Command Block Wrapper
 *   Host <--[data   ]--- Device  (IN)    Optional data phase (read)
 *   Host  --[data   ]--> Device  (OUT)   Optional data phase (write)
 *   Host <--[CSW 13B]--- Device          Command Status Wrapper
 */

#include "ch.h"
#include "hal.h"
#include "nanovna.h"
#include "usbcfg.h"
#include "usb_msd.h"

#ifdef __USB_MSD__

#include <string.h>

/*===========================================================================*/
/* Constants                                                                 */
/*===========================================================================*/

/* Endpoint numbers (same as CDC EP1, used for both IN and OUT bulk) */
#define MSD_EP_IN    1U
#define MSD_EP_OUT   1U

/* BOT (Bulk-Only Transport) signatures */
#define MSD_CBW_SIGNATURE   0x43425355UL  /* 'USBC' */
#define MSD_CSW_SIGNATURE   0x53425355UL  /* 'USBS' */
#define MSD_CBW_SIZE        31U
#define MSD_CSW_SIZE        13U

/* CBW flags */
#define MSD_CBW_DIRECTION_IN   0x80U   /* Device-to-host data phase */

/* CSW status codes */
#define MSD_CSW_CMD_PASSED   0U
#define MSD_CSW_CMD_FAILED   1U

/* SCSI transparent command set opcodes */
#define SCSI_TEST_UNIT_READY        0x00U
#define SCSI_REQUEST_SENSE          0x03U
#define SCSI_INQUIRY                0x12U
#define SCSI_MODE_SENSE6            0x1AU
#define SCSI_START_STOP_UNIT        0x1BU
#define SCSI_PREVENT_ALLOW_REMOVAL  0x1EU
#define SCSI_READ_CAPACITY10        0x25U
#define SCSI_READ10                 0x28U
#define SCSI_WRITE10                0x2AU

/* SCSI sense keys */
#define SCSI_SKEY_NO_SENSE          0x00U
#define SCSI_SKEY_NOT_READY         0x02U
#define SCSI_SKEY_ILLEGAL_REQUEST   0x05U

/* Sector size matches SD card */
#define MSD_SECTOR_SIZE             512U

/*
 * Transmit-side timeout (ms).  Once we have started sending data to the host
 * the ACK should arrive quickly.  A 2-second timeout guards against a hung
 * host without blocking normal idle periods (which can be many seconds).
 * The receive side (CBW) has NO timeout so the drive works correctly while
 * the host-side FS is simply mounted and idle.
 */
#define MSD_TX_TIMEOUT_MS          2000U

/*
 * Maximum time (ms) to wait for the host to (re-)enumerate the device after a
 * USB bus reset or on initial connection.  If no SET_CONFIGURATION arrives
 * within this window the cable is assumed to be unplugged and USB drive mode
 * is exited.
 */
#define MSD_ENUMERATION_TIMEOUT_MS 5000U

/*===========================================================================*/
/* Packed structures                                                         */
/*===========================================================================*/

typedef struct {
  uint32_t dSignature;
  uint32_t dTag;
  uint32_t dDataTransferLength;
  uint8_t  bmFlags;
  uint8_t  bLUN;
  uint8_t  bCBLength;
  uint8_t  CB[16];
} __attribute__((packed)) msd_cbw_t;

typedef struct {
  uint32_t dSignature;
  uint32_t dTag;
  uint32_t dDataResidue;
  uint8_t  bStatus;
} __attribute__((packed)) msd_csw_t;

/*===========================================================================*/
/* State                                                                     */
/*===========================================================================*/

static volatile bool msd_running    = false;
static volatile bool msd_configured = false;

/* Sense data for REQUEST_SENSE */
static uint8_t msd_sense_key = SCSI_SKEY_NO_SENSE;
static uint8_t msd_asc       = 0U;
static uint8_t msd_ascq      = 0U;

/* Cached sector count (obtained once from GET_SECTOR_COUNT) */
static uint32_t msd_sector_count = 0U;

/*===========================================================================*/
/* Buffers                                                                   */
/*===========================================================================*/

/* CBW and CSW buffers (word-aligned for USB DMA) */
static __attribute__((aligned(4))) uint8_t msd_cbw_buf[MSD_CBW_SIZE];
static __attribute__((aligned(4))) uint8_t msd_csw_buf[MSD_CSW_SIZE];

/*
 * Sector data buffer: reuse spi_buffer (at least 4096 bytes, unused during
 * USB drive mode since sweep and display updates are suspended).
 */
#define msd_sector_buf  ((uint8_t *)spi_buffer)

/*===========================================================================*/
/* USB MSC Descriptors                                                       */
/*===========================================================================*/

/* Device descriptor: idProduct 0x5741 differs from CDC 0x5740 */
static const uint8_t msd_device_descriptor_data[18] = {
  USB_DESC_DEVICE(0x0200,    /* bcdUSB (2.0)                */
                  0x00,      /* bDeviceClass (iface-defined)*/
                  0x00,      /* bDeviceSubClass             */
                  0x00,      /* bDeviceProtocol             */
                  0x40,      /* bMaxPacketSize0 (64)        */
                  0x0483,    /* idVendor (ST)               */
                  0x5741,    /* idProduct (MSC mode)        */
                  0x0100,    /* bcdDevice (1.00)            */
                  1,         /* iManufacturer               */
                  2,         /* iProduct                    */
                  3,         /* iSerialNumber               */
                  1)         /* bNumConfigurations          */
};

static const USBDescriptor msd_device_descriptor = {
  sizeof msd_device_descriptor_data,
  msd_device_descriptor_data
};

/*
 * Configuration descriptor (9 + 9 + 7 + 7 = 32 bytes):
 *   1 interface (MSC), 2 bulk endpoints (EP1 IN + EP1 OUT).
 */
static const uint8_t msd_configuration_descriptor_data[32] = {
  /* Configuration */
  USB_DESC_CONFIGURATION(32,    /* wTotalLength               */
                         1,     /* bNumInterfaces             */
                         1,     /* bConfigurationValue        */
                         0,     /* iConfiguration             */
                         0xC0,  /* bmAttributes (self-powered)*/
                         50),   /* bMaxPower (100 mA)         */
  /* Interface: Mass Storage / SCSI transparent / BOT */
  USB_DESC_INTERFACE(0x00,      /* bInterfaceNumber           */
                     0x00,      /* bAlternateSetting          */
                     0x02,      /* bNumEndpoints              */
                     0x08,      /* bInterfaceClass (MSC)      */
                     0x06,      /* bInterfaceSubClass (SCSI)  */
                     0x50,      /* bInterfaceProtocol (BOT)   */
                     0),        /* iInterface                 */
  /* EP1 IN (bulk, device-to-host) */
  USB_DESC_ENDPOINT(0x81,       /* bEndpointAddress           */
                    0x02,       /* bmAttributes (Bulk)        */
                    0x0040,     /* wMaxPacketSize (64)        */
                    0x00),      /* bInterval                  */
  /* EP1 OUT (bulk, host-to-device) */
  USB_DESC_ENDPOINT(0x01,       /* bEndpointAddress           */
                    0x02,       /* bmAttributes (Bulk)        */
                    0x0040,     /* wMaxPacketSize (64)        */
                    0x00),      /* bInterval                  */
};

static const USBDescriptor msd_configuration_descriptor = {
  sizeof msd_configuration_descriptor_data,
  msd_configuration_descriptor_data
};

/* String 0: language (US English) */
static const uint8_t msd_string0[] = {
  USB_DESC_BYTE(4),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  USB_DESC_WORD(0x0409)
};

/* String 1: vendor — "tinysa.org" */
static const uint8_t msd_string1[] = {
  USB_DESC_BYTE(22),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  't', 0, 'i', 0, 'n', 0, 'y', 0, 's', 0, 'a', 0, '.', 0, 'o', 0, 'r', 0, 'g', 0
};

/* String 2: product */
#ifdef TINYSA4
static const uint8_t msd_string2[] = {
  USB_DESC_BYTE(16),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  't', 0, 'i', 0, 'n', 0, 'y', 0, 'S', 0, 'A', 0, '4', 0
};
#else
static const uint8_t msd_string2[] = {
  USB_DESC_BYTE(14),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  't', 0, 'i', 0, 'n', 0, 'y', 0, 'S', 0, 'A', 0
};
#endif

/* String 3: serial number (ChibiOS kernel version) */
static const uint8_t msd_string3[] = {
  USB_DESC_BYTE(8),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

static const USBDescriptor msd_strings[] = {
  {sizeof msd_string0, msd_string0},
  {sizeof msd_string1, msd_string1},
  {sizeof msd_string2, msd_string2},
  {sizeof msd_string3, msd_string3}
};

static const USBDescriptor *msd_get_descriptor(USBDriver *usbp,
                                                uint8_t dtype,
                                                uint8_t dindex,
                                                uint16_t lang) {
  (void)usbp; (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &msd_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &msd_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4U)
      return &msd_strings[dindex];
  }
  return NULL;
}

/*===========================================================================*/
/* Endpoint configuration                                                    */
/*===========================================================================*/

static USBInEndpointState  msd_ep1instate;
static USBOutEndpointState msd_ep1outstate;

static const USBEndpointConfig msd_ep1config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,   /* setup_cb  — not needed for bulk */
  NULL,   /* in_cb     — polling used instead */
  NULL,   /* out_cb    — polling used instead */
  0x0040, /* in_maxsize  */
  0x0040, /* out_maxsize */
  &msd_ep1instate,
  &msd_ep1outstate,
};

/*===========================================================================*/
/* USB event handler                                                         */
/*===========================================================================*/

static void msd_usb_event(USBDriver *usbp, usbevent_t event) {
  switch (event) {
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();
    usbInitEndpointI(usbp, MSD_EP_IN, &msd_ep1config);
    msd_configured = true;
    chSysUnlockFromISR();
    return;
  case USB_EVENT_RESET:
    /*
     * USB bus reset: _usb_reset() has already nulled all endpoint configs
     * and cleared the receiving/transmitting bitmasks.  Mark the device as
     * unconfigured so the BOT loop waits for the host to re-enumerate.
     * Do NOT treat this as a fatal error — Windows and other OSes routinely
     * issue a bus reset during MSC enumeration.
     */
    msd_configured = false;
    return;
  case USB_EVENT_SUSPEND:
    /*
     * USB bus suspended.  The configuration is retained across suspend/resume
     * so there is no new USB_EVENT_CONFIGURED on wakeup.  Clearing
     * msd_configured here would cause the BOT loop to wait forever for a
     * second CONFIGURED event that never arrives.  Leave it set; the polling
     * loop detects the non-ACTIVE state and retries.
     */
    return;
  case USB_EVENT_ADDRESS:
  case USB_EVENT_WAKEUP:
  case USB_EVENT_STALLED:
    return;
  }
}

/* MSC class requests: Mass Storage Reset (0xFF) and Get Max LUN (0xFE) */
static bool msd_requests_hook(USBDriver *usbp) {
  if ((usbp->setup.bmRequestType & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS &&
      (usbp->setup.bmRequestType & USB_RTYPE_RECIPIENT_MASK) == USB_RTYPE_RECIPIENT_INTERFACE) {
    switch (usbp->setup.bRequest) {
    case 0xFF:  /* Mass Storage Reset */
      usbSetupTransfer(usbp, NULL, 0, NULL);
      return true;
    case 0xFE:  /* Get Max LUN: report 0 (one LUN, the SD card) */
      {
        /* lun_count is a constant (zero = one LUN); must be addressable for usbSetupTransfer */
        static const uint8_t lun_count = 0U;
        usbSetupTransfer(usbp, (uint8_t *)&lun_count, 1, NULL);
      }
      return true;
    }
  }
  return false;
}

/* USB MSC configuration, used in place of the CDC usbcfg while in drive mode */
const USBConfig msd_usbcfg = {
  msd_usb_event,
  msd_get_descriptor,
  msd_requests_hook,
  NULL   /* no SOF handler needed */
};

/*===========================================================================*/
/* Blocking USB transfer helpers (polling-based)                             */
/*===========================================================================*/

/*
 * Poll the USB 'transmitting' bitmask until the IN transfer on MSD_EP_IN
 * is complete, or we time out / USB disconnects.
 * Button press is NOT checked here: once a data or CSW phase has started the
 * BOT protocol requires it to complete before returning to the idle state.
 * Exit detection happens in msd_receive_data() between commands.
 * Returns true on success.
 */
static bool msd_transmit(const uint8_t *buf, size_t n) {
  osalSysLock();
  if (usbGetDriverStateI(&USBD1) != USB_ACTIVE) {
    osalSysUnlock();
    return false;
  }
  usbStartTransmitI(&USBD1, MSD_EP_IN, buf, n);
  osalSysUnlock();

  systime_t start = chVTGetSystemTimeX();
  while (USBD1.transmitting & (1U << MSD_EP_IN)) {
    if (USBD1.state != USB_ACTIVE) return false;
    if (chVTTimeElapsedSinceX(start) > MS2ST(MSD_TX_TIMEOUT_MS)) return false;
    chThdSleepMilliseconds(1);
  }
  return USBD1.state == USB_ACTIVE;
}

/*
 * Poll the USB 'receiving' bitmask until the OUT transfer on MSD_EP_OUT
 * is complete, or the USB bus changes state or the user presses the button.
 * There is intentionally NO timeout: a correctly mounted drive may sit idle
 * for an arbitrary amount of time between host commands.
 * Returns true when data is available.
 * Sets msd_running = false and returns false on button press.
 * Returns false (without touching msd_running) on any USB state change
 * (reset, suspend, disconnect) so the BOT loop can wait for re-enumeration.
 */
static bool msd_receive_data(uint8_t *buf, size_t n) {
  osalSysLock();
  if (usbGetDriverStateI(&USBD1) != USB_ACTIVE) {
    osalSysUnlock();
    return false;
  }
  usbStartReceiveI(&USBD1, MSD_EP_OUT, buf, n);
  osalSysUnlock();

  while (USBD1.receiving & (1U << MSD_EP_OUT)) {
    if (USBD1.state != USB_ACTIVE) return false;
    if (operation_requested & OP_LEVER) {
      operation_requested &= (uint8_t)~OP_LEVER;
      msd_running = false;
      return false;
    }
    chThdSleepMilliseconds(1);
  }
  return USBD1.state == USB_ACTIVE;
}

/* Send Command Status Wrapper */
static bool msd_send_csw(uint32_t tag, uint32_t residue, uint8_t status) {
  msd_csw_t *csw = (msd_csw_t *)msd_csw_buf;
  csw->dSignature   = MSD_CSW_SIGNATURE;
  csw->dTag         = tag;
  csw->dDataResidue = residue;
  csw->bStatus      = status;
  return msd_transmit(msd_csw_buf, MSD_CSW_SIZE);
}

/*===========================================================================*/
/* SCSI command handlers                                                     */
/*===========================================================================*/

/* Standard INQUIRY response (36 bytes) */
static const uint8_t msd_inquiry_data[36] = {
  0x00,                               /* Peripheral device type (block)  */
  0x80,                               /* Removable media bit             */
  0x04,                               /* SCSI version (2)                */
  0x02,                               /* Response data format            */
  0x1F,                               /* Additional length (31)          */
  0x00, 0x00, 0x00,                   /* Reserved                        */
  't', 'i', 'n', 'y', 's', 'a', ' ', ' ',  /* Vendor (8 bytes)          */
  'S', 'D', ' ', 'C', 'a', 'r', 'd', ' ',  /* Product (16 bytes)        */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '1', '.', '0', '0'                  /* Revision (4 bytes)              */
};

static bool scsi_inquiry(const msd_cbw_t *cbw) {
  uint32_t len = cbw->dDataTransferLength;
  if (len > sizeof(msd_inquiry_data)) len = sizeof(msd_inquiry_data);
  return msd_transmit(msd_inquiry_data, len);
}

static bool scsi_test_unit_ready(void) {
  if (disk_status(0) & STA_NOINIT) {
    msd_sense_key = SCSI_SKEY_NOT_READY;
    msd_asc  = 0x3AU;   /* Medium not present */
    msd_ascq = 0x00U;
    return false;
  }
  msd_sense_key = SCSI_SKEY_NO_SENSE;
  msd_asc = 0U; msd_ascq = 0U;
  return true;
}

static bool scsi_request_sense(const msd_cbw_t *cbw) {
  uint8_t sense[18];
  memset(sense, 0, sizeof(sense));
  sense[0]  = 0x70U;       /* Fixed-format, valid          */
  sense[2]  = msd_sense_key;
  sense[7]  = 0x0AU;       /* Additional length = 10       */
  sense[12] = msd_asc;
  sense[13] = msd_ascq;
  uint32_t len = cbw->dDataTransferLength;
  if (len > sizeof(sense)) len = sizeof(sense);
  return msd_transmit(sense, len);
}

static bool scsi_read_capacity10(void) {
  if (msd_sector_count == 0U) {
    msd_sense_key = SCSI_SKEY_NOT_READY;
    msd_asc  = 0x3AU;
    msd_ascq = 0x00U;
    return false;
  }
  uint8_t resp[8];
  uint32_t last_lba = msd_sector_count - 1U;
  resp[0] = (uint8_t)(last_lba >> 24);
  resp[1] = (uint8_t)(last_lba >> 16);
  resp[2] = (uint8_t)(last_lba >>  8);
  resp[3] = (uint8_t)(last_lba);
  resp[4] = (uint8_t)(MSD_SECTOR_SIZE >> 24);
  resp[5] = (uint8_t)(MSD_SECTOR_SIZE >> 16);
  resp[6] = (uint8_t)(MSD_SECTOR_SIZE >>  8);
  resp[7] = (uint8_t)(MSD_SECTOR_SIZE);
  return msd_transmit(resp, sizeof(resp));
}

static bool scsi_read10(const msd_cbw_t *cbw) {
  uint32_t lba   = ((uint32_t)cbw->CB[2] << 24) | ((uint32_t)cbw->CB[3] << 16)
                 | ((uint32_t)cbw->CB[4] <<  8) | ((uint32_t)cbw->CB[5]);
  uint16_t count = (uint16_t)(((uint16_t)cbw->CB[7] << 8) | cbw->CB[8]);
  for (uint16_t i = 0U; i < count; i++) {
    if (disk_read(0, msd_sector_buf, lba + i, 1) != RES_OK) {
      msd_sense_key = SCSI_SKEY_NOT_READY;
      msd_asc  = 0x11U;   /* Unrecovered read error */
      msd_ascq = 0x00U;
      return false;
    }
    if (!msd_transmit(msd_sector_buf, MSD_SECTOR_SIZE))
      return false;
  }
  return true;
}

static bool scsi_write10(const msd_cbw_t *cbw) {
  uint32_t lba   = ((uint32_t)cbw->CB[2] << 24) | ((uint32_t)cbw->CB[3] << 16)
                 | ((uint32_t)cbw->CB[4] <<  8) | ((uint32_t)cbw->CB[5]);
  uint16_t count = (uint16_t)(((uint16_t)cbw->CB[7] << 8) | cbw->CB[8]);
  for (uint16_t i = 0U; i < count; i++) {
    if (!msd_receive_data(msd_sector_buf, MSD_SECTOR_SIZE))
      return false;
    if (disk_write(0, msd_sector_buf, lba + i, 1) != RES_OK) {
      msd_sense_key = SCSI_SKEY_NOT_READY;
      msd_asc  = 0x03U;   /* Peripheral device write fault */
      msd_ascq = 0x00U;
      return false;
    }
  }
  return true;
}

static bool scsi_mode_sense6(const msd_cbw_t *cbw) {
  /* Minimal mode parameter header — no block descriptor */
  static const uint8_t mode_data[4] = {
    0x03U,  /* Mode data length (3 remaining bytes) */
    0x00U,  /* Medium type                          */
    0x00U,  /* Device-specific (not write-protected)*/
    0x00U   /* Block descriptor length = 0          */
  };
  uint32_t len = cbw->dDataTransferLength;
  if (len > sizeof(mode_data)) len = sizeof(mode_data);
  return msd_transmit(mode_data, len);
}

/*===========================================================================*/
/* BOT command dispatcher                                                    */
/*===========================================================================*/

/* Process one CBW.  Returns msd_running (true = continue, false = exit). */
static bool process_msd_cbw(void) {
  const msd_cbw_t *cbw = (const msd_cbw_t *)msd_cbw_buf;

  if (cbw->dSignature != MSD_CBW_SIGNATURE ||
      cbw->bCBLength < 1U || cbw->bCBLength > 16U) {
    /*
     * Malformed CBW: per MSC BOT spec §6.6.1 the device must stall both
     * bulk endpoints.  The host will then issue a Mass Storage Reset or a
     * USB bus reset to recover.  We do NOT exit — just wait for that reset
     * to arrive (msd_configured will be cleared, the BOT loop re-enumerates).
     */
    osalSysLock();
    usbStallReceiveI(&USBD1,  MSD_EP_OUT);
    usbStallTransmitI(&USBD1, MSD_EP_IN);
    osalSysUnlock();
    return msd_running;
  }

  uint32_t tag     = cbw->dTag;
  uint8_t  cmd     = cbw->CB[0];
  bool     cmd_ok  = true;
  bool     has_in  = (cbw->bmFlags & MSD_CBW_DIRECTION_IN) != 0U;
  uint32_t residue = cbw->dDataTransferLength;

  msd_sense_key = SCSI_SKEY_NO_SENSE;
  msd_asc = 0U; msd_ascq = 0U;

  switch (cmd) {
  case SCSI_TEST_UNIT_READY:
    cmd_ok = scsi_test_unit_ready();
    break;
  case SCSI_INQUIRY:
    cmd_ok = scsi_inquiry(cbw);
    if (cmd_ok) residue = 0U;
    break;
  case SCSI_REQUEST_SENSE:
    cmd_ok = scsi_request_sense(cbw);
    if (cmd_ok) residue = 0U;
    break;
  case SCSI_READ_CAPACITY10:
    cmd_ok = scsi_read_capacity10();
    if (cmd_ok) residue = 0U;
    break;
  case SCSI_READ10:
    cmd_ok = scsi_read10(cbw);
    if (cmd_ok) residue = 0U;
    break;
  case SCSI_WRITE10:
    cmd_ok = scsi_write10(cbw);
    if (cmd_ok) residue = 0U;
    break;
  case SCSI_MODE_SENSE6:
    cmd_ok = scsi_mode_sense6(cbw);
    if (cmd_ok) residue = 0U;
    break;
  case SCSI_START_STOP_UNIT:
    /*
     * CB[4] bit layout: bit 0 = START, bit 1 = LoEj.
     * Value 0x02 means LoEj=1, START=0 → eject/stop the media.
     * Windows "Safely Remove Hardware" sends exactly this before suspending
     * the device.  Set msd_running=false so the BOT loop exits after sending
     * the CSW, restoring the CDC interface gracefully.
     * All other START_STOP_UNIT variants (spin-up, etc.) are acknowledged
     * as success but otherwise ignored — we are always "spun up".
     */
    if ((cbw->CB[4] & 0x03U) == 0x02U) {  /* LoEj=1 (bit 1), START=0 (bit 0) */
      msd_running = false;
    }
    cmd_ok = true;   /* Always succeed regardless of LoEj/START values */
    break;
  default:
    msd_sense_key = SCSI_SKEY_ILLEGAL_REQUEST;
    msd_asc  = 0x20U;   /* Invalid command operation code */
    msd_ascq = 0x00U;
    cmd_ok = false;
    /* Stall IN endpoint when host expects data we cannot provide */
    if (has_in && cbw->dDataTransferLength > 0U) {
      osalSysLock();
      usbStallTransmitI(&USBD1, MSD_EP_IN);
      osalSysUnlock();
    }
    break;
  }

  msd_send_csw(tag, residue, cmd_ok ? MSD_CSW_CMD_PASSED : MSD_CSW_CMD_FAILED);
  return msd_running;
}

/*===========================================================================*/
/* Display helper                                                            */
/*===========================================================================*/

static void msd_draw_screen(void) {
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_fill(0, 0, LCD_WIDTH, LCD_HEIGHT);
  ili9341_set_foreground(LCD_FG_COLOR);
  ili9341_drawstring_7x13("USB DRIVE MODE",        (LCD_WIDTH - 14 * 7) / 2, LCD_HEIGHT / 2 - 26);
  ili9341_drawstring_7x13("SD card to PC",         (LCD_WIDTH - 13 * 7) / 2, LCD_HEIGHT / 2 - 8);
  ili9341_drawstring_7x13("Press button to exit",  (LCD_WIDTH - 20 * 7) / 2, LCD_HEIGHT / 2 + 10);
}

/*===========================================================================*/
/* Public entry point                                                        */
/*===========================================================================*/

void usb_msd_loop(void) {
  /* Pause sweep and dismiss any open UI */
  uint8_t old_sweep_mode = sweep_mode;
  sweep_mode = 0U;
  ui_mode_normal();

  /* Unmount FatFS to allow direct sector access by the host.
   * If unmounting fails the volume may have pending writes; proceed anyway
   * since the drive was already synced by the previous f_sync/f_close calls
   * and direct sector access is required for MSC. */
  (void)f_unmount("");

  /*
   * Query SD card geometry NOW, while the SPI bus is idle and CDC is still
   * active.  Doing this before usbConnectBus guarantees that msd_sector_count
   * is valid the instant the host sends its first READ_CAPACITY command after
   * enumeration.  If we queried it after usbConnectBus, the host could send
   * READ_CAPACITY before the ioctl finished and receive NOT_READY, triggering
   * a 1–10 second retry backoff that makes mounting appear very slow.
   */
  msd_sector_count = 0U;
  disk_ioctl(0, GET_SECTOR_COUNT, &msd_sector_count);

  /*
   * Switch USB from CDC to MSC:
   *   1. Physically disconnect so the host sees the device leave.
   *   2. Stop the CDC (SDU) driver.
   *   3. Restart USB with the MSC configuration.
   *   4. Reconnect.
   */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(200);
  sduStop(&SDU1);
  usbStop(&USBD1);
  usbStart(&USBD1, &msd_usbcfg);
  usbConnectBus(&USBD1);

  msd_draw_screen();

  msd_running    = true;
  msd_configured = false;

  /*
   * BOT processing loop.
   *
   * Design goals:
   *  - Survive USB bus resets.  Windows (and other OSes) routinely issue one
   *    or more bus resets during MSC enumeration and may issue further resets
   *    during normal operation.  After each reset the host re-enumerates the
   *    device and sends a fresh INQUIRY/READ_CAPACITY sequence; we must handle
   *    that transparently without exiting to the main screen.
   *  - Survive USB bus suspend/resume.  The configuration survives a
   *    suspend/resume cycle (no second CONFIGURED event is generated), so
   *    we must not wait for a second CONFIGURED event after a wakeup.
   *  - Exit cleanly on button press or if the host never reconnects within
   *    a MSD_ENUMERATION_TIMEOUT_MS window (cable unplugged).
   */
  while (msd_running) {

    /*
     * If not configured — either on startup or after a USB bus reset — wait
     * for the host to re-enumerate the device.  A MSD_ENUMERATION_TIMEOUT_MS
     * timeout guards against the case where the cable was unplugged.
     */
    if (!msd_configured) {
      systime_t t_wait = chVTGetSystemTimeX();
      while (!msd_configured && msd_running) {
        chThdSleepMilliseconds(10);
        if (chVTTimeElapsedSinceX(t_wait) > MS2ST(MSD_ENUMERATION_TIMEOUT_MS)) {
          /* Host did not re-enumerate: device likely unplugged */
          msd_running = false;
        }
        if (operation_requested & OP_LEVER) {
          operation_requested &= (uint8_t)~OP_LEVER;
          msd_running = false;
        }
      }
      /* Re-check msd_running; on timeout/button skip to exit */
      continue;
    }

    /*
     * Receive a Command Block Wrapper from the host.
     * msd_receive_data blocks until data arrives, the USB state changes
     * (reset / suspend), or the user presses the button.
     */
    if (!msd_receive_data(msd_cbw_buf, MSD_CBW_SIZE)) {
      /*
       * Receive failed.  Most likely causes:
       *  - USB bus reset  → msd_configured will be false
       *  - USB suspend    → USBD1.state == USB_SUSPENDED
       *  - Button press   → msd_running already set false by msd_receive_data
       *
       * In any case check the button here too, because when the USB is
       * suspended msd_receive_data returns false immediately (without ever
       * entering its polling loop) so OP_LEVER is never seen otherwise.
       */
      if (operation_requested & OP_LEVER) {
        operation_requested &= (uint8_t)~OP_LEVER;
        msd_running = false;
      }
      chThdSleepMilliseconds(10);
      continue;
    }

    /* Execute the SCSI command and send the Command Status Wrapper */
    process_msd_cbw();
    /* msd_running == false if button was pressed inside process_msd_cbw */
  }

  msd_running = false;

  /* Disconnect MSC and restore CDC */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(200);
  usbStop(&USBD1);

  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

  /* Restore sweep mode */
  sweep_mode = old_sweep_mode;
}

#endif  /* __USB_MSD__ */
