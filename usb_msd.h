/*
 * USB Mass Storage Class (MSC) driver for tinySA.
 * Exposes the SD card as a removable USB drive when connected to a PC.
 */

#ifndef _USB_MSD_H_
#define _USB_MSD_H_

#ifdef __USB_MSD__

/*
 * Enter USB drive mode: switches USB from CDC to MSC, presents the SD card
 * as a removable drive, then switches back to CDC on exit.
 * Blocks until the user presses a button or USB is disconnected.
 * Must be called from a thread context (not ISR).
 */
void usb_msd_loop(void);

#endif  /* __USB_MSD__ */

#endif  /* _USB_MSD_H_ */
