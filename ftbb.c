/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003-2004  Theodore A. Roth  <troth@openavr.org>
 * Copyright (C) 2005 Johnathan Corgan <jcorgan@aeinet.com>
 * Copyright (C) 2006 Thomas Fischl <tfischl@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * avrdude interface for serial programming via FTDI bit bang mode operation. 
 */

#include "ac_cfg.h"
#include "avr.h"
#include "pgm.h"
#include "ftbb.h"
#include <string.h>

#if FTDI_SUPPORT
#define FTBB_DEBUG 1

/* for debugging paged write can be switched off and buffer size set to 1 */
#define FTBB_USE_PAGED_WRITE 1
#define BBBUFFER_SIZE 100

#if HAVE_LIBFTD2XX
#include <ftd2xx.h>
#elif HAVE_FTDI_H
#include <ftdi.h>
#endif

#define RESET   (1<<(pgm->pinno[PIN_AVR_RESET]-1))
#define SCK	(1<<(pgm->pinno[PIN_AVR_SCK]-1))
#define MOSI	(1<<(pgm->pinno[PIN_AVR_MOSI]-1))
#define MISO	(1<<(pgm->pinno[PIN_AVR_MISO]-1))
#define FTDDR	(MOSI|SCK|RESET)

#if HAVE_LIBFTD2XX
static FT_STATUS status;
static FT_HANDLE handle;
#elif HAVE_FTDI_H
static struct ftdi_context device;
static int status;
#endif

static unsigned char txbits;

/* Buffer to collect bitbang sequences */
static unsigned char bbbuffer[BBBUFFER_SIZE];
static int bbbuffer_pos = 0;

/* 
 * Send data from bitbang buffer to ftdi
 */
static void bbbuffer_flush(void) {

  if (bbbuffer_pos == 0) return;

#if HAVE_LIBFTD2XX
      FT_Write(handle, bbbuffer, bbbuffer_pos, &written);
#elif HAVE_FTDI_H
      ftdi_write_data(&device, bbbuffer, bbbuffer_pos);
#endif /* HAVE_LIBFTD2XX */

  bbbuffer_pos = 0;
}

/*
 * Insert byte into bitbang buffer
 */
static void bbbuffer_add(unsigned char value) {

  if (bbbuffer_pos >= BBBUFFER_SIZE)
    bbbuffer_flush();

  bbbuffer[bbbuffer_pos] = value;  
  bbbuffer_pos ++;
}

/*
 * Transmit command over SPI to AVR
 * skipreceive: count of bytes which aren't read back.
 * AVRs receive data only during the last byte-transfer (skipreceive = 3)
 * If no receiption is needed (skipreceive = 4), no read back is used.
 */
static void spi_transmit(PROGRAMMER * pgm, unsigned char sendbuffer[4], 
                         unsigned char receivebuffer[4], unsigned char skipreceive)
{
#if HAVE_LIBFTD2XX
    DWORD written;
#endif /* HAVE_LIBFTD2XX */

  unsigned char pins = 0;
  int i, bitpos;

  for (i = 0; i < 4; i++) {

    /* clear receive byte */
    receivebuffer[i] = 0;

    for (bitpos = 7; bitpos >= 0; bitpos--) {

      /* check if databit is high */
      if ((sendbuffer[i] & (1 << bitpos)) != 0)
        txbits |= MOSI;
      else
        txbits &= ~MOSI;

      /* set data with low SCK */
      txbits &= ~SCK;

      bbbuffer_add(txbits);

      if (i < skipreceive) {
      } else {

        /* it seems that larger packets arn't sent out before the read command without sleep */ 
        int temp = bbbuffer_pos;
        bbbuffer_flush();
        if (temp > 5)
          usleep(BBBUFFER_SIZE * 10);

#if HAVE_LIBFTD2XX
        FT_GetBitMode(handle, &pins);
#elif HAVE_FTDI_H
        ftdi_read_pins(&device, &pins);        
#endif /* HAVE_LIBFTD2XX */

        if ((pins & MISO) != 0)
          receivebuffer[i] |= (1 << bitpos);        
      }

      /* set SCL high */
      txbits |= SCK;

      bbbuffer_add(txbits);
    }
    
  }
  bbbuffer_flush();
}

/* Generic programmer command function for pgm->cmd
 *
 * Sends four bytes of command in sequence and collects responses
 *
 */
static int ftbb_cmd(PROGRAMMER *pgm, unsigned char cmd[4], unsigned char res[4])
{
    
#if FTBB_DEBUG
    printf("CMD: %02X%02X%02X%02X  ", cmd[0], cmd[1], cmd[2], cmd[3]);
#endif

    spi_transmit(pgm, cmd, res, 0);

#if FTBB_DEBUG
    printf("RES: %02X%02X%02X%02X\n", res[0], res[1], res[2], res[3]);
#endif

    return 0;
}

#if FTBB_USE_PAGED_WRITE
/*
 * Write page to flash
 */
static int ftbb_flush_page(PROGRAMMER *pgm, AVRPART *p, AVRMEM * mem, int addr)
{
  unsigned char cmd[4];
  unsigned char res[4];
  
  if (mem->op[AVR_OP_WRITEPAGE] == NULL) {
      fprintf(stderr, "write page instruction not defined for part \"%s\"\n", p->desc);
      return -1;
  }

  /* if word-addressed */
  if ((mem->op[AVR_OP_LOADPAGE_LO]) || (mem->op[AVR_OP_READ_LO]))
    addr = addr / 2;

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(mem->op[AVR_OP_WRITEPAGE], cmd);
  avr_set_addr(mem->op[AVR_OP_WRITEPAGE], cmd, addr);
  spi_transmit(pgm, cmd, res, 4);

  bbbuffer_flush();
  usleep(mem->max_write_delay);

  return 0;
}

/*
 * Write byte to flash / load byte to the AVR page buffer
 */
static int ftbb_write_byte(PROGRAMMER *pgm, AVRPART *p, AVRMEM * mem, int addr, unsigned char data)
{
  unsigned char cmd[4];
  unsigned char res[4];
  OPCODE * writeop;
  unsigned short caddr;


  /*
   * determine which memory opcode to use
   */
  if (mem->op[AVR_OP_WRITE_LO]) {
    if (addr & 0x01)
      writeop = mem->op[AVR_OP_WRITE_HI];
    else
      writeop = mem->op[AVR_OP_WRITE_LO];
    caddr = addr / 2;
  }
  else if (mem->op[AVR_OP_LOADPAGE_LO]) {
    if (addr & 0x01)
      writeop = mem->op[AVR_OP_LOADPAGE_HI];
    else
      writeop = mem->op[AVR_OP_LOADPAGE_LO];
    caddr = addr / 2;
  }
  else {
    writeop = mem->op[AVR_OP_WRITE];
    caddr = addr;
  }

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(writeop, cmd);
  avr_set_addr(writeop, cmd, caddr);
  avr_set_input(writeop, cmd, data);
  spi_transmit(pgm, cmd, res, 4);

  return 0;
}

/*
 * Write whole binary dataset to flash
 */
static int ftbb_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  int address = 0;
  int page_bytes = page_size;

  while (address < n_bytes) {

    if (strcmp(m->desc, "flash")) 
      avr_write_byte(pgm, p, m, address, m->buf[address]);
    else
      ftbb_write_byte(pgm, p, m, address, m->buf[address]);

    page_bytes --;

    /* flush page if necessary */
    if (m->paged && (page_bytes == 0)) {
      ftbb_flush_page(pgm, p, m, address);
      page_bytes = page_size;
    }

    report_progress (address, n_bytes, NULL);
    address ++;
  }

  /* do we need an final page write? */
  if (m->paged && (page_bytes < page_size)) {
    ftbb_flush_page(pgm, p, m, address - 1);
  }

  return address;
 
}

#endif // FTBB_USE_PAGED_WRITE

/* Programmer initialization command for pgm->initialize
 *
 * Pulse RESET with SCK low, then send SPI start programming command
 *
 */
static int ftbb_initialize(PROGRAMMER *pgm, AVRPART *p)
{
#if HAVE_LIBFTD2XX
    DWORD written;
#endif /* HAVE_LIBFTD2XX */

    // Atmel documentation says to raise RESET for 2 cpu clocks while sclk is low
    // then lower RESET and wait 20 ms.
    txbits |= RESET;

#if HAVE_LIBFTD2XX
    FT_Write(handle, &txbits, 1, &written);
#elif HAVE_FTDI_H
    ftdi_write_data(&device, &txbits, 1);
#endif /* HAVE_LIBFTD2XX */

    usleep(1000); // 2 AVR cpu clocks at any realistic clock rate
    txbits &= ~RESET;

#if HAVE_LIBFTD2XX
    FT_Write(handle, &txbits, 1, &written);
#elif HAVE_FTDI_H
    ftdi_write_data(&device, &txbits, 1);
#endif /* HAVE_LIBFTD2XX */

    usleep(20000);

    pgm->program_enable(pgm, p);
    return 0;
}

/* Programmer status display for pgm->display
 *
 * Not-implemented
 *
 */
static void ftbb_display(PROGRAMMER *pgm, char *p)
{
#if FTBB_DEBUG
    printf("ftbb: display called with: %s\n", p);
#endif

    printf("FTBB: RESET mapped to pinno %d\n", pgm->pinno[PIN_AVR_RESET]);
    printf("FTBB: SCK   mapped to pinno %d\n", pgm->pinno[PIN_AVR_SCK]);
    printf("FTBB: MOSI  mapped to pinno %d\n", pgm->pinno[PIN_AVR_MOSI]);
    printf("FTBB: MISO  mapped to pinno %d\n", pgm->pinno[PIN_AVR_MISO]);
}  

/* Programmer enable command for pgm->enable
 *
 * Lowers SCK and RESET in preparation for serial programming
 *
 */
static void ftbb_enable(PROGRAMMER *pgm)
{
#if HAVE_LIBFTD2XX
    DWORD written;
#endif /* HAVE_LIBFTD2XX */

    // Lower SCK & RESET
    txbits &= ~SCK;
    txbits &= ~RESET;

#if HAVE_LIBFTD2XX
    FT_Write(handle, &txbits, 1, &written);
#elif HAVE_FTDI_H
    ftdi_write_data(&device, &txbits, 1);
#endif /* HAVE_LIBFTD2XX */
}

/* Programmer disable command for pgm->disable
 *
 * Raises RESET to return to normal chip operation
 *
 */
static void ftbb_disable(PROGRAMMER *pgm)
{
#if HAVE_LIBFTD2XX
    DWORD written;
#endif /* HAVE_LIBFTD2XX */  
    // Raise RESET to return to normal mode
    txbits |= RESET;

#if HAVE_LIBFTD2XX
    FT_Write(handle, &txbits, 1, &written);
#elif HAVE_FTDI_H
    ftdi_write_data(&device, &txbits, 1);
#endif /* HAVE_LIBFTD2XX */
}

/* Programmer programming mode enable function for pgm->program_enable
 *
 * Starts SPI programming mode 
 *
 */
static int ftbb_program_enable(PROGRAMMER *pgm, AVRPART *p)
{
    unsigned char cmd[4];
    unsigned char res[4];
    int           status;
  
    if (p->op[AVR_OP_PGM_ENABLE] == NULL) {
        fprintf(stderr, "program enable instruction not defined for part \"%s\"\n", p->desc);
        return -1;
    }

    memset(cmd, 0, sizeof(cmd));
    avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
    status = pgm->cmd(pgm, cmd, res);
    
    if (status == 0 && res[2] != cmd[1]) {
      fprintf(stderr, "%s: %s %s received bad echo: expected %02x but was %02x\n",
                      pgm->type, p->desc,
                      "program enable",
                      cmd[1], res[2]);
      status = -8;
    }
    
    return status;
}

/* Progammer erase function for pgm->erase
 *
 * Sends chip erase command and sleeps the chip erase delay
 *
 */ 
static int ftbb_chip_erase(PROGRAMMER *pgm, AVRPART *p)
{
    unsigned char cmd[4];
    unsigned char res[4];

    if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
        fprintf(stderr, "chip erase instruction not defined for part \"%s\"\n", p->desc);
        return -1;
    }

    memset(cmd, 0, sizeof(cmd));
    avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
    pgm->cmd(pgm, cmd, res);
    usleep(p->chip_erase_delay);

    return 0;
}


/* Parse routine for device name
 *
 * FFFF:FFFF:0
 *
 */
int ftbb_parse_name(char *name, int *vid, int *pid, int *ifc)
{
    printf("name=%s\n", name);
    return 0;
}

/* Programmer open command for pgm->open
 *
 * Opens FTD2XX device specified by 'name', performs a chip reset, then
 * sets the baudrate and bit bang mode
 *
 */
static int ftbb_open(PROGRAMMER *pgm, char *name)
{
    int vid, pid, ifc;
    ftbb_parse_name(name, &vid, &pid, &ifc);
    
#if HAVE_LIBFTD2XX
    int devnum = 0;
    if (strcmp(name, "ft0")) {
        fprintf(stderr, "ERROR: FTD2XX device selection not yet implemented!\n");
        return -1;
    }

    // Call FTD2XX library to open device
    if ((status = FT_Open(devnum, &handle)) != FT_OK) {
        fprintf(stderr, "Failed to open FTD2XX device #%d.\n", devnum);
        return -1;        
    }

    // Reset chipset
    if ((status = FT_ResetDevice(handle)) != FT_OK) {
        fprintf(stderr, "Failed to reset chipset for FTD2XX device #%d.\n", devnum);
        return -1;
    }

    // Set baud rate for bit bang interface
    if ((status = FT_SetBaudRate(handle, pgm->baudrate)) != FT_OK) {
        fprintf(stderr, "Failed to set baud rate for FTD2XX device #%d.\n", devnum);
        return -1;
    }

    // Set bit bang direction and mode
    if ((status = FT_SetBitMode(handle, FTDDR, 1)) != FT_OK) {
        fprintf(stderr, "Failed to set bit bang mode for FTD2XX device #%d.\n", devnum);
        return -1;
    }
#elif HAVE_FTDI_H
    // Open device via FTDI library *** FIXME *** hardcoded VID and PID
    if (ftdi_usb_open(&device, EZDOP_VENDORID, EZDOP_PRODUCTID)) {
	fprintf(stderr, "ftdi_usb_open: %s", device.error_str);
	return -1;
    }

    // Reset FTDI chipset
    if (ftdi_usb_reset(&device)) {
        fprintf(stderr, "ftdi_usb_reset: %s", device.error_str);
	return -1;
    }

    // Set FTDI chipset baudrate for bitbang
    if (ftdi_set_baudrate(&device, pgm->baudrate)) {
	fprintf(stderr, "ftdi_set_baudrate: %s", device.error_str);
	return -1;
    }

    // Enable bitbang
    if (ftdi_enable_bitbang(&device, FTDDR)) {
        fprintf(stderr, "ftdi_enable_bitbang: %s", device.error_str);
	return -1;
    }

    // Minimum chunk size for reads to reduce latency
    if (ftdi_read_data_set_chunksize(&device, 256)) {
        fprintf(stderr, "ftdi_read_data_set_chunksize: %s", device.error_str);
	return -1;
    }
#endif /* HAVE_LIBFTD2XX */

    return 0;
}

/* Programmer close function for pgm->close
 *
 * Releases FTD2XX device
 *
 */
static void ftbb_close(PROGRAMMER *pgm)
{
#if HAVE_LIBFTD2XX
    FT_Close(handle);
#elif HAVE_FTDI_H
    ftdi_deinit(&device);
#endif
}
#endif /* FTDI_SUPPORT */

/* Programmer initialization command for startup.
 *
 * Sets appropriate function pointers in structure
 * Tests FTD2XX interface by enumerating number of devices
 *
 */
void ftbb_initpgm (PROGRAMMER *pgm)
{
#if FTDI_SUPPORT

#if HAVE_LIBFTD2XX
    DWORD num_devices;
#endif

    strcpy(pgm->type, "ftbb");
    pgm->initialize = ftbb_initialize;
    pgm->display = ftbb_display;
    pgm->enable = ftbb_enable;
    pgm->disable = ftbb_disable;
    pgm->program_enable = ftbb_program_enable;
    pgm->chip_erase = ftbb_chip_erase;  
    pgm->cmd = ftbb_cmd;
    pgm->open = ftbb_open;
    pgm->close = ftbb_close;  

#if FTBB_USE_PAGED_WRITE
    pgm->paged_write = ftbb_paged_write;
#endif

#if HAVE_LIBFTD2XX
    if ((status = FT_ListDevices(&num_devices, NULL, FT_LIST_NUMBER_ONLY)) != FT_OK)
        fprintf(stderr, "Failed to initialize FTD2XX interface. (%li)\n", status);
    else
        printf("%lu FTD2XX device(s) found.\n", num_devices);
#elif HAVE_FTDI_H
    if ((status = ftdi_init(&device)) < 0)
        fprintf(stderr, "Failed to initialize FTDI interface. (%i)\n", status);
#endif     

    txbits = RESET;
#endif /* FTDI_SUPPORT */
}

