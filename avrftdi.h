#ifndef avrftdi_h
#define avrfdti_h

#ifdef __cplusplus
extern "C" {
#endif

#define SCK 0x01
#define SDO 0x02
#define SDI 0x04

#define RX 0x20
#define TX 0x11

#define TRX (RX | TX)

#define PGM 0x1
#define RDY 0x2
#define VFY 0x4
#define ERR 0x8

#define E(x) if ((x)) { fprintf(stdout, "%s:%d %s() %s: %s (%d)\n\t%s\n", __FILE__, __LINE__, __FUNCTION__, \
	#x, strerror(errno), errno, ftdi_get_error_string(&ftdic)); exit(1); }

void avrftdi_initpgm        (PROGRAMMER * pgm);

#ifdef __cplusplus
}
#endif

#endif


