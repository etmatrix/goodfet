/*! \file goodfet.h
  \author Travis Goodspeed
  \brief Port descriptions for the GoodFET platform.
*/

/* #ifdef __MSPGCC__ */
/* #include <msp430.h> */
/* #else */
/* #include <signal.h> */
/* #include <msp430.h> */
/* #include <iomacros.h> */
/* #endif */

#ifndef _GNU_ASSEMBLER_
#include <msp430.h>
#endif

#define useuart1

/* PIN MAPPING
 *          -----------------------------
 *          |        P3.4  1  2 VCC     |
 *          |        P3.3  3  4         |
 *     ------        P3.5  5  6         |
 *     | USB         P2.7  7  8 P6.0    |
 *     ------        GND   9 10         |
 *          |        P6.1 11 12 RX      |
 *          |GoodFET      13 14 TX      |
 *          -----------------------------
 */

//LED on P1.0
#define PLEDOUT P1OUT
#define PLEDDIR P1DIR
#define PLEDPIN BIT0

//LED on P4.7
#define PLED2OUT P4OUT
#define PLED2DIR P4DIR
#define PLED2PIN BIT7

// SPI CLK P2.7 SPI A0
#define SPICLKOUT P2OUT
#define SPICLKDIR P2DIR
#define SPICLKREN P2REN

// SPI MOSI P3.3 SPI A0
#define SPIMOSIOUT P3OUT
#define SPIMOSIDIR P3DIR
#define SPIMOSIREN P3REN

// SPI MISO P3.4 SPI A0
#define SPIMISODIR P3DIR
#define SPIMISOOUT P3OUT
#define SPIMISOIN  P3IN
#define SPIMISOREN P3REN

#undef MOSI
#undef MISO
#undef SCK

#define MOSI BIT3
#define MISO BIT4
#define SCK  BIT7
#define SS   BIT5

#undef TDI
#undef TDO
#undef TCK
#undef TMS

#define TDI MOSI
#define TDO MISO
#define TCK SCK
#define TMS SS

#define TCLK TDI

#define TST  BIT0
#define RST  BIT1

#undef SETMOSI
#undef CLRMOSI
#undef SETCLK
#undef CLRCLK
#undef READMISO

#define SETMOSI SPIMOSIOUT|=MOSI
#define CLRMOSI SPIMOSIOUT&=~MOSI
#define SETCLK SPICLKOUT|=SCK
#define CLRCLK SPICLKOUT&=~SCK
#define READMISO (SPIMISOIN&MISO?1:0)

//No longer works for Hope badge.
// SPI CS P3.5
#define SETSS P3OUT|=SS
#define CLRSS P3OUT&=~SS
#define DIRSS P3DIR|=SS

//Used for the Nordic port, !RST pin on regular GoodFET.
// Chip Enable P3.6
#define SETCE P3OUT|=BIT6
#define CLRCE P3OUT&=~BIT6
#define DIRCE P3DIR|=BIT6

#undef SETTMS
#undef CLRTMS
#undef SETTCK
#undef CLRTCK
#undef SETTDI
#undef CLRTDI

#define SETTMS SETSS
#define CLRTMS CLRSS
#define SETTCK SETCLK
#define CLRTCK CLRCLK
#define SETTDI SPIMOSIOUT|=TDI
#define CLRTDI SPIMOSIOUT&=~TDI

#undef SETTST
#undef CLRTST
#undef SETRST
#undef CLRRST

#define JTAGTSTDIR P6DIR
#define JTAGRSTDIR P6DIR

#define JTAGTSTOUT P6OUT
#define JTAGRSTOUT P6OUT

#define SETTST JTAGTSTOUT|=TST
#define CLRTST JTAGTSTOUT&=~TST
#define SETRST JTAGRSTOUT|=RST
#define CLRRST JTAGRSTOUT&=~RST

#define SETTCLK SETTDI
#define CLRTCLK CLRTDI

#undef SAVETCLK
#undef RESTORETCLK

#define SAVETCLK savedtclk=SPIMOSIOUT&TCLK;
#define RESTORETCLK if(savedtclk) SPIMOSIOUT|=TCLK; else SPIMOSIOUT&=~TCLK


// network byte order converters
#define htons(x) ((((uint16_t)(x) & 0xFF00) >> 8) | \
				 (((uint16_t)(x) & 0x00FF) << 8))
#define htonl(x) ((((uint32_t)(x) & 0xFF000000) >> 24) | \
				  (((uint32_t)(x) & 0x00FF0000) >> 8) | \
				  (((uint32_t)(x) & 0x0000FF00) << 8) | \
				  (((uint32_t)(x) & 0x000000FF) << 24))

#define ntohs htons
#define ntohl htonl

