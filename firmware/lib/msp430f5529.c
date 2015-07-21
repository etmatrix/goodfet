//! MSP430F2618 clock and I/O definitions

// Included by other 2xx ports, such as the 2274.
#include <msp430.h>
#include <sys/crtld.h>


#include "platform.h"

#include "dco_calib.h"


static void Init_FLL_Settle(unsigned int fsystem, unsigned int ratio);
static uint16_t SetVCore(uint8_t level);

#define XT1_XT2_PORT_SEL            P5SEL
#define XT1_ENABLE                  (BIT4 + BIT5)
#define XT2_ENABLE                  (BIT2 + BIT3)


#define PMM_STATUS_OK     0
#define PMM_STATUS_ERROR  1

#define MHZ                     1000000
#define MCLK_FREQ               (16*MHZ)
#define MCLK_MS_COUNT           (MCLK_FREQ/1000)

#define st(x)      do { x } while (__LINE__ == -1)

#define SELECT_MCLK_SMCLK(sources) st(UCSCTL4 = (UCSCTL4 & ~(SELM_7 + SELS_7)) | (sources);)

#ifndef __bis_SR_register
#define __bis_SR_register(x)    __asm__ __volatile__( "bis %0, r2" : :"i" ((uint16_t)x) );
#endif
#ifndef __bic_SR_register
#define __bic_SR_register(x)    __asm__ __volatile__( "bic %0, r2" : :"i" ((uint16_t)x) );
#endif

//! Receive a byte.
unsigned char serial1_rx()
{
    while (!(UCA1IFG & UCRXIFG));
    return UCA1RXBUF;
}

//! Transmit a byte on the second UART.
void serial1_tx(unsigned char x)
{
    while (!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = x;
}

//! Set the baud rate of the second uart.
void setbaud1(unsigned char rate)
{
    switch(rate){
       case 1://9600 baud
           UCA1BR1 = 0x06;
           UCA1BR0 = 0x82;
           break;
       case 2://19200 baud
          UCA1BR1 = 0x03;
          UCA1BR0 = 0x41;
          break;
       case 3://38400 baud
          UCA1BR1 = 0xa0;
          UCA1BR0 = 0x01;
          break;
       case 4://57600 baud
          UCA1BR1 = 0x1d;
          UCA1BR0 = 0x01;
          break;
       default:
       case 5://115200 baud
          UCA1BR0 = 0x8a;
          UCA1BR1 = 0x00;
          break;
    }
}

void msp430_init_uart1()
{
    P4SEL = BIT5 + BIT4;                    // P4.4,5 = USCI_A1 TXD/RXD
    UCA1CTL0 = 0x00;

    UCA1CTL1 |= UCSSEL_2;                     // SMCLK

    setbaud(5);//default baud, 115200

    UCA1MCTL = 0;                             // Modulation UCBRSx = 5
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}


//! Initialization is correct.
void msp430_init_dco_done() {}

//! Initialize the MSP430 clock.
void msp430_init_dco() 
{
    XT1_XT2_PORT_SEL |= XT1_ENABLE + XT2_ENABLE;
    // Set Vcore to accomodate for max. allowed system speed
    SetVCore(3);
    Init_FLL_Settle(MCLK_MS_COUNT, 488);
}

static void Init_FLL(unsigned int fsystem, unsigned int ratio) {
	unsigned int d, dco_div_bits;
	unsigned int mode = 0;

	// Save actual state of FLL loop control, then disable it. This is needed to
	// prevent the FLL from acting as we are making fundamental modifications to
	// the clock setup.
	unsigned int srRegisterState = READ_SR & SCG0;
	__bic_SR_register(SCG0); d
	= ratio;
	dco_div_bits = FLLD__2;        // Have at least a divider of 2

	if (fsystem > 16000) {
		d >>= 1;
		mode = 1;
	} else {
		fsystem <<= 1;               // fsystem = fsystem * 2
	}

	while (d > 512) {
		dco_div_bits = dco_div_bits + FLLD0;  // Set next higher div level
		d >>= 1;
	}

	UCSCTL0 = 0x0000;              // Set DCO to lowest Tap

	UCSCTL2 &= ~(0x03FF);          // Reset FN bits
	UCSCTL2 = dco_div_bits | (d - 1);

	if (fsystem <= 630)            //           fsystem < 0.63MHz
		UCSCTL1 = DCORSEL_0;
	else if (fsystem < 1250)      // 0.63MHz < fsystem < 1.25MHz
		UCSCTL1 = DCORSEL_1;
	else if (fsystem < 2500)      // 1.25MHz < fsystem <  2.5MHz
		UCSCTL1 = DCORSEL_2;
	else if (fsystem < 5000)      // 2.5MHz  < fsystem <    5MHz
		UCSCTL1 = DCORSEL_3;
	else if (fsystem < 10000)     // 5MHz    < fsystem <   10MHz
		UCSCTL1 = DCORSEL_4;
	else if (fsystem < 20000)     // 10MHz   < fsystem <   20MHz
		UCSCTL1 = DCORSEL_5;
	else if (fsystem < 40000)     // 20MHz   < fsystem <   40MHz
		UCSCTL1 = DCORSEL_6;
	else
		UCSCTL1 = DCORSEL_7;

	while (SFRIFG1 & OFIFG) {                          // Check OFIFG fault flag
		UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT2OFFG); // Clear OSC flaut Flags
		SFRIFG1 &= ~OFIFG;                             // Clear OFIFG fault flag
	}

	if (mode == 1) {                              		  // fsystem > 16000
		SELECT_MCLK_SMCLK(SELM__DCOCLK + SELS__DCOCLK);
		// Select DCOCLK
	} else {
		SELECT_MCLK_SMCLK(SELM__DCOCLKDIV + SELS__DCOCLKDIV);
		// Select DCODIVCLK
	}

	__bis_SR_register(srRegisterState);	                // Restore previous SCG0
}

static void Init_FLL_Settle(unsigned int fsystem, unsigned int ratio) {
	volatile unsigned int x = ratio * 32;

	Init_FLL(fsystem, ratio);

	while (x--) {
		__delay_cycles(30);
	}
}

static uint16_t SetVCoreUp(uint8_t level)
{
    uint16_t PMMRIE_backup, SVSMHCTL_backup, SVSMLCTL_backup;

    // The code flow for increasing the Vcore has been altered to work around
    // the erratum FLASH37.
    // Please refer to the Errata sheet to know if a specific device is affected
    // DO NOT ALTER THIS FUNCTION

    // Open PMM registers for write access
    PMMCTL0_H = 0xA5;

    // Disable dedicated Interrupts
    // Backup all registers
    PMMRIE_backup = PMMRIE;
    PMMRIE &= ~(SVMHVLRPE | SVSHPE | SVMLVLRPE | SVSLPE | SVMHVLRIE |
                SVMHIE | SVSMHDLYIE | SVMLVLRIE | SVMLIE | SVSMLDLYIE);
    SVSMHCTL_backup = SVSMHCTL;
    SVSMLCTL_backup = SVSMLCTL;

    // Clear flags
    PMMIFG = 0;

    // Set SVM highside to new level and check if a VCore increase is possible
    SVSMHCTL = SVMHE | SVSHE | (SVSMHRRL0 * level);

    // Wait until SVM highside is settled
    while ((PMMIFG & SVSMHDLYIFG) == 0) ;

    // Clear flag
    PMMIFG &= ~SVSMHDLYIFG;

    // Check if a VCore increase is possible
    if ((PMMIFG & SVMHIFG) == SVMHIFG){     // -> Vcc is too low for a Vcore increase
        // recover the previous settings
        PMMIFG &= ~SVSMHDLYIFG;
        SVSMHCTL = SVSMHCTL_backup;

        // Wait until SVM highside is settled
        while ((PMMIFG & SVSMHDLYIFG) == 0) ;

        // Clear all Flags
        PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);

        PMMRIE = PMMRIE_backup;             // Restore PMM interrupt enable register
        PMMCTL0_H = 0x00;                   // Lock PMM registers for write access
        return PMM_STATUS_ERROR;            // return: voltage not set
    }

    // Set also SVS highside to new level
    // Vcc is high enough for a Vcore increase
    SVSMHCTL |= (SVSHRVL0 * level);

    // Wait until SVM highside is settled
    while ((PMMIFG & SVSMHDLYIFG) == 0) ;

    // Clear flag
    PMMIFG &= ~SVSMHDLYIFG;

    // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;

    // Set SVM, SVS low side to new level
    SVSMLCTL = SVMLE | (SVSMLRRL0 * level) | SVSLE | (SVSLRVL0 * level);

    // Wait until SVM, SVS low side is settled
    while ((PMMIFG & SVSMLDLYIFG) == 0) ;

    // Clear flag
    PMMIFG &= ~SVSMLDLYIFG;
    // SVS, SVM core and high side are now set to protect for the new core level

    // Restore Low side settings
    // Clear all other bits _except_ level settings
    SVSMLCTL &= (SVSLRVL0 + SVSLRVL1 + SVSMLRRL0 + SVSMLRRL1 + SVSMLRRL2);

    // Clear level settings in the backup register,keep all other bits
    SVSMLCTL_backup &= ~(SVSLRVL0 + SVSLRVL1 + SVSMLRRL0 + SVSMLRRL1 + SVSMLRRL2);

    // Restore low-side SVS monitor settings
    SVSMLCTL |= SVSMLCTL_backup;

    // Restore High side settings
    // Clear all other bits except level settings
    SVSMHCTL &= (SVSHRVL0 + SVSHRVL1 + SVSMHRRL0 + SVSMHRRL1 + SVSMHRRL2);

    // Clear level settings in the backup register,keep all other bits
    SVSMHCTL_backup &= ~(SVSHRVL0 + SVSHRVL1 + SVSMHRRL0 + SVSMHRRL1 + SVSMHRRL2);

    // Restore backup
    SVSMHCTL |= SVSMHCTL_backup;

    // Wait until high side, low side settled
    while (((PMMIFG & SVSMLDLYIFG) == 0) && ((PMMIFG & SVSMHDLYIFG) == 0)) ;

    // Clear all Flags
    PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);

    PMMRIE = PMMRIE_backup;                 // Restore PMM interrupt enable register
    PMMCTL0_H = 0x00;                       // Lock PMM registers for write access

    return PMM_STATUS_OK;
}

/*******************************************************************************
 * \brief  Decrease Vcore by one level
 *
 * \param  level    Level to which Vcore needs to be decreased
 * \return status   Success/failure
 ******************************************************************************/

static uint16_t SetVCoreDown(uint8_t level)
{
    uint16_t PMMRIE_backup, SVSMHCTL_backup, SVSMLCTL_backup;

    // The code flow for decreasing the Vcore has been altered to work around
    // the erratum FLASH37.
    // Please refer to the Errata sheet to know if a specific device is affected
    // DO NOT ALTER THIS FUNCTION

    // Open PMM registers for write access
    PMMCTL0_H = 0xA5;

    // Disable dedicated Interrupts
    // Backup all registers
    PMMRIE_backup = PMMRIE;
    PMMRIE &= ~(SVMHVLRPE | SVSHPE | SVMLVLRPE | SVSLPE | SVMHVLRIE |
                SVMHIE | SVSMHDLYIE | SVMLVLRIE | SVMLIE | SVSMLDLYIE);
    SVSMHCTL_backup = SVSMHCTL;
    SVSMLCTL_backup = SVSMLCTL;

    // Clear flags
    PMMIFG &= ~(SVMHIFG | SVSMHDLYIFG | SVMLIFG | SVSMLDLYIFG);

    // Set SVM, SVS high & low side to new settings in normal mode
    SVSMHCTL = SVMHE | (SVSMHRRL0 * level) | SVSHE | (SVSHRVL0 * level);
    SVSMLCTL = SVMLE | (SVSMLRRL0 * level) | SVSLE | (SVSLRVL0 * level);

    // Wait until SVM high side and SVM low side is settled
    while ((PMMIFG & SVSMHDLYIFG) == 0 || (PMMIFG & SVSMLDLYIFG) == 0) ;

    // Clear flags
    PMMIFG &= ~(SVSMHDLYIFG + SVSMLDLYIFG);
    // SVS, SVM core and high side are now set to protect for the new core level

    // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;

    // Restore Low side settings
    // Clear all other bits _except_ level settings
    SVSMLCTL &= (SVSLRVL0 + SVSLRVL1 + SVSMLRRL0 + SVSMLRRL1 + SVSMLRRL2);

    // Clear level settings in the backup register,keep all other bits
    SVSMLCTL_backup &= ~(SVSLRVL0 + SVSLRVL1 + SVSMLRRL0 + SVSMLRRL1 + SVSMLRRL2);

    // Restore low-side SVS monitor settings
    SVSMLCTL |= SVSMLCTL_backup;

    // Restore High side settings
    // Clear all other bits except level settings
    SVSMHCTL &= (SVSHRVL0 + SVSHRVL1 + SVSMHRRL0 + SVSMHRRL1 + SVSMHRRL2);

    // Clear level settings in the backup register, keep all other bits
    SVSMHCTL_backup &= ~(SVSHRVL0 + SVSHRVL1 + SVSMHRRL0 + SVSMHRRL1 + SVSMHRRL2);

    // Restore backup
    SVSMHCTL |= SVSMHCTL_backup;

    // Wait until high side, low side settled
    while (((PMMIFG & SVSMLDLYIFG) == 0) && ((PMMIFG & SVSMHDLYIFG) == 0)) ;

    // Clear all Flags
    PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);

    PMMRIE = PMMRIE_backup;                // Restore PMM interrupt enable register
    PMMCTL0_H = 0x00;                      // Lock PMM registers for write access
    return PMM_STATUS_OK;                  // Return: OK
}

static uint16_t SetVCore(uint8_t level)
{
    uint16_t actlevel;
    uint16_t status = 0;

    level &= PMMCOREV_3;                   // Set Mask for Max. level
    actlevel = (PMMCTL0 & PMMCOREV_3);     // Get actual VCore
                                           // step by step increase or decrease
    while ((level != actlevel) && (status == 0)) {
        if (level > actlevel){
            status = SetVCoreUp(++actlevel);
        }
        else {
            status = SetVCoreDown(--actlevel);
        }
    }

    return status;
}

