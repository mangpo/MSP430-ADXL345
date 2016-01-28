//  Interfacing ADXL345 accelerometer with MSP430G2553 with I2C communication
//  and printing restuls to serial port using UART.
//
//                                /|\  /|\
//               ADXL345          10k  10k     MSP430G2xx3
//                slave            |    |        master
//             -----------------   |    |  -----------------
//            |              SDA|<-|---+->|P1.7/UCB0SDA  XIN|-
//            |                 |  |      |                 |
//            |                 |  |      |             XOUT|-
//            |              SCL|<-+----->|P1.6/UCB0SCL     |
//            |                 |         |                 |
//
//  For Sparkfun ADXL345,
//    * connect SDO to ground
//    * connect CS to VCC
//
//  Original Code Made By :
//  Prof. Ravi Butani
//  Marwadi Education Foundation, Rajkot GUJARAT-INDIA
//  ravi.butani@marwadieducation.edu.in
//  https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/260094
//
//  Modified By :
//  Phitchaya Mangpo Phothilimthana
//  mangpo@eecs.berkeley.edu

//******************************************************************************
#include <msp430g2553.h>

#define NUM_BYTES_TX 2  
#define NUM_BYTES_RX 6
#define ADXL_345     0x53

int RXByteCtr, RPT_Flag = 0,x1,y1,z1;       // enables repeated start when 1
volatile unsigned char RxBuffer[8];         // Allocate 6 byte of RAM
unsigned char *PRxData;                     // Pointer to RX data
unsigned char TXByteCtr, RX = 0;
unsigned char MSData[3];
unsigned int wdtCounter = 0;

void Setup_TX(unsigned char);
void Setup_RX(unsigned char);
void Transmit(unsigned char,unsigned char);
void TransmitOne(unsigned char);
void Receive(void);
void Setup_UART();
void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength);
void UARTSendInt(unsigned int x);

int main(void)
{
  // LED
  P1DIR |= BIT0;
  P1OUT |= BIT0;

  // UART
  BCSCTL1 = CALBC1_1MHZ; // Set DCO to 1MHz
  DCOCTL = CALDCO_1MHZ; // Set DCO to 1MHz

  // Configure hardware UART
  P1SEL |= BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD
  P1SEL2 |= BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD
    
  Setup_UART();
  UARTSendArray("Hello\n", 6);
  __delay_cycles(1000);

  // ADXL345
  P1SEL  |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
  P1SEL2 |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0

  // Init sequence for ADXL345
  //Transmit process
  Setup_TX(ADXL_345);
  RPT_Flag = 0;
  Transmit(0x2D,0x00);                    // STUCK
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

  //Transmit process
  Setup_TX(ADXL_345);
  RPT_Flag = 0;
  Transmit(0x2D,0x10);
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

  //Transmit process
  Setup_TX(ADXL_345);
  RPT_Flag = 0;
  Transmit(0x2D,0x08);
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  
  // Un-comment next block to change range of ADXL345
  /*
    Setup_TX(ADXL_345);
    RPT_Flag = 1;
    Transmit(0x31,0x01);                            // Range Select at add 0x31 write 0x00 for 2g(default)/ 0x01 for 4g/ 0x02 for 8g/ 0x03 for 16g
    while (UCB0CTL1 & UCTXSTP);         // Ensure stop condition got sent
  */

  /* Set watchdog timer interval to 1000ms (requires external crystal to work) */
  WDTCTL = WDT_ADLY_1000;

  /* "Interrupt enable 1" for the Watchdog Timer interrupt */
  IE1 |= WDTIE;

  /* Go into low power mode 3, general interrupts enabled */
  __bis_SR_register( LPM3_bits + GIE );

  /* Do nothing...forever */
  for( ; ; ) { }
}


/* Watchdog Timer interrupt service routine.  The function prototype
 *  tells the compiler that this will service the Watchdog Timer, and
 *  then the function follows.
 *    */
void watchdog_timer(void) __attribute__((interrupt(WDT_VECTOR)));
void watchdog_timer(void)
{
    wdtCounter++;
    /* Count 1 interrupts x 1000ms = 1000ms, or one second */
    if(wdtCounter == 1) {
      //Transmit process
      Setup_TX(ADXL_345);
      RPT_Flag = 0;
      TransmitOne(0x32);                      // Request Data from ADXL345
      while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

      //Receive process
      Setup_RX(ADXL_345);
      Receive();
      while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

      x1 = (((int)RxBuffer[1]) << 8) | RxBuffer[0];
      y1 = (((int)RxBuffer[3]) << 8) | RxBuffer[2];
      z1 = (((int)RxBuffer[5]) << 8) | RxBuffer[4];
    
      // now You have XYZ axis reading in x1,x2,x3 variable....Bingo... you can play with it as you like....
      // Below if sense x and y angle and Red led is on if its more then 45 or less then -45...
      // you can put your own condition here...

      if ((x1 > 128) || (y1 > 128) || (x1 < -128) || (y1 < -128)) {
        P1OUT |= BIT0; // red led on
      }
      else {
        P1OUT &= ~BIT0; // red led off
      }

      Setup_UART();
      UARTSendArray("sample\n", 7);
      UARTSendInt(x1);
      UARTSendInt(y1);
      UARTSendInt(z1);
      /* Reset the counter for the next blink */
      wdtCounter = 0;
    }
}

//-------------------------------------------------------------------------------
// The USCI_B0 data ISR is used to move received data from the I2C slave
// to the MSP430 memory. It is structured such that it can be used to receive
// any 2+ number of bytes by pre-loading RXByteCtr with the byte count.
//-------------------------------------------------------------------------------
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  
  if(RX == 1){                              // Master Recieve?
    RXByteCtr--;                              // Decrement RX byte counter
    if (RXByteCtr)
      {
        *PRxData++ = UCB0RXBUF;                 // Move RX data to address PRxData
      }
    else
      {
        if(RPT_Flag == 0)
          UCB0CTL1 |= UCTXSTP;                // No Repeated Start: stop condition
        if(RPT_Flag == 1){                    // if Repeated Start: do nothing
          RPT_Flag = 0;
        }
        *PRxData++ = UCB0RXBUF;                   // Move final RX data to PRxData
        __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
      }}
  else{                                     // Master Transmit
    if (TXByteCtr)                        // Check TX byte counter
      {
        UCB0TXBUF = MSData[TXByteCtr];          // Load TX buffer
        TXByteCtr--;                            // Decrement TX byte counter
      }
    else
      {
        /* UCB0CTL1 |= UCTXSTP;                    // I2C stop condition */
        /* IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag */
        if(RPT_Flag == 1){
          RPT_Flag = 0;
          TXByteCtr = NUM_BYTES_TX;                // Load TX byte counter
          __bic_SR_register_on_exit(CPUOFF);
        }
        else{
          UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
          IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
      }
  }
}

void Setup_UART() {
  _DINT();
  IE2 &= ~UCB0RXIE; 
  IE2 &= ~UCB0TXIE; 
  UCA0CTL1 |= UCSSEL_2; // Use SMCLK
  UCA0BR0 = 104; // Set baud rate to 9600 with 1MHz clock (Data Sheet 15.3.13)
  UCA0BR1 = 0; // Set baud rate to 9600 with 1MHz clock
  UCA0MCTL = UCBRS0; // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine
  IE2 |= UCA0RXIE; // Enable USCI_A0 RX interrupt
}

void Setup_TX(unsigned char Dev_ID){
  _DINT();
  RX = 0;
  IE2 &= ~UCA0RXIE; // Disable USCI_A0 RX interrupt
  IE2 &= ~UCB0RXIE; 
  while (UCB0CTL1 & UCTXSTP);               // Ensure stop condition got sent// Disable RX interrupt
  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
  UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
  UCB0BR1 = 0;
  UCB0I2CSA = Dev_ID;                       // Slave Address is 048h
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  IE2 |= UCB0TXIE;                          // Enable TX interrupt
}

void Setup_RX(unsigned char Dev_ID){
  _DINT();
  RX = 1;
  IE2 &= ~UCA0RXIE; // Disable USCI_A0 RX interrupt
  IE2 &= ~UCB0TXIE; 
  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
  UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
  UCB0BR1 = 0;
  UCB0I2CSA = Dev_ID;                       // Slave Address is 048h
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  IE2 |= UCB0RXIE;                          // Enable RX interrupt
}

void Transmit(unsigned char Reg_ADD,unsigned char Reg_DAT){
  MSData[2]= Reg_ADD;
  MSData[1]= Reg_DAT;
  TXByteCtr = NUM_BYTES_TX;                  // Load TX byte counter
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
  __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

void TransmitOne(unsigned char Reg_ADD){
  MSData[1]= Reg_ADD;
  TXByteCtr = 1;                  // Load TX byte counter
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
  __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

void Receive(void){
  PRxData = (unsigned char *)RxBuffer;    // Start of RX buffer
  RXByteCtr = NUM_BYTES_RX;             // Load RX byte counter
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  UCB0CTL1 |= UCTXSTT;                    // I2C start condition
  __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength){

  while(ArrayLength--){ // Loop until StringLength == 0 and post decrement
    while(!(IFG2 & UCA0TXIFG)); // Wait for TX buffer to be ready for new data
    UCA0TXBUF = *TxArray; //Write the character at the location specified py the pointer
    TxArray++; //Increment the TxString pointer to point to the next character
  }
  IFG2 &= ~UCA0TXIFG;                     // Clear USCI_A0 int flag
}

void UARTSendInt(unsigned int x){
  unsigned char buff[10];
  unsigned char data[10];
  unsigned char index = 0, i = 0;

  while(x > 0) {
    unsigned char val = x % 16;
    if(val < 10)
      buff[index] = 48+val;
    else
      buff[index] = 97+val-10;
    index++;
    x /= 16;
  }
  buff[index] = '\n';

  while(index > 0) {
    index--;
    data[i] = buff[index];
    i++;
  }

  if(i==0) {
    data[0] = '0';
    i++;
  }
  data[i] = '\n';
  UARTSendArray(data, i+1);
}
