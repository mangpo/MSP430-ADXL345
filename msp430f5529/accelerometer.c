#include <msp430f5529.h>
//#include <stdio.h>

#define NUM_BYTES_TX 2
#define NUM_BYTES_RX 6
#define ADXL_345     0x53

int RXByteCtr, x1,y1,z1;       // enables repeated start when 1
volatile unsigned char RxBuffer[6];         // Allocate 6 byte of RAM
unsigned char *PRxData;                     // Pointer to RX data
unsigned char TXByteCtr, RX = 0;
unsigned char MSData[2];

void Setup_TX(unsigned char);
void Setup_RX(unsigned char);
void Transmit(unsigned char,unsigned char);
void TransmitOne(unsigned char);
void Receive(void);
//void Setup_UART();
//void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength);

/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    // LED
    P1DIR |= BIT0;
    P1OUT |= BIT0;

    // Configure hardware UART: RXD 1.1->3.4  TXD 1.2->3.3
    //P3SEL  |= BIT3 + BIT4 ; // P2.5 = RXD, P2.4=TXD

    // ADXL345: MISO (SCL) 1.6->3.1, MOSI (SDA) 1.7->3.0
    P3SEL  |= BIT0 + BIT1;                     // Assign I2C pins to USCI_B0
    //printf("init\n");

    // Init sequence for ADXL345
    //Transmit process
    Setup_TX(ADXL_345);
    Transmit(0x2D,0x00);                    // STUCK
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    //Transmit process
    Setup_TX(ADXL_345);
    Transmit(0x2D,0x10);
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    //Transmit process
    Setup_TX(ADXL_345);
    Transmit(0x2D,0x08);
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    long long i;

    while(1){
      //printf("0\n");
      //Setup_UART();
      //UARTSendArray("Hello\n", 6);
      //UARTSendArray("0\n", 2);

    // loop to measure completion time
    for(i=0;i<1000;i++) { // ~ 15 sec

    //while(1){
      //Transmit process
      Setup_TX(ADXL_345);
      TransmitOne(0x32);                                   // Request Data from ADXL345 in 2g Range 10Bit resolution
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

      /* Setup_UART(); */
      /* UARTSendArray("sample\n", 7); */
      /* UARTSendInt(x1); */
      /* UARTSendInt(y1); */
      /* UARTSendInt(z1); */
      //printf("%d %d %d\n", x1, y1, z1);
      //__delay_cycles(1000000);  // delay 1 sec
      // you can change by changing delay

    }
    //Setup_UART();
    //UARTSendArray("Hello\n", 6);
    //UARTSendArray("1\n", 2);
    //printf("1\n");
    //__delay_cycles(1000);
    //P1OUT ^= BIT0;
    }

    /* Go into low power mode 3, general interrupts enabled */
    __bis_SR_register( LPM3_bits + GIE );

    /* Do nothing...forever */
    for( ; ; ) { }
}

/*
void Setup_UART() {
  //UCB0IE &= ~UCB0RXIE;
  //UCB0IE &= ~UCB0TXIE;
	UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
	UCA0CTL1 |= UCSSEL_2;                     // SMCLK
	UCA0BR0 = 6;                              // 1MHz 9600 (see User's Guide)
	UCA0BR1 = 0;                              // 1MHz 9600
	UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
  UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine
  UCA0IE |= UCRXIE; //UCA0IE; // Enable USCI_A0 RX interrupt
}

void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength){

  while(ArrayLength--){ // Loop until StringLength == 0 and post decrement
    while(!(UCA0IFG & UCTXIFG)); // Wait for TX buffer to be ready for new data
    UCA0TXBUF = *TxArray; //Write the character at the location specified py the pointer
    TxArray++; //Increment the TxString pointer to point to the next character
  }
  UCA0IFG &= ~UCTXIFG;                     // Clear USCI_A0 int flag
}*/

//-------------------------------------------------------------------------------
// The USCI_B0 data ISR is used to move received data from the I2C slave
// to the MSP430 memory. It is structured such that it can be used to receive
// any 2+ number of bytes by pre-loading RXByteCtr with the byte count.
//-------------------------------------------------------------------------------
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  if(RX == 1){                              // Master Recieve?
    RXByteCtr--;                              // Decrement RX byte counter
    if (RXByteCtr)
      {
        *PRxData++ = UCB0RXBUF;                 // Move RX data to address PRxData
      }
    else
      {
        UCB0CTL1 |= UCTXSTP;                // No Repeated Start: stop condition
        *PRxData++ = UCB0RXBUF;                   // Move final RX data to PRxData
        __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
      }}
  else{                                     // Master Transmit
    if (TXByteCtr)                        // Check TX byte counter
      {
        TXByteCtr--;                            // Decrement TX byte counter
        UCB0TXBUF = MSData[TXByteCtr];          // Load TX buffer
      }
    else
      {
          UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
          UCB0IFG &= ~UCTXIFG;                     // Clear USCI_B0 TX int flag
          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
      }
  }
}

void Setup_TX(unsigned char Dev_ID){
  _DINT();
  RX = 0;
  UCA0IE &= ~UCRXIE; // Disable USCI_A0 RX interrupt
  UCB0IE &= ~UCRXIE;
  while (UCB0CTL1 & UCTXSTP);               // Ensure stop condition got sent// Disable RX interrupt
  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
  UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
  UCB0BR1 = 0;
  UCB0I2CSA = Dev_ID;                       // Slave Address is 048h
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  UCB0IE |= UCTXIE;                          // Enable TX interrupt
}

void Setup_RX(unsigned char Dev_ID){
  _DINT();
  RX = 1;
  UCA0IE &= ~UCRXIE; // Disable USCI_A0 RX interrupt
  UCB0IE &= ~UCTXIE;
  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
  UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
  UCB0BR1 = 0;
  UCB0I2CSA = Dev_ID;                       // Slave Address is 048h
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  UCB0IE |= UCRXIE;                          // Enable RX interrupt
}

void Transmit(unsigned char Reg_ADD,unsigned char Reg_DAT){
  MSData[1]= Reg_ADD;
  MSData[0]= Reg_DAT;
  TXByteCtr = NUM_BYTES_TX;                  // Load TX byte counter
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
  __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

void TransmitOne(unsigned char Reg_ADD){
  MSData[0]= Reg_ADD;
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
