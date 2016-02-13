//  0 gesture classifier

//******************************************************************************
#include <msp430g2553.h>
#include "fixed-point.h"

#define true 1
#define false 0

volatile unsigned char RxBuffer[6];         // Allocate 6 byte of RAM
int pass = 0;

fp_t acc[3];
fp_t dir_filter_ref[3];
  
fp_t f[8];
fp_t s[8]  = { 12990 , -321 , -7587, 12990 , -321 , -7587, 12990 , -321};
char started = false;

void Setup_UART();
void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength);
void hmm();

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // LED
  P1DIR |= BIT0;
  /* P1OUT |= BIT0; */

  // UART
  BCSCTL1 = CALBC1_1MHZ; // Set DCO to 1MHz
  DCOCTL = CALDCO_1MHZ; // Set DCO to 1MHz

  // Configure hardware UART
  P1SEL |= BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD
  P1SEL2 |= BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD
    
  Setup_UART();

  int i;
  while(true) {
    UARTSendArray("a\n",2);
    for(i=0;i<100;i++) {
      acc[0] = ((((int)RxBuffer[1]) << 8) | RxBuffer[0]) << 6;
      acc[1] = ((((int)RxBuffer[3]) << 8) | RxBuffer[2]) << 6;
      acc[2] = ((((int)RxBuffer[5]) << 8) | RxBuffer[4]) << 6;
      hmm();
    }
    UARTSendArray("b\n",2);
  }
  //__delay_cycles(1000);
  __bis_SR_register( LPM3_bits + GIE );
  for( ; ; ) { }
}

void Setup_UART() {
  _DINT();
  UCA0CTL1 |= UCSSEL_2; // Use SMCLK
  UCA0BR0 = 104; // Set baud rate to 9600 with 1MHz clock (Data Sheet 15.3.13)
  UCA0BR1 = 0; // Set baud rate to 9600 with 1MHz clock
  UCA0MCTL = UCBRS0; // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine
  IE2 |= UCA0RXIE; // Enable USCI_A0 RX interrupt
}

void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength){

  while(ArrayLength--){ // Loop until StringLength == 0 and post decrement
    while(!(IFG2 & UCA0TXIFG)); // Wait for TX buffer to be ready for new data
    UCA0TXBUF = *TxArray; //Write the character at the location specified py the pointer
    TxArray++; //Increment the TxString pointer to point to the next character
  }
  IFG2 &= ~UCA0TXIFG;                     // Clear USCI_A0 int flag
}

//-------------------------------------------------------------------------------
// HMM
//-------------------------------------------------------------------------------
char filter(){
  fp_t abs = fp_add(fp_add(fp_mul(acc[0], acc[0]),
                           fp_mul(acc[1], acc[1])),
                    fp_mul(acc[2], acc[2]));
  ////////////////////////////////////////////////////////////////////////////////
  //idle state filter
  if (!(fp_cmp(abs, 10485)==1 ||
        fp_cmp(abs, 8847)==-1)) { // if between 0.01 - 0.09
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // def = directional equivalence filter
  fp_t def_sensitivity = 2621;
  if (fp_cmp(acc[0], fp_sub(dir_filter_ref[0], def_sensitivity))==-1 ||
      fp_cmp(acc[0], fp_add(dir_filter_ref[0], def_sensitivity))== 1 ||
      fp_cmp(acc[1], fp_sub(dir_filter_ref[1], def_sensitivity))==-1 ||
      fp_cmp(acc[1], fp_add(dir_filter_ref[1], def_sensitivity))== 1 ||
      fp_cmp(acc[2], fp_sub(dir_filter_ref[2], def_sensitivity))==-1 ||
      fp_cmp(acc[2], fp_add(dir_filter_ref[2], def_sensitivity))==1) {
    dir_filter_ref[0] = acc[0];
    dir_filter_ref[1] = acc[1];
    dir_filter_ref[2] = acc[2];
    pass++;
    return true;
  }
  return true;
}

//The quantizer, maps accelerometer readings to a set of integers.
char derive_group(){
  fp_t a, b, c, d;
  fp_t minDist = 0x7fff; //0x7fff;
  char minGroup=0;
  fp_t *ref;
  char i;

  for (i = 0; i < 14; i++){
    ref = s;
    a = fp_sub(ref[0], acc[0]);
    b = fp_sub(ref[1], acc[1]);
    c = fp_sub(ref[2], acc[2]);
    d = fp_add(fp_add(fp_mul(a,a), fp_mul(b,b)), fp_mul(c,c));
    if (fp_cmp(d, minDist) == -1){
      minDist = d;
      minGroup = i;
    }
  }
  /* UARTSendArray("group=", 6); */
  /* UARTSendInt(minGroup); */
  return minGroup;
}

//Performs the next iteration of the HMM forward algorithm
fp_t forward_proc_inc(char o){
  fp_t ord = 0;
  fp_t sum;
  char k,l;

  if (started == false){
    for (l = 0; l < 8; l++){
      f[l] = s[l]; // pi*b
      //f[l] = b[o][l];
    }
    for (l = 1; l < 8; l++){
      f[l] = 0;
    }
    return 0;
  }else{
    for (k = 0; k < 8; k++){
      sum = 0;
      for (l = 0; l < 8; l++){
        //sum = fp_add(sum, fp_mul(s[l], a[(l<<3) + k]));
        sum = fp_add(sum, fp_mul(s[l], s[k]));
      }
      f[k] = fp_mul(sum, s[k]);
      //f[k] = fp_mul(sum, b[o][k]);
      ord |= f[k];
    }
  }
  return ord;
}

//int pass = 0;

//Called with each accelerometer reading
void input_reading(){
  fp_t ord = 0;
  char i,l;
  if (filter()){
    //pass++;
    for (i = 0; i < 2; i++){
      char group = derive_group();
      ord |= forward_proc_inc(group);
    }

    //counts the number of bits we can shift left by - the leading zeros
    char n = 0;
    while(ord > 0){
      n += 1;
      ord = ord << 1;
    }
    if (n>4) {
      n -= 4;
    }
    else {
      n = 0;
    }
    for (l = 0; l < 8; l++) {
      s[l] = f[l] << n;
      f[l] = s[l] << n;
    }
    if (started == false) started = true;
    // constantly moving ~35 s for 1000 iter   => 29 iter/s => 34 ms
    // one gestuer => 5 s
  }
}

int iter = 0;
void hmm() {
  input_reading();
}
