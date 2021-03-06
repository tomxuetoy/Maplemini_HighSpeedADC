// pin 19 (Yellow): CLK
// pin 20 (Orange): SI of x axis
// pin 18 (Blue)  : SI of y axis
// Red            : 5V
// White          : Gnd

#include <wirish/wirish.h>
#include <libmaple/adc.h>

#define COM SerialUSB // use this for Maple

unsigned long start = 0;
unsigned long stop = 0;
unsigned long sampleCount = 100;
unsigned long pixelIndex = 0;
uint32 pixelVal_x[129];
uint32 pixelVal_y[129];

int SI_x = 20;
int SI_y = 18;
int CLK = 19;
int AO_x = 3;   // channel 8
int AO_y = 4;   // channel 7

// GPIO
gpio_dev *CLK_dev;
uint8 CLK_pin;
gpio_dev *SI_x_dev;
uint8 SI_x_pin;
gpio_dev *SI_y_dev;
uint8 SI_y_pin;

uint32 adc_sequence = 0; //Temporary
uint8 adc_length = 1; //The number of channels to be converted per ADC channel
uint8 ADC1_Sequence[] = { /*The ADC1 channels sequence, left to right*/
  8 }; /* channel 8 - pin 4. Set the sequence 1-6 for SQR3 (left will be first). Must top up to all 6 channels with zeros */
uint8 ADC2_Sequence[] = { /*The ADC2 channels sequence, left to right*/
  7 };  /* channel 7 - pin 3 */

/*
* calc_adc_sequence(ADCx_Sequence) converts the SQR3 6 channels' (each ADC1 and ADC2) list into
 * a valid 6 X 5=30 bits sequence format and returns that 30 bits number.
 * For more channels, repeat the same for SQR2, SQR1. (For SQR1 4 channels only!)
 */
uint32 calc_adc_sequence(uint8 adc_sequence_array[6])
{
  adc_sequence = 0;
  for (int i = 0; i<6; i++) // There are 6 available sequences in each SQR3 SQR2, and 4 in SQR1.
  {
    /*This function converts the array into one number by multiplying each 5-bits channel numbers
     by multiplications of 2^5
     */
    adc_sequence = adc_sequence + adc_sequence_array[i] * pow(2, (i * 5));
  }
  return adc_sequence;
} //end calc_adc_sequence

void setup()
{
  // the following line is needed for Maple
  pinMode(AO_x, INPUT_ANALOG);
  pinMode(AO_y, INPUT_ANALOG);
  pinMode(CLK, OUTPUT);
  pinMode(SI_x, OUTPUT);
  pinMode(SI_y, OUTPUT);
  // 下面的代码可能不全，会导致串口无输出
  adc_init(ADC1); //rcc_clk_enable(ADC1->clk_id), Must be the first adc command!
  adc_init(ADC2);

  adc_enable(ADC1); //ADC_CR2_ADON_BIT = 1
  adc_enable(ADC2);

  adc_calibrate(ADC1); //Optional
  adc_calibrate(ADC2);

  // 采样率优化的关键寄存器参数
  adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
  adc_set_sample_rate(ADC1, ADC_SMPR_1_5);
  adc_set_sample_rate(ADC2, ADC_SMPR_1_5);

  adc_set_exttrig(ADC1, 1); //External trigger must be Enabled for both ADC.
  adc_set_exttrig(ADC2, 1); //External trigger must be Enabled for both ADC.
  adc_set_extsel(ADC1, ADC_ADC12_SWSTART); //External trigger Event
  adc_set_extsel(ADC2, ADC_ADC12_SWSTART); //External trigger Event

  adc_set_reg_seqlen(ADC1, adc_length); //The number of channels to be converted.
  adc_set_reg_seqlen(ADC2, adc_length);
  ADC1->regs->SQR3 |= calc_adc_sequence(ADC1_Sequence);
  ADC2->regs->SQR3 |= calc_adc_sequence(ADC2_Sequence);  

  // right alligned
  ADC1->regs->CR2 &= (0xfffff7ff);
  ADC2->regs->CR2 &= (0xfffff7ff);

  // DMA bit set, the key!!! I was stucked here for several days.
  // RM0008 page 219: Note: In dual ADC mode, to read the slave converted data on the master data register, the DMA 
  // bit must be enabled even if it is not used to transfer converted regular channel data.
  ADC1->regs->CR2 |= (1<<8);
  ADC2->regs->CR2 |= (1<<8);

  ADC1->regs->CR1 |= (6 << 16);       // 0b0110: Regular simultaneous mode only
//  ADC2->regs->CR1 |= (6 << 16);     // For CR1 dual mode, These bits are reserved in ADC2 and ADC3.

  // digitalWrite() preparation work
  // CLK
  CLK_dev = PIN_MAP[CLK].gpio_device;
  CLK_pin = PIN_MAP[CLK].gpio_bit;
  // SI x
  SI_x_dev = PIN_MAP[SI_x].gpio_device;
  SI_x_pin = PIN_MAP[SI_x].gpio_bit;
  // SI y
  SI_y_dev = PIN_MAP[SI_y].gpio_device;
  SI_y_pin = PIN_MAP[SI_y].gpio_bit;
}

void sampleSensor()
{
  pixelIndex = 0;

  // digitalWrite(CLK, LOW);
  CLK_dev->regs->BSRR = (1U << CLK_pin)<<16;    // CLK set low
  SI_x_dev->regs->BSRR = (1U << SI_x_pin)<<16;  // SI x set low
  SI_y_dev->regs->BSRR = (1U << SI_y_pin)<<16;  // SI y set low
  SI_x_dev->regs->BSRR = (1U << SI_x_pin);      // SI x set high
  SI_y_dev->regs->BSRR = (1U << SI_y_pin);      // SI y set high
  CLK_dev->regs->BSRR = (1U << CLK_pin);        // CLK set high

  // start the 1st ADC sample and conversion
  ADC1->regs->CR2 |= ADC_CR2_SWSTART | ADC_CR2_CONT_BIT;
  while (!(ADC1->regs->SR & ADC_SR_EOC))
    ;
//  pixelVal_x[pixelIndex] = (uint16)(ADC1->regs->DR & ADC_DR_DATA);
  pixelVal_x[pixelIndex] = (uint16)(ADC1->regs->DR);
  pixelVal_y[pixelIndex] = ((uint32)(ADC1->regs->DR & ADC_DR_ADC2DATA))>>16;

  SI_x_dev->regs->BSRR = (1U << SI_x_pin)<<16;  // SI x set low
  SI_y_dev->regs->BSRR = (1U << SI_y_pin)<<16;  // SI y set low
  CLK_dev->regs->BSRR = (1U << CLK_pin)<<16;    // CLK set low

  while (pixelIndex < 128)
  {
    pixelIndex++;

    // digitalWrite(CLK, HIGH);
    CLK_dev->regs->BSRR = (1U << CLK_pin);

    // pixelVal_x[pixelIndex] = analogRead(AO_x);
    ADC1->regs->CR2 |= ADC_CR2_SWSTART | ADC_CR2_CONT_BIT;
    while (!(ADC1->regs->SR & ADC_SR_EOC))
      ;
    pixelVal_x[pixelIndex] = (uint16)(ADC1->regs->DR);
//    pixelVal_x[pixelIndex] = (uint16)(ADC1->regs->DR & ADC_DR_DATA);
    pixelVal_y[pixelIndex] = ((uint32)(ADC1->regs->DR & ADC_DR_ADC2DATA))>>16;
    // the last one: pixelIndex = 128

    // digitalWrite(CLK, LOW);
    CLK_dev->regs->BSRR = (1U << CLK_pin)<<16;
  }
}

void loop()
{
  COM.println("\nStarting loops:");

  start = micros();
  for(int m=0;m<sampleCount;m++)
  {
      sampleSensor();
      sampleSensor();
  }
  stop = micros();

  COM.println("Stop loops:");
  COM.print("Elapsed Time: ");
  COM.print(stop - start);
  COM.print(" us (for ");
  COM.print(sampleCount*129*2);
  COM.println(" analog reads)");
  COM.print((stop-start)/(double)(sampleCount*129*2));
  COM.print(" us (for 1 sample) ");
  COM.print((stop-start)/(double)(sampleCount*2));
  COM.print(" us (for 1 sensor) ");

  COM.println(" pixelVal_x = ");
  for(int k=0;k<128;k++)
  {
    COM.print(pixelVal_x[k]);
    COM.print("  ");
    if(k % 20 == 0)
      COM.println("");
  }

  COM.println("\n pixelVal_y = ");
  for(int k=0;k<128;k++)
  {
    COM.print(pixelVal_y[k]);
    COM.print("  ");
    if(k % 20 == 0)
      COM.println("");
  }

  COM.print("\n PCLK2 = ");
  COM.println(PCLK2);
  COM.print("\n ADC1->regs->CR1 = ");
  COM.println(ADC1->regs->CR1);
  COM.print("\n ADC2->regs->CR1 = ");
  COM.println(ADC2->regs->CR1);
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
// 下面的代码缺失的话，下载后maple mini串口设备消失，无法调试
__attribute__((constructor)) void premain() {
  init();
}

int main(void) {
  setup();
  while (true) {
    loop();
  }
  return 0;
}

//Starting loops:
//Stop loops:
//Elapsed Time: 30897 us (for 25800 analog reads)
//1.20 us (for 1 sample) 154.49 us (for 1 sensor)  pixelVal_x =
//619
//604  610  601  612  597  603  599  607  599  605  598  610  609  601  600  614  604  601  605  603
//599  605  603  593  601  608  603  609  614  611  611  602  610  611  600  611  605  609  606  606
//605  606  609  607  608  598  596  610  610  605  598  607  602  608  607  607  606  603  607  599
//601  605  607  614  596  608  593  614  602  615  602  600  595  610  606  612  591  602  603  611
//607  609  603  611  603  612  598  595  591  613  600  625  598  615  612  620  607  615  610  617
//604  610  600  615  604  615  604  616  599  617  595  615  604  612  606  612  601  610  598  609
//596  608  599  604  600  602  608
//                                    pixelVal_y =
//616
//605  599  590  609  602  597  600  611  596  602  595  612  602  602  603  608  602  601  591  617
//600  595  593  596  611  614  618  613  595  620  614  609  593  604  600  610  597  610  600  604
//619  621  565  543  558  597  614  614  606  612  603  613  605  613  607  614  610  614  607  615
//611  615  604  622  614  590  586  602  617  634  591  608  608  610  599  603  604  614  604  603
//607  619  614  615  606  616  619  621  634  632  612  613  603  606  613  621  622  626  627  622
//624  638  631  638  646  639  636  634  628  610  586  593  624  614  605  617  607  611  616  617
//617  625  584  566  564  606  639
//                                    PCLK2 = 72000000

// ADC1->regs->CR1 = 393216

// ADC2->regs->CR1 = 0
