/*
* The original code is By leaflabs.com forums member "Stephen from NYC" for Analog Read (Pin 0) Speed Test
* his code link is here: http://forums.leaflabs.com/topic.php?id=154
* Tom Xue made some big change to test the high speed ADC of Maple mini @ 2013-12-22 17:53:44
*/
#include <wirish/wirish.h>
#include <libmaple/adc.h>

#define COMM SerialUSB // use this for Maple
// #define COMM Serial // use this for Arduino
unsigned long start = 0;
unsigned long stop = 0;
unsigned long counter = 0;
unsigned long limit = 100000;
int outPin = 18;
int analogPin = 3;
int val = 0;

void setup()
{
    // the following line is needed for Maple
    pinMode(analogPin, INPUT_ANALOG);
    pinMode(outPin, OUTPUT);
    // 下面的代码可能不全，会导致串口无输出
    // adc_init(ADC1); //rcc_clk_enable(ADC1->clk_id), Must be the first adc command!
    // adc_init(ADC2);
    //
    // adc_enable(ADC1); //ADC_CR2_ADON_BIT = 1
    // adc_enable(ADC2);
    //
    // adc_calibrate(ADC1); //Optional
    // adc_calibrate(ADC2);
    // 采样率优化的关键寄存器参数
    adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
    adc_set_sample_rate(ADC1, ADC_SMPR_1_5);
    // the following line is needed for Arduino
    // COMM.begin(57600);
}

void loop()
{
    COMM.println("\nStarting loops:");
    counter = 0;

    // digitalWrite() preparation work
    gpio_dev *gpiodev = PIN_MAP[outPin].gpio_device;
    uint8 pin = PIN_MAP[outPin].gpio_bit; // pin = 4 when outPin is 18

    // analogRead() preparation work
    const adc_dev *dev = PIN_MAP[analogPin].adc_device;
    adc_reg_map *regs = dev->regs;
    adc_set_reg_seqlen(dev, 1);
    regs->SQR3 = PIN_MAP[analogPin].adc_channel;
    
    start = millis();
    while (counter++ < limit)
    {
        /* digitalWrite(outPin, HIGH); */
        gpiodev->regs->BSRR = (1U << pin);
//      gpiodev->regs->BSRR = 0x10;

        /* val = analogRead(analogPin); */
	regs->CR2 |= ADC_CR2_SWSTART;
	while (!(regs->SR & ADC_SR_EOC))
            ;
	val = (uint16)(regs->DR & ADC_DR_DATA);

        /* digitalWrite(outPin, HIGH); */
        gpiodev->regs->BSRR = (1U << pin)<<16;
//      gpiodev->regs->BSRR = 0x100000;
    }
    stop = millis();

    COMM.println("Stop loops:");
    COMM.print("Elapsed Time: ");
    COMM.print(stop - start);
    COMM.print(" milliseconds (for ");
    COMM.print(limit);
    COMM.println(" analog reads)");
    COMM.println(" val = ");
    COMM.println(val);
    COMM.println(" PCLK2 = ");
    COMM.println(PCLK2);
    COMM.println(" pin = ");
    COMM.println(pin);
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

/*
Running result (simplified analogRead+digitalWrite()) is below:
Starting loops:
Stop loops:
Elapsed Time: 112 milliseconds (for 100000 analog reads)
 val =
2134
 PCLK2 =
72000000
 pin =
4

Starting loops:
Stop loops:
Elapsed Time: 111 milliseconds (for 100000 analog reads)
 val =
2039
 PCLK2 =
72000000
 pin =
4






If we precalculate the shift operation's result and use it directly, like below:
gpiodev->regs->BSRR = 0x10;
gpiodev->regs->BSRR = 0x100000;

then the running result is: (save quite a little time)
Starting loops:
Stop loops:
Elapsed Time: 110 milliseconds (for 100000 analog reads)
 val =
2117
 PCLK2 =
72000000
 pin =
4

Starting loops:
Stop loops:
Elapsed Time: 110 milliseconds (for 100000 analog reads)
 val =
2067
 PCLK2 =
72000000
 pin =
4







If we use digitalWrite() in the loop, then the running result is:
Starting loops:
Stop loops:
Elapsed Time: 244 milliseconds (for 100000 analog reads)
 val =
2020
 PCLK2 =
72000000
 pin =
4

Starting loops:
Stop loops:
Elapsed Time: 243 milliseconds (for 100000 analog reads)
 val =
2050
 PCLK2 =
72000000
 pin =
4








If we comment out the digitalWrite() code, like below:
gpiodev->regs->BSRR = (1U << pin);
gpiodev->regs->BSRR = (1U << pin)<<16;

then the running result is:
Starting loops:
Stop loops:
Elapsed Time: 99 milliseconds (for 100000 analog reads)
 val =
2109
 PCLK2 =
72000000
 pin =
4

Starting loops:
Stop loops:
Elapsed Time: 98 milliseconds (for 100000 analog reads)
 val =
2161
 PCLK2 =
72000000
 pin =
4

*/
