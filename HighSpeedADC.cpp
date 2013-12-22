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
    
    start = micros();
    while (counter++ < limit)
    {
//        digitalWrite(outPin, HIGH);
        gpiodev->regs->BSRR = (1U << pin);
//      gpiodev->regs->BSRR = 0x10;

//        val = analogRead(analogPin);
		regs->CR2 |= ADC_CR2_SWSTART;
		while (!(regs->SR & ADC_SR_EOC))
            ;
		val = (uint16)(regs->DR & ADC_DR_DATA);

//        digitalWrite(outPin, LOW);
        gpiodev->regs->BSRR = (1U << pin)<<16;
//      gpiodev->regs->BSRR = 0x100000;
    }
    stop = micros();
//    COMM.begin();
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
//    COMM.end();
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
Case 1: simplified analogRead() + simplified digitalWrite()
Elapsed Time: 107186 milliseconds (for 100000 analog reads)
Elapsed Time: 107185 milliseconds (for 100000 analog reads)


Case 2: simplified analogRead + without GPIO control
Elapsed Time: 96064 milliseconds (for 100000 analog reads)
Elapsed Time: 96065 milliseconds (for 100000 analog reads)


Case 3: original analogRead + without GPIO control,
Elapsed Time: 199089 milliseconds (for 100000 analog reads)
Elapsed Time: 199085 milliseconds (for 100000 analog reads)


Case 4: simplified analogRead() + simplified (and precalculated) digitalWrite()
Elapsed Time: 108584 milliseconds (for 100000 analog reads)
Elapsed Time: 108583 milliseconds (for 100000 analog reads)


Case 5: simplified analogRead() + original digitalWrite()
Elapsed Time: 243624 milliseconds (for 100000 analog reads)
Elapsed Time: 243621 milliseconds (for 100000 analog reads)


Case 6: original analogRead() + original digitalWrite()
Elapsed Time: 324387 milliseconds (for 100000 analog reads)
Elapsed Time: 324394 milliseconds (for 100000 analog reads)

*/
