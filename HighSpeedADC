#include <wirish/wirish.h>
#include <libmaple/adc.h>

#define COMM SerialUSB // use this for Maple
// #define COMM Serial // use this for Arduino
unsigned long start = 0;
unsigned long stop = 0;
unsigned long counter = 0;
unsigned long limit = 100000;
int outPin = 18;
int val = 0;

void setup()
{
    // the following line is needed for Maple
    pinMode(3, INPUT_ANALOG);
    pinMode(outPin, OUTPUT);
    // 下面的代码可能不全，会导致串口无输出
    //  adc_init(ADC1);    //rcc_clk_enable(ADC1->clk_id), Must be the first adc command!
    //  adc_init(ADC2);
    // 
    //  adc_enable(ADC1);  //ADC_CR2_ADON_BIT = 1
    //  adc_enable(ADC2);
    // 
    //  adc_calibrate(ADC1);  //Optional
    //  adc_calibrate(ADC2);
    //  采样率优化的关键寄存器参数
    adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
    adc_set_sample_rate(ADC1, ADC_SMPR_1_5);
    // the following line is needed for Arduino
    // COMM.begin(57600);
}

void loop()
{
    COMM.println("\nStarting loops:");
    start = millis();
    counter = 0;
    while (counter++ < limit)
    {
        //digitalWrite(outPin, HIGH);
        val = analogRead(3);
        //digitalWrite(outPin, LOW);
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

//代码运行结果：
//Starting loops :
//Stop loops :
//Elapsed Time : 189 milliseconds(for 100000 analog reads)
//val =
//2062
//PCLK2 =
//72000000
//
//Starting loops :
//Stop loops :
//Elapsed Time : 190 milliseconds(for 100000 analog reads)
//val =
//2060
//PCLK2 =
//72000000
//
//
//如果将GPIO管脚输出打开（消除注释），如下：
//while (counter++ < limit)
//{
//	//digitalWrite(outPin, HIGH);
//	val = analogRead(3);
//	//digitalWrite(outPin, LOW);
//}
//
//则运行结果如下：
//Starting loops :
//Stop loops :
//Elapsed Time : 309 milliseconds(for 100000 analog reads)
//val =
//2169
//PCLK2 =
//72000000
//
//Starting loops :
//Stop loops :
//Elapsed Time : 309 milliseconds(for 100000 analog reads)
//val =
//2133
//PCLK2 =
//72000000
