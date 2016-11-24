/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "memstreams.h"

#define ADC_GRP1_NUM_CHANNELS   2
// #define ADC_GRP1_BUF_DEPTH      8400
#define ADC_GRP1_BUF_DEPTH      1050

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

static virtual_timer_t vt3, vt4, vt5;

static const uint8_t message[] = "0123456789ABCDEF";
//static uint8_t buffer[16];

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    IN12 --> PC.2 --> ADC123.
 *              IN13 --> PC.3 --> ADC123.
 */

static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  0,                        /* CR1 */  /*bits25:24--> 00: 12bit, 01: 10bit, 10: 8bit, 11: 6bit*/ /*ADC_CR1_RES_1, ADC_CR1_RES_0, ADC_CR1_RES*/
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN12(ADC_SAMPLE_28) | ADC_SMPR1_SMP_AN13(ADC_SAMPLE_28) ,  /* SMPR1 */
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,                        /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN12) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN13)
};

static void led3off(void *p) {

  (void)p;
  palClearPad(GPIOD, GPIOD_LED3);
}

static void led4off(void *p) {

  (void)p;
  palClearPad(GPIOD, GPIOD_LED4);
}

static void led5off(void *p) {

  (void)p;
  palClearPad(GPIOD, GPIOD_LED5);
}

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {

  (void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {

  (void)uartp;
  palSetPad(GPIOD, GPIOD_LED5);
  chSysLockFromISR();
  chVTResetI(&vt5);
  chVTSetI(&vt5, MS2ST(200), led5off, NULL);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {

  (void)uartp;
  (void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {

  (void)uartp;
  (void)c;
  /* Flashing the LED each time a character is received.*/
  palSetPad(GPIOD, GPIOD_LED4);
  chSysLockFromISR();
  chVTResetI(&vt4);
  chVTSetI(&vt4, MS2ST(200), led4off, NULL);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {

  (void)uartp;

  /* Flashing the LED each time a character is received.*/
  palSetPad(GPIOD, GPIOD_LED3);
  chSysLockFromISR();
  chVTResetI(&vt3);
  chVTSetI(&vt3, MS2ST(200), led3off, NULL);
  chSysUnlockFromISR();
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_1 = {
  txend1,
  txend2,
  rxend,
  rxchar,
  rxerr,
  38400,
  0,
  USART_CR2_LINEN,
  0
};

/*
 * Orange LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE)
  {
    palClearPad(GPIOD, GPIOD_LED6);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOD, GPIOD_LED6);
    chThdSleepMilliseconds(500);
  }
}


/********************************************Thread2 implementation*******************************/

 thread_t *thread2_p = NULL;     //pointer thread2 az noe "thread_t"(ghabele tavajoh ke in type_def dar chibi3 hast)


 static THD_WORKING_AREA(waThread2, 128);//ekhtesase fazaye poshte hafeze be thread2 be andaze 128byte
 static THD_FUNCTION(Thread2, arg) {

   (void)arg;

 //  chRegSetThreadName("touchpad");    // baraye nam gozariye thread ha mishe az in estefade kard

   while(TRUE) {
     chEvtWaitAny((eventmask_t)1);  //montazer mimune ta yek event ya hamun msg_t az intrupt berese
     //palSetPad(GPIOE,GPIOD_LED6);
     //chThdSleepMilliseconds(500);
     //palClearPad(GPIOE,GPIOD_LED6);
    //  chThdSleepMilliseconds(500);
     extChannelDisable(&EXTD1, 0); //disable
     adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);  //ba labe bala ravande yek bar ADC anjam mishavad

     uartStartSend(&UARTD2, 10, "ADCSTART\r\n");
     chThdSleepMilliseconds(30);

     uartStopSend(&UARTD2);

     int i, x=0;
     char smpl[7];

     for (i=0 ; i <= (ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH)-1; i=i+1){
       x=chsnprintf(smpl, sizeof(smpl), "%u\r\n", samples1[i]);

       uartStartSend(&UARTD2, x, smpl);
       chThdSleepMilliseconds(3);
     }

     uartStopSend(&UARTD2);

     uartStartSend(&UARTD2,9, "ADCSTOP\r\n");
     chThdSleepMilliseconds(30);

     uartStopSend(&UARTD2);

 /*    if (palReadPad(GPIOA, GPIOA_BUTTON)) {
       gptStopTimer(&GPTD6);
       dacStopConversion(&DACD1);
     }
 */
     chThdSleepMilliseconds(3000);





    //  chThdSleepMilliseconds(500);
    extChannelEnable(&EXTD1, 0);  //enable ext interrupt to catch again

   }
 }

// /* Triggered when the button is pressed or released. The LED5 is set to ON.*/
static void extcb1(EXTDriver *extp, expchannel_t channel) {
/*  static virtual_timer_t vt4;

  (void)extp;
  (void)channel;

  palSetPad(GPIOD, GPIOD_LED5);
  chSysLockFromISR();
  chVTResetI(&vt4);*/

  /* LED4 set to OFF after 200mS.*/
/*  chVTSetI(&vt4, MS2ST(200), led5off, NULL);
  chSysUnlockFromISR();*/

  (void)extp;
  (void)channel;

  chSysLockFromISR();
  chEvtSignalI(thread2_p, (eventmask_t)1);  //faal shodan flag baraye thread2
  chSysUnlockFromISR();
}

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};


/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();


  /*
   * Setting up analog inputs used by the demo.
   */
  palSetGroupMode(GPIOC, PAL_PORT_BIT(2) | PAL_PORT_BIT(3),
                  0, PAL_MODE_INPUT_ANALOG);
 /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

/*
   * our own thread to be called after EXT
   */
   thread2_p = chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL); //thread baraye interrupt ba arzeshe normal

  /*
   * Activates the UART driver 2, PA2(TX) and PA3(RX) are routed to USART2.
   */
  uartStart(&UARTD2, &uart_cfg_1);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /*
   * Activates the ADC1 driver and the temperature sensor.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTSVREFE();

/*
   * Linear conversion.
   */
  // adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
  // chThdSleepMilliseconds(1000);
  chThdSleepMilliseconds(1);

  /*
   * Activates the EXT driver 1.
   */
  extStart(&EXTD1, &extcfg);
  extChannelEnable(&EXTD1, 0);

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {


    chThdSleepMilliseconds(500);
  }
}
