/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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
#include "test.h"

#include "chprintf.h"
#include "shell.h"
#include "lis302dl.h"

#include "usbcfg.h"

#include <stdlib.h>
#include <stdbool.h>

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(256)


#define ADC_GRP1_NUM_CHANNELS   2

#define ADC_GRP2_NUM_CHANNELS   8
#define ADC_GRP2_BUF_DEPTH      16
//static adcsample_t samples2[ADC_GRP2_NUM_CHANNELS * ADC_GRP2_BUF_DEPTH];

BSEMAPHORE_DECL(adccomplete, TRUE); 
/*
 * ADC streaming callback.
 */
adcsample_t samples[8];


uint16_t count = 0;

static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void)buffer;
  (void)adcp;
  (void)n;

  //palTogglePad(GPIOB,5);

  chBSemSignalI(&adccomplete);

  count++;
  if (count > 4096) count = 0;

}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}


bool GetLine(BaseSequentialStream *chp, char *line , unsigned size) {
   char *p = line;

   while (true) {
     char c;
 
     if (chSequentialStreamRead(chp, (uint8_t *)&c, 1) == 0)
       return false;
     if (c == 4) {
       return false;
     }
     if ((c == 8) || (c == 127)) {
       if (p != line) {
         p--;
       }
       continue;
     }
     if (c == '\r') {
       *p = 0;
       return true;
     }
     if (c < 0x20)
       continue;
     if (p < line + size - 1) {
       *p++ = (char)c;
     }
   }
 }
 
char linebuffer[100];

static THD_WORKING_AREA(waGetlineTHread, 128);
static THD_FUNCTION(GetlineTHread, arg) {

  (void)arg;
   while(true){
	  
	  bool result = GetLine((BaseSequentialStream*)&SDU1 , linebuffer, sizeof(linebuffer));

	  if (result == true)
	  {

	   unsigned bufferdata = strtol(linebuffer , NULL , 0);
	   bufferdata = (1000000)/bufferdata;
	   gptChangeInterval(&GPTD3 , bufferdata);
	 
	  }
  }
};
								
static void gpt3cb(GPTDriver *gptp) {

  (void)gptp;
  palTogglePad(GPIOB,5);
}

static const GPTConfig gpt1cfg = {
  1000000,    /* 1MHz timer clock.*/
  gpt3cb,   /* Timer callback.*/
  TIM_CR2_MMS_1  , // select to TRGO = update event , BITS = 010
  0
};


static const ADCConversionGroup adcgrpcfg2 = {
  TRUE,
  ADC_GRP2_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0,                        /* CR1 */
  ADC_CR2_EXTSEL_3  | ADC_CR2_EXTEN_0,        /* CR2 */    /* BITS  = 1000 , Timer 3 TRGO event */  /*Trigger detection on the rising edge*/
  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN12(ADC_SAMPLE_3) |
  ADC_SMPR1_SMP_AN14(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN15(ADC_SAMPLE_3),

  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3) |
  ADC_SMPR2_SMP_AN3(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN8(ADC_SAMPLE_3),

  ADC_SQR1_NUM_CH(ADC_GRP2_NUM_CHANNELS),
  ADC_SQR2_SQ8_N(ADC_CHANNEL_IN8) | ADC_SQR2_SQ7_N(ADC_CHANNEL_IN15),
  ADC_SQR3_SQ6_N(ADC_CHANNEL_IN14) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN2) | 
  ADC_SQR3_SQ3_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN12) | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN11)
};

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    IN11.
 */

union tempframe{
    struct
    {
        char framestart [2];
        char size;
        uint16_t adcdata0;
        uint16_t adcdata1;
        uint16_t adcdata2;
        uint16_t adcdata3;
        uint16_t adcdata4;
        uint16_t adcdata5;
        uint16_t adcdata6;
        uint16_t adcdata7; 
    } __attribute__((packed)) frame;
    unsigned char data[19];
} dat;

static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;

  dat.frame.framestart[0] = 0xAA;
  dat.frame.framestart[1] = 0xBB;
  dat.frame.size=16-2*6;

  while (true)
  {

    chBSemWait(&adccomplete); 
   // palSetPad(GPIOD, 7);

    dat.frame.adcdata0=samples[0];
    dat.frame.adcdata1=samples[1];
    dat.frame.adcdata2=samples[2];
    dat.frame.adcdata3=samples[3];
    dat.frame.adcdata4=samples[4];
    dat.frame.adcdata5=samples[5];
    dat.frame.adcdata6=samples[6];
    dat.frame.adcdata7=samples[7];

    dat.frame.adcdata1 = count;


    // chprintf((BaseSequentialStream*) &SDU1, "1");

    /*chprintf((BaseSequentialStream*) &SDU1,"%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                                                     samples[0],count,       
                                                     samples[2],samples[3],
                                                     samples[4],samples[5],
                                                     samples[6],samples[7]
                                                    );*/
    // don't send last 4 channels                                                 
    streamWrite((BaseSequentialStream*) &SDU1, dat.data, sizeof(dat.data)-2*6);

    // palClearPad(GPIOD, 7);

  }
};



void startADC(void){
  unsigned bufferdata = 1000;
  adcStart(&ADCD1 , NULL);   // Config file is emty so using  null
  adcSTM32EnableTSVREFE();
  adcStartConversion(&ADCD1 , &adcgrpcfg2 , samples, 1);
  gptStart(&GPTD3 , &gpt1cfg);
  gptStartContinuous(&GPTD3 , bufferdata);  // ADC freq 1 khz


};

///******************************************************************************///
static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;
 (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%08lx %08lx %4lu %4lu %9s\r\n",
             (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
             (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
             states[tp->p_state]);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
  thread_t *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: test\r\n");
    return;
  }
  tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriorityX(),
                           TestThread, chp);
  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }
  chThdWait(tp);
}

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"test", cmd_test},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};

/*===========================================================================*/
/* Accelerometer related.                                                    */
/*===========================================================================*/

/*
 * PWM configuration structure.
 * Cyclic callback enabled, channels 1 and 4 enabled without callbacks,
 * the active state is a logic one.
 */
static const PWMConfig pwmcfg = {
  100000,                                   /* 100kHz PWM clock frequency.  */
  128,                                      /* PWM period is 128 cycles.    */
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  /* HW dependent part.*/
  0,
  0
};

/*
 * SPI1 configuration structure.
 * Speed 5.25MHz, CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
 * The slave select line is the pin GPIOE_CS_SPI on the port GPIOE.
 */
static const SPIConfig spi1cfg = {
  NULL,
  /* HW dependent part.*/
  GPIOE,
  GPIOE_CS_SPI,
  SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

/*
 * SPI2 configuration structure.
 * Speed 21MHz, CPHA=0, CPOL=0, 8bits frames, MSb transmitted first.
 * The slave select line is the pin 12 on the port GPIOA.
 */
static const SPIConfig spi2cfg = {
  NULL,
  /* HW dependent part.*/
  GPIOB,
  12,
  0
};

/*
 * This is a periodic thread that reads accelerometer and outputs
 * result to SPI2 and PWM.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  static int8_t xbuf[4], ybuf[4];   /* Last accelerometer data.*/
  systime_t time;                   /* Next deadline.*/

  (void)arg;
  chRegSetThreadName("reader");

  /* LIS302DL initialization.*/
  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG1, 0x43);
  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG2, 0x00);
  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG3, 0x00);

  /* Reader thread loop.*/
  time = chVTGetSystemTime();
  while (true) {
    int32_t x, y;
    unsigned i;

    /* Keeping an history of the latest four accelerometer readings.*/
    for (i = 3; i > 0; i--) {
      xbuf[i] = xbuf[i - 1];
      ybuf[i] = ybuf[i - 1];
    }

    /* Reading MEMS accelerometer X and Y registers.*/
    xbuf[0] = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTX);
    ybuf[0] = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTY);

    /* Transmitting accelerometer the data over SPI2.*/
    spiSelect(&SPID2);
    spiSend(&SPID2, 4, xbuf);
    spiSend(&SPID2, 4, ybuf);
    spiUnselect(&SPID2);

    /* Calculating average of the latest four accelerometer readings.*/
    x = ((int32_t)xbuf[0] + (int32_t)xbuf[1] +
         (int32_t)xbuf[2] + (int32_t)xbuf[3]) / 4;
    y = ((int32_t)ybuf[0] + (int32_t)ybuf[1] +
         (int32_t)ybuf[2] + (int32_t)ybuf[3]) / 4;

    /* Reprogramming the four PWM channels using the accelerometer data.*/
    if (y < 0) {
      pwmEnableChannel(&PWMD4, 0, (pwmcnt_t)-y);
      pwmEnableChannel(&PWMD4, 2, (pwmcnt_t)0);
    }
    else {
      pwmEnableChannel(&PWMD4, 2, (pwmcnt_t)y);
      pwmEnableChannel(&PWMD4, 0, (pwmcnt_t)0);
    }
    if (x < 0) {
      pwmEnableChannel(&PWMD4, 1, (pwmcnt_t)-x);
      pwmEnableChannel(&PWMD4, 3, (pwmcnt_t)0);
    }
    else {
      pwmEnableChannel(&PWMD4, 3, (pwmcnt_t)x);
      pwmEnableChannel(&PWMD4, 1, (pwmcnt_t)0);
    }

    /* Waiting until the next 250 milliseconds time interval.*/
    chThdSleepUntil(time += MS2ST(100));
  }
}

/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/

/*
 * Application entry point.
 */
int main(void) {
  thread_t *shelltp = NULL;
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  palSetPadMode(GPIOD,7,PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB,5,PAL_MODE_OUTPUT_PUSHPULL);

  palSetPad(GPIOC, 8);


  /*
   * Setting up analog inputs used by the demo.
   */
  palSetGroupMode(GPIOC, PAL_PORT_BIT(1) | PAL_PORT_BIT(2) | PAL_PORT_BIT(4) | PAL_PORT_BIT(5),
                  0, PAL_MODE_INPUT_ANALOG);
  palSetGroupMode(GPIOA, PAL_PORT_BIT(1) | PAL_PORT_BIT(2) | PAL_PORT_BIT(3),
                  0, PAL_MODE_INPUT_ANALOG);
  palSetGroupMode(GPIOB, PAL_PORT_BIT(0) , 
                  0, PAL_MODE_INPUT_ANALOG);

  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Initializes a serial-over-USB CDC driver.
   */

  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1000);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /*
   * Initializes the SPI driver 1 in order to access the MEMS. The signals
   * are already initialized in the board file.
   */
  spiStart(&SPID1, &spi1cfg);

  /*
   * Initializes the SPI driver 2. The SPI2 signals are routed as follow:
   * PB12 - NSS.
   * PB13 - SCK.
   * PB14 - MISO.
   * PB15 - MOSI.
   */
  spiStart(&SPID2, &spi2cfg);
  palSetPad(GPIOB, 12);
  palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);           /* NSS.     */
  palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(5) |
                           PAL_STM32_OSPEED_HIGHEST);           /* SCK.     */
  palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(5));              /* MISO.    */
  palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(5) |
                           PAL_STM32_OSPEED_HIGHEST);           /* MOSI.    */

  /*
   * Initializes the PWM driver 4, routes the TIM4 outputs to the board LEDs.
   */
  pwmStart(&PWMD4, &pwmcfg);
  palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_ALTERNATE(2));      /* Green.   */
  palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_ALTERNATE(2));      /* Orange.  */
  palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_ALTERNATE(2));      /* Red.     */
  palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_ALTERNATE(2));      /* Blue.    */


/*
  *  Threads 
   */
  

  chThdCreateStatic(waThread1, sizeof(waThread1),
                    NORMALPRIO + 10, Thread1, NULL);

  startADC();
  
  chThdCreateStatic(waThread2, sizeof(waThread2),
                      NORMALPRIO + 10, Thread2, NULL);

  chThdCreateStatic(waGetlineTHread, sizeof(waGetlineTHread),
                    NORMALPRIO + 10, GetlineTHread, NULL);
  


/*													
  *
   */

  /*
   * Normal main() thread activity, in this demo it just performs
   * a shell respawn upon its termination.
   */
  while (true) {
    if (!shelltp) {
      if (SDU1.config->usbp->state == USB_ACTIVE) {
        /* Spawns a new shell.*/
        shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
      }
    }
    else {
      /* If the previous shell exited.*/
      if (chThdTerminatedX(shelltp)) {
        /* Recovers memory of the previous shell.*/
        chThdRelease(shelltp);
        shelltp = NULL;
      }
    }
    chThdSleepMilliseconds(500);
  }
}
