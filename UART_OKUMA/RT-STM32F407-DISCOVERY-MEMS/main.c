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

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(256)
#define SBUS_WA_SIZE    THD_WORKING_AREA_SIZE(2048)   

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

int pwm_timchannel_mapping[12] = 
{
  0,  // 0
  1,  // 1
  2,  // 2
  3,  // 3
  0,  // 4
  2,  // 5
  3,  // 6
  1,  // 7
  2,  // 8
  3,  // 9
  0,  // 10
  1   // 11
};

PWMDriver* pwm_driver_mapping[12] =
{
  &PWMD1, // 0
  &PWMD1, // 1
  &PWMD1, // 2
  &PWMD1, // 3
  &PWMD3, // 4
  &PWMD3, // 5
  &PWMD3, // 6
  &PWMD5, // 7
  &PWMD5, // 8
  &PWMD5, // 9
  &PWMD9, // 10
  &PWMD9, // 11
};

static void command_set_pwm(BaseSequentialStream *chp, int argc, char* argv[]){

  if(argc != 2)
  {
    chprintf(chp, "set_pwm command needs 2 arguments; first one is channel number , second one is PWM value \r\n");
    return;
  }
  
  int channel_num = strtol(argv[0],NULL,0);

  if (channel_num < 0 || channel_num > 11  )   // argv[0] shows which is channel
  {
        chprintf(chp, "PWM channels should be between 0-11\r\n");
        return;
  }

  int pwm_value = strtol(argv[1],NULL,0);
  if (pwm_value < 0 || pwm_value > 100)
  {
  chprintf(chp, "PWM value is out of range\r\n");

  return;
  }
         
   pwmEnableChannel(pwm_driver_mapping[channel_num], pwm_timchannel_mapping[channel_num], pwm_value ) ;

 /* if (channel_num <= 3)    // argv[1]  shows pwm value 
  {  
         pwmEnableChannel(&PWMD1, channel_num, pwm_value);
  }
  else if (channel_num <= 6)    
  { 
    if (channel_num == 4 )
    {
      channel_num = 3 ; 
    }
         pwmEnableChannel(&PWMD3, channel_num - 3, pwm_value);   // 0  2  3 olmali
  }
  else if (channel_num <= 9)     
  {  
         pwmEnableChannel(&PWMD5, channel_num - 6, pwm_value);    // 1 2 3 olmali
  }
  else if (channel_num <= 11)    
  {  
         pwmEnableChannel(&PWMD9, channel_num - 10, pwm_value);     // 0 1 olmali
  }*/

}


static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"test", cmd_test},
  {"set_pwm", command_set_pwm},
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
  2048,                                      /* PWM period is 128 cycles.    */
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

    /* Waiting until the next 250 milliseconds time interval.*/
    chThdSleepUntil(time += MS2ST(100));
  }
}

/** @brief SBUS configuration.*/
static const SerialConfig SBUS_config =
{
  100000,
  USART_CR1_PCE,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

static THD_WORKING_AREA(waThreadSBUS, SBUS_WA_SIZE);
static THD_FUNCTION(ThreadSBUS, arg) {
  (void)arg;
  chThdSleepMilliseconds(2000);

  union
  {
  	struct 
  	{ 
  		unsigned channel0 : 11;
  		unsigned channel1 : 11;
  		unsigned channel2 : 11;
  		unsigned channel3 : 11;
  		unsigned channel4 : 11;
  		unsigned channel5 : 11;
  		unsigned channel6 : 11;
  		unsigned channel7 : 11;
  		unsigned channel8 : 11;
  		unsigned channel9 : 11;
  		unsigned channel10 : 11;
  		unsigned channel11 : 11;
  		unsigned channel12 : 11;
  		unsigned channel13 : 11;
  		unsigned channel14 : 11;
  		unsigned channel15 : 11;
  	} __attribute__((packed)) channels;

    uint8_t SBUSdata[24];
  }SBUSUnion;

  while(1)
  {	
   	int sync = chSequentialStreamGet(&SD2);  
   	if(sync == 0x0F){
	  int num = chSequentialStreamRead((BaseSequentialStream*) &SD2, SBUSUnion.SBUSdata, 24);

	  pwmEnableChannel(pwm_driver_mapping[0], pwm_timchannel_mapping[0], SBUSUnion.channels.channel0);
	  pwmEnableChannel(pwm_driver_mapping[1], pwm_timchannel_mapping[1], SBUSUnion.channels.channel1);
	  pwmEnableChannel(pwm_driver_mapping[2], pwm_timchannel_mapping[2], SBUSUnion.channels.channel2);
	  pwmEnableChannel(pwm_driver_mapping[3], pwm_timchannel_mapping[3], SBUSUnion.channels.channel3);
	  

	  chprintf((BaseSequentialStream*)&SDU1, "%d,%d,%d,%d\r\n",
	  	SBUSUnion.channels.channel0,
	  	SBUSUnion.channels.channel1,
	  	SBUSUnion.channels.channel2,
	  	SBUSUnion.channels.channel3);

   	} 

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
  sdStart(&SD2, &SBUS_config);
  palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));

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
  	pwmStart(&PWMD1, &pwmcfg);
  	pwmStart(&PWMD3, &pwmcfg);
  	pwmStart(&PWMD5, &pwmcfg);  
    pwmStart(&PWMD9, &pwmcfg);

  	palSetPadMode(GPIOE, GPIOE_PIN9,  PAL_MODE_ALTERNATE(1));      
  	palSetPadMode(GPIOE, GPIOE_PIN11, PAL_MODE_ALTERNATE(1));      
  	palSetPadMode(GPIOE, GPIOE_PIN13, PAL_MODE_ALTERNATE(1));     
  	palSetPadMode(GPIOE, GPIOE_PIN14, PAL_MODE_ALTERNATE(1));

    pwmEnableChannel(&PWMD1, 0, 50);         //channels number should start 0
    pwmEnableChannel(&PWMD1, 1, 50);
    pwmEnableChannel(&PWMD1, 2, 50);
    pwmEnableChannel(&PWMD1, 3, 50);

  	palSetPadMode(GPIOC, GPIOC_PIN6,  PAL_MODE_ALTERNATE(2));      
  	palSetPadMode(GPIOC, GPIOC_PIN8,  PAL_MODE_ALTERNATE(2));     
  	palSetPadMode(GPIOC, GPIOC_PIN9,  PAL_MODE_ALTERNATE(2));

    pwmEnableChannel(&PWMD3, 0, 50);         //channels number should start 0
    pwmEnableChannel(&PWMD3, 2, 50);
    pwmEnableChannel(&PWMD3, 3, 50);

    palSetPadMode(GPIOA, GPIOA_PIN1,  PAL_MODE_ALTERNATE(2));      
    palSetPadMode(GPIOA, GPIOA_PIN2,  PAL_MODE_ALTERNATE(2));     
    palSetPadMode(GPIOA, GPIOA_PIN3,  PAL_MODE_ALTERNATE(2));

    pwmEnableChannel(&PWMD5, 1, 50);         //channels number should start 0
    pwmEnableChannel(&PWMD5, 2, 50);
    pwmEnableChannel(&PWMD5, 3, 50);

    palSetPadMode(GPIOE, GPIOE_PIN5,  PAL_MODE_ALTERNATE(3));      
    palSetPadMode(GPIOE, GPIOE_PIN6,  PAL_MODE_ALTERNATE(3));     

    pwmEnableChannel(&PWMD9, 0, 50);         //channels number should start 0
    pwmEnableChannel(&PWMD9, 1, 50);



  chThdCreateStatic(waThreadSBUS, sizeof(waThreadSBUS),
                    NORMALPRIO + 10, ThreadSBUS, NULL);
  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1),
                    NORMALPRIO + 10, Thread1, NULL);

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
