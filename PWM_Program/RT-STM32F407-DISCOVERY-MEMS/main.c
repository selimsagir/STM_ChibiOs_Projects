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

static void command_set_pwm(BaseSequentialStream *chp, int argc, char *argv[]){

  if (argc != 4)
    {
        chprintf(chp, "pwm_set needs 4 arguments\r\n");
        return;
    }
  
  for(int i=0 ; i<4 ; i++){
	
	     int pwm_value = strtol(argv[i],NULL,0); 	//strtol function is convert string to long integer)
		 
	  	 if (pwm_value < 0 || pwm_value > 100)
	       {
	        chprintf(chp, "pwm value is out of range\r\n");
	        return;
	       }
	       
	       pwmEnableChannel(&PWMD1, i, pwm_value);
  }



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
  100,                                      /* PWM period is 128 cycles.    */
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

///////////////******************************************///////////////////////////////////////////
  
   

   
  
///////////////******************************************///////////////////////////////////////////

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
  	pwmStart(&PWMD1, &pwmcfg);
  	pwmStart(&PWMD3, &pwmcfg);
  	pwmStart(&PWMD4, &pwmcfg);
  	pwmStart(&PWMD5, &pwmcfg);

  	palSetPadMode(GPIOE, GPIOE_PIN9,  PAL_MODE_ALTERNATE(1));      
  	palSetPadMode(GPIOE, GPIOE_PIN11, PAL_MODE_ALTERNATE(1));      
  	palSetPadMode(GPIOE, GPIOE_PIN13, PAL_MODE_ALTERNATE(1));     
  	palSetPadMode(GPIOE, GPIOE_PIN14, PAL_MODE_ALTERNATE(1));

  	palSetPadMode(GPIOC, GPIOC_PIN6,  PAL_MODE_ALTERNATE(2));      
  	palSetPadMode(GPIOC, GPIOC_MCLK,  PAL_MODE_ALTERNATE(2));      //pin7 name is gpioc_mclk in ChibiOs/os/hal/boards/ST_STM32F4_DISCOVERY/board.h 
  	palSetPadMode(GPIOC, GPIOC_PIN8,  PAL_MODE_ALTERNATE(2));     
  	palSetPadMode(GPIOC, GPIOC_PIN9,  PAL_MODE_ALTERNATE(2));

  	palSetPadMode(GPIOD, GPIOD_PIN9,  PAL_MODE_ALTERNATE(1));      
  	palSetPadMode(GPIOD, GPIOD_PIN11, PAL_MODE_ALTERNATE(1));      
  	palSetPadMode(GPIOD, GPIOD_PIN13, PAL_MODE_ALTERNATE(1));     
  	palSetPadMode(GPIOD, GPIOD_PIN14, PAL_MODE_ALTERNATE(1));

	pwmEnableChannel(&PWMD1, 0, 50);   			 //channels number should start 0
	pwmEnableChannel(&PWMD1, 1, 50);
	pwmEnableChannel(&PWMD1, 2, 50);
	pwmEnableChannel(&PWMD1, 3, 50);

	pwmEnableChannel(&PWMD3, 0, 50);   			 //channels number should start 0
	pwmEnableChannel(&PWMD3, 1, 50);
	pwmEnableChannel(&PWMD3, 2, 50);
	pwmEnableChannel(&PWMD3, 3, 50);



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
