#include "subsystems/imu.h"

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/nvic.h>

#include "mcu_periph/i2c.h"

void imu_aspirin_arch_int_enable(void) {

#ifdef ASPIRIN_USE_GYRO_INT
  nvic_set_priority(NVIC_EXTI15_10_IRQ, 0x0F);
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);
#endif

  nvic_set_priority(NVIC_EXTI2_IRQ, 0x0F);
  nvic_enable_irq(NVIC_EXTI2_IRQ);

  // should not be needed anymore, handled by the spi driver
#if 0
  /* Enable DMA1 channel4 IRQ Channel ( SPI RX) */
  nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
  nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
#endif
}

void imu_aspirin_arch_int_disable(void) {

#ifdef ASPIRIN_USE_GYRO_INT
  nvic_disable_irq(NVIC_EXTI15_10_IRQ);
#endif

  nvic_disable_irq(NVIC_EXTI2_IRQ);

  // should not be needed anymore, handled by the spi driver
#if 0
  /* Enable DMA1 channel4 IRQ Channel ( SPI RX) */
  nvic_disable_irq(NVIC_DMA1_CHANNEL4_IRQ);
#endif
}

void imu_aspirin_arch_init(void) {

  // This was needed for Lisa/L????
#if 0
  /* Set "mag ss" and "mag reset" as floating inputs ------------------------*/
  /* "mag ss"    (PC12) is shorted to I2C2 SDA       */
  /* "mag reset" (PC13) is shorted to I2C2 SCL       */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
  gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
	        GPIO_CNF_INPUT_FLOAT, GPIO12 | GPIO13);
#endif

  /* Gyro --------------------------------------------------------------------*/
  /* configure external interrupt exti15_10 on PC14( gyro int ) */
  /* 陀螺仪外部中断配置*/
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN |
			                    RCC_APB2ENR_AFIOEN);
  gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
	  GPIO_CNF_INPUT_FLOAT, GPIO14);

#ifdef ASPIRIN_USE_GYRO_INT
  exti_select_source(EXTI14, GPIOC);
  exti_set_trigger(EXTI14, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI14);
#endif

  /* configure external interrupt exti2 on PB2( accel int ) */
  /* 加速度计外部中断配置*/
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
	        GPIO_CNF_INPUT_FLOAT, GPIO2);
  exti_select_source(EXTI2, GPIOB);
  exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI2);

}


/****** the interrupts should be handled in the peripheral drivers *******/

/*
 * Gyro data ready 陀螺仪 数据准备好
 */
void exti15_10_isr(void) {

  /* clear EXTI 清中断 */ 
  exti_reset_request(EXTI14);

#ifdef ASPIRIN_USE_GYRO_INT
  imu_aspirin.gyro_eoc = TRUE;
  imu_aspirin.status = AspirinStatusReadingGyro;
#endif

}

/*
 * Accel data ready 加速度计 数据准备好
 */
void exti2_isr(void) {

  /* clear EXTI */
  exti_reset_request(EXTI2);

  //adxl345_start_reading_data();
}

