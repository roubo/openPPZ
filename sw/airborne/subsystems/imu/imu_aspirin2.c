/*
 * Copyright (C) 2012 Christophe DeWagter
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
#include "subsystems/imu.h"

#include "led.h"
#include "mcu_periph/spi.h"

// Peripherials
#include "peripherals/mpu60X0.h"
#include "peripherals/hmc58xx_regs.h"
#include "peripherals/ms5611.h"

#ifndef MPU6000_SLAVE_IDX
#define MPU6000_SLAVE_IDX SPI_SLAVE2
#endif

#ifndef MPU6000_SPI_DEV
#define MPU6000_SPI_DEV spi2
#endif

/* HMC58XX default conf */
#ifndef HMC58XX_DO
#define HMC58XX_DO 0x6 // Data Output Rate (6 -> 50Hz with HMC5843, 75Hz with HMC5883)
                       // 数据输出速度（6对应为hmc5843的50hz，hmc5883的75hz）
#endif
#ifndef HMC58XX_MS
#define HMC58XX_MS 0x0 // Measurement configuration测量配置
#endif
#ifndef HMC58XX_GN
#define HMC58XX_GN 0x1 // Gain configuration (1 -> +- 1 Gauss)增益配置
#endif
#ifndef HMC58XX_MD
#define HMC58XX_MD 0x0 // Continious measurement mode连续测量模式
#endif

#define HMC58XX_CRA ((HMC58XX_DO<<2)|(HMC58XX_MS))
#define HMC58XX_CRB (HMC58XX_GN<<5)

struct ImuAspirin2 imu_aspirin2;

struct spi_transaction aspirin2_mpu60x0;

// initialize peripherals
static void mpu_configure(void);
static void trans_cb( struct spi_transaction *trans );

//aspirin_mpu60x0的初始化，imu_aspirin的状态设置
void imu_impl_init(void) {
  aspirin2_mpu60x0.select = SPISelectUnselect;//spi的通讯状态
  aspirin2_mpu60x0.cpol = SPICpolIdleHigh;//当spi时钟空闲时拉低
  aspirin2_mpu60x0.cpha = SPICphaEdge2;
  aspirin2_mpu60x0.dss = SPIDss8bit;
  aspirin2_mpu60x0.bitorder = SPIMSBFirst;
  aspirin2_mpu60x0.cdiv = SPIDiv64;
  aspirin2_mpu60x0.slave_idx = MPU6000_SLAVE_IDX;//从机ID号
  aspirin2_mpu60x0.output_length = IMU_ASPIRIN_BUFFER_LEN;//32的
  aspirin2_mpu60x0.input_length = IMU_ASPIRIN_BUFFER_LEN;
  aspirin2_mpu60x0.after_cb = trans_cb;

  imu_aspirin2.status = Aspirin2StatusUninit;
  imu_aspirin2.imu_available = FALSE;
  aspirin2_mpu60x0.input_buf = &imu_aspirin2.input_buf_p[0];
  aspirin2_mpu60x0.output_buf = &imu_aspirin2.output_buf_p[0];
}

//在main.c中被main_periodic（）调用
void imu_periodic(void)
{
  //imu_init()中imu_aspirin2的状态已设为Aspirin2StatusUninit
  if (imu_aspirin2.status == Aspirin2StatusUninit) {
    mpu_configure();
    imu_aspirin2.status = Aspirin2StatusIdle;//imu空闲

    aspirin2_mpu60x0.output_length = 22;//读和写的字数
    aspirin2_mpu60x0.input_length = 22;
    //mpu的读缓冲区的buf[0]为中断寄存器状态和spi读
    aspirin2_mpu60x0.output_buf[0] = MPU60X0_REG_INT_STATUS + MPU60X0_SPI_READ;
    for (int i=1; i<aspirin2_mpu60x0.output_length; i++) {
        aspirin2_mpu60x0.output_buf[i] = 0;//剩余的21个缓冲区存放为0
    }
  }
  else {
    spi_submit(&(MPU6000_SPI_DEV), &aspirin2_mpu60x0);
  }
}

static void trans_cb(struct spi_transaction *trans __attribute__ ((unused))) {
  if ( imu_aspirin2.status != Aspirin2StatusUninit ) {
    imu_aspirin2.imu_available = TRUE;
  }
}

static inline void mpu_set(uint8_t _reg, uint8_t _val)
{
  aspirin2_mpu60x0.output_buf[0] = _reg;//buf[0]存放寄存器地址
  aspirin2_mpu60x0.output_buf[1] = _val;//buf[1]存放值
  spi_submit(&(MPU6000_SPI_DEV), &aspirin2_mpu60x0);

  // FIXME: no busy waiting! if really needed add a timeout!!!!
    while(aspirin2_mpu60x0.status != SPITransSuccess);//直到aspirin2交换成功后才跳出循环:
}

static inline void mpu_wait_slave4_ready(void)
{
  uint8_t ret = 0x80;
  while (ret & 0x80)
  {
    aspirin2_mpu60x0.output_buf[0] = MPU60X0_REG_I2C_SLV4_CTRL | MPU60X0_SPI_READ ;
    aspirin2_mpu60x0.output_buf[1] = 0;
    spi_submit(&(MPU6000_SPI_DEV), &aspirin2_mpu60x0);

    // FIXME: no busy waiting! if really needed add a timeout!!!!
      while(aspirin2_mpu60x0.status != SPITransSuccess);

    ret = aspirin2_mpu60x0.input_buf[1];
  }
}

static void mpu_configure(void)
{
  aspirin2_mpu60x0.output_length = 2;//mpu的读和写的字数
  aspirin2_mpu60x0.input_length = 2;

  ///////////////////
  // Reset the MPU复位MPU
  mpu_set( MPU60X0_REG_PWR_MGMT_1,
           0x01 << 7);		// -device reset pwr所有的内部寄存器都复位到默认值
  mpu_set( MPU60X0_REG_USER_CTRL,
	   (1 << 2) |           // Trigger a FIFO_RESET   FIFObuffer复位
	   (1 << 1) |           // Trigger a I2C_MST_RESET I2C 主机复位
	   (1 << 0) );          // Trigger a SIG_COND_RESET 所有传感器的信号线复位

  ///////////////////
  // Configure power:配置电源
   
  // MPU60X0_REG_PWR_MGMT_1
  mpu_set( MPU60X0_REG_PWR_MGMT_1,
           0x01);		// -switch to gyroX clock
                                // x轴陀螺仪的PLL参考

  // Wait for the new clock to stabilize.
  // FIXME: This must not be a delay!
  // It should be done using the MPU-6000 interrupt!
  //等待新的时钟稳定下来，必须要有该延时通过mpu-6000的中断完成
  {for (int i = 0; i < 1000000; i++) { asm("nop"); }}

  // MPU60X0_REG_PWR_MGMT_2: Nothing should be in standby: default OK
  // -No standby and no wake timer

  /////////////////////////
  // Measurement Settings 测量设置
#if PERIODIC_FREQUENCY == 60//此处根据main.c中的设置
// Accelerometer: Bandwidth 44Hz, Delay 4.9ms
// Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1Khz
// 加速度计的带宽44hz，延时4.9ms
// 陀螺仪的带宽42hz,延时4.9ms
#  define MPU_DIG_FILTER 3
// -100Hz output = 1kHz / (9 + 1)
#  define MPU_SMPLRT_DIV 9
#else
#  if PERIODIC_FREQUENCY == 120
//   Accelerometer: Bandwidth 44Hz, Delay 4.9ms
//   Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1Khz
#    define MPU_DIG_FILTER 3
//   -100Hz output = 1kHz / (9 + 1)
#    define MPU_SMPLRT_DIV 9
#  else
#    if PERIODIC_FREQUENCY == 512
//     Accelerometer: Bandwidth 260Hz, Delay 0ms
//     Gyroscope: Bandwidth 256Hz, Delay 0.89ms sampling 8Khz
#      define MPU_DIG_FILTER 0
//     -500Hz output = 1kHz / (1 + 1)
#      define MPU_SMPLRT_DIV 1
#    else
#    error PERIODIC_FREQUENCY should be either 60Hz, 120Hz or 512Hz. Otherwise manually fix the sensor rates
#    endif
#  endif
#endif
  aspirin2_mpu60x0.output_buf[1] = (2 << 3) | (MPU_DIG_FILTER << 0);
  spi_submit(&(MPU6000_SPI_DEV), &aspirin2_mpu60x0);
  mpu_set( MPU60X0_REG_CONFIG,
           (2 << 3) | 			// Fsync / ext sync on gyro X (bit 3->6)
                                        //外部信号连接到FSYNC，同步采集gyrox的信号
           (MPU_DIG_FILTER << 0) );	// Low-Pass Filter低通滤波

  // MPU60X0_REG_SMPLRT_DIV
  mpu_set( MPU60X0_REG_SMPLRT_DIV, MPU_SMPLRT_DIV);//采样率为1khz/(9+1）=100hz

  // MPU60X0_REG_GYRO_CONFIG
  mpu_set( MPU60X0_REG_GYRO_CONFIG,
           (3 << 3) );			// -2000deg/sec全量程范围角速度2000度/s

  // MPU60X0_REG_ACCEL_CONFIG
  mpu_set( MPU60X0_REG_ACCEL_CONFIG,
           (0 << 0) |			// No HPFL
           (3 << 3) );			// Full Scale = 16g全量程为16g

#ifndef MPU6000_NO_SLAVES
PRINT_CONFIG_MSG("Reading MPU slaves")

  /////////////////////////////////////
  // SPI Slave Configuration Section
  //SPI的从机配置
  // Power the Aux I2C Circuit:使用辅助的I2C通讯协议
  // MPU60X0_REG_AUX_VDDIO = 0 (good on startup):  (0 << 7);	// MPU6000: 0=Vdd. MPU6050 : 0=VLogic 1=Vdd
  
  // MPU60X0_REG_USER_CTRL:
  mpu_set( MPU60X0_REG_USER_CTRL,
           (1 << 5) |		// I2C_MST_EN: Enable Aux I2C Master Mode使能I2C的主机模式
           (1 << 4) |		// I2C_IF_DIS: Disable I2C on primary interface
                                //失能I2C，使能SPI
           (0 << 1) );		// Trigger a I2C_MST_RESET

  // Enable the aux i2c使能辅助的i2c作为ms5611的通讯协议
  // i2c主机时钟为256khz,主机从一个从机读到另一个从机读之间需要restart
  mpu_set( MPU60X0_REG_I2C_MST_CTRL,
           (0 << 7) | 		// no multimaster
           (0 << 6) |		// do not delay IRQ waiting for all external slaves
           (0 << 5) | 		// no slave 3 FIFO
           (0 << 4) | 		// restart or stop/start from one slave to another: read -> write is always stop/start
           (8 << 0) );		// 0=348kHz 8=256kHz, 9=500kHz

  mpu_set( MPU60X0_REG_I2C_MST_DELAY,
           (0 << 2) |		// No Delay Slave 2
           (1 << 3) );		// Delay Slave 3

#if defined IMU_ASPIRIN_VERSION_2_1 && USE_IMU_ASPIRIN2_BARO_SLAVE

  // MS5611 Send Reset气压计在aspirin2.1中使用的I2C通讯
  mpu_set( MPU60X0_REG_I2C_SLV4_ADDR, (MS5611_ADDR0));//写，地址为0x77
  mpu_set( MPU60X0_REG_I2C_SLV4_DO,  MS5611_REG_RESET);//向从机4(ms5611）写入0x1e
  mpu_set( MPU60X0_REG_I2C_SLV4_CTRL,
           (1 << 7) |		// Slave 4 enable使能从机4
           (0 << 6) |		// Byte Swap
           (1 << 5) |		// Reg_Dis: do not write the register, just the data
                                //不是写寄存器地址，只是写或读数据
           (0 << 0) );		// Full Speed全速采样

  mpu_wait_slave4_ready();

  // Wait at least 2.8ms

#endif // read MS5611 as MPU slave

  // HMC5883 Magnetometer Configuration磁力计配置
  // 磁力计三通道的输出速率为75hz,测量模式为：正常测量配置，采样平均数：1,
  //       三通道的增益为1090(默认）
  //       连续测量模式
  mpu_set( MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set( MPU60X0_REG_I2C_SLV4_REG,  HMC58XX_REG_CFGA); //从机4的内部配置寄存器A：0x00，该寄存器用于数据交换的开始
  mpu_set( MPU60X0_REG_I2C_SLV4_DO,  HMC58XX_CRA);//写数据0x06<<2到从机4寄存器A
  mpu_set( MPU60X0_REG_I2C_SLV4_CTRL,
           (1 << 7) |		// Slave 4 enable
           (0 << 6) |		// Byte Swap
           (0 << 0) );		// Full Speed

  mpu_wait_slave4_ready();

  mpu_set( MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set( MPU60X0_REG_I2C_SLV4_REG,  HMC58XX_REG_CFGB);//从机内部配置寄存器B：0x01
  mpu_set( MPU60X0_REG_I2C_SLV4_DO,  HMC58XX_CRB);//写数据0x01<<5到从机4配置寄存器B
  mpu_set( MPU60X0_REG_I2C_SLV4_CTRL,
           (1 << 7) |		// Slave 4 enable
           (0 << 6) |		// Byte Swap
           (0 << 0) );		// Full Speed

  mpu_wait_slave4_ready();

  mpu_set( MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set( MPU60X0_REG_I2C_SLV4_REG,  HMC58XX_REG_MODE);
  mpu_set( MPU60X0_REG_I2C_SLV4_DO,  HMC58XX_MD);//写数据0x00到从机4
  mpu_set( MPU60X0_REG_I2C_SLV4_CTRL,
           (1 << 7) |		// Slave 4 enable
           (0 << 6) |		// Byte Swap
           (3 << 0) );		// From now on a delayed rate of 1/4 is defined...

  // HMC5883 Reading:
  // a) write hmc-register to HMC
  // b) read 6 bytes from HMC
 // 读HMC5883的x轴数据输出寄存器A（高位）的值
  mpu_set( MPU60X0_REG_I2C_SLV0_ADDR, (HMC58XX_ADDR >> 1) | MPU60X0_SPI_READ);
  mpu_set( MPU60X0_REG_I2C_SLV0_REG,  HMC58XX_REG_DATXM);//从机0的内部寄存器0x03
  // Put the enable command as last.
  mpu_set( MPU60X0_REG_I2C_SLV0_CTRL,
           (1 << 7) |		// Slave 0 enable从机0
           (0 << 6) |		// Byte Swap
           (6 << 0) );		// Read 6 bytes

	// Slave 0 Control:

#if defined IMU_ASPIRIN_VERSION_2_1 && USE_IMU_ASPIRIN2_BARO_SLAVE
PRINT_CONFIG_MSG("Reading the MS5611 as MPU slave")
/*


  // Read MS5611 Calibration
  mpu_set( MPU60X0_REG_I2C_SLV1_ADDR, (MS5611_ADDR0) | MPU60X0_SPI_READ);
  mpu_set( MPU60X0_REG_I2C_SLV1_REG,  MS5611_REG_ADCREAD);
  // Put the enable command as last.
  mpu_set( MPU60X0_REG_I2C_SLV1_CTRL,
           (1 << 7) |		// Slave 1 enable
           (0 << 6) |		// Byte Swap
           (3 << 0) );		// Read 6 bytes

*/
  //MS5611的读
  // Full Rate Request For Pressure向气压计的“最大速度”的请求
  mpu_set( MPU60X0_REG_I2C_SLV2_ADDR, (MS5611_ADDR0));
  mpu_set( MPU60X0_REG_I2C_SLV2_DO,  0x48);
  // Put the enable command as last.将使能命令放在最后
  mpu_set( MPU60X0_REG_I2C_SLV2_CTRL,
           (1 << 7) |		// Slave 2 enable从机2写1字节
           (0 << 6) |		// Byte Swap
           (1 << 5) |		// Rig Dis: Write Only
           (1 << 0) );		// Write 1 byte

  // Reduced rate request For Temperature: Overwrites the Pressure Request
  //向温度端发出减小速度的请求：重新写气压计请求
  mpu_set( MPU60X0_REG_I2C_SLV3_ADDR, (MS5611_ADDR0));
  mpu_set( MPU60X0_REG_I2C_SLV3_DO,  0x58);
  // Put the enable command as last.
  mpu_set( MPU60X0_REG_I2C_SLV3_CTRL,
           (1 << 7) |		// Slave 3 enable从机3写1字节
           (0 << 6) |		// Byte Swap
           (1 << 5) |		// Rig Dis: Write Only
           (1 << 0) );		// Write 1 byte

  mpu_set( MPU60X0_REG_I2C_SLV1_ADDR, (MS5611_ADDR0) | MPU60X0_SPI_READ);
  mpu_set( MPU60X0_REG_I2C_SLV1_REG,  MS5611_REG_ADCREAD);
  // Put the enable command as last.
  mpu_set( MPU60X0_REG_I2C_SLV1_CTRL,
           (1 << 7) |		// Slave 1 enable从机1读6字节
           (0 << 6) |		// Byte Swap
           (3 << 0) );		// Read 6 bytes

#endif // read MS5611 as MPU slave

#endif

}

