Author : xiaoxin
Date   : 2013/05/07
Email  : talia_bian@san412.in
 
Directions:
     sw/airborne/arch/stm32/

goal：
     分析该文件夹下的源码，参考原英文注释对部分源文件作出中文注释，并对该文件夹下的代码架构做出
     一些总结，由于本人能力有限，其中还有很多地方理解不到位, 所以翻译的也必然会不正确，希望后续
     的阅码者能够对此做出改善和对该文档的进一步补充说明。

List:
    ——> peripherals 文件夹下主要是四个传感器的中断配置和接口配置
        1 . hmc5843.c/.h:
            hmc5843:电子罗盘/三轴加速度计。使用外部中断5和PB5
                    主要功能函数：hmc5843_arch_init();//hmc5843初始化
                                  hmc5843_arch_reset();//hmc5843 i2c配置 
                                  exti9_5_isr();//清中断         
        2 . ms2100_arch.c/.h:
            ms2100:双轴的地磁力计。使用外部中断5和PB5
                   主要功能函数：ms2100_arch_init();//ms2100初始化
                                 ms2100_reset_cb();//ms2100复位（加入复位时间）
                                 exti9_5_isr();//清中断
        3 . max1168_arch.c/.h:
            max1168:低功耗，多通道，16位逐次逼近型ADC芯片，使用外部中断2和PB2(V1.1）/PD2(V1.0）
                   主要功能函数：max1168_arch_init();//max1168初始化
                                 exti2_isr(）；//清中断
        4 . sc18is600_arch.c/.h:
            sc18is600:SPI到I2C的接口芯片
                   主要功能函数：sc18is600_arch_init（）；//配置从机（PB2),外部中断（PD2），SPI
                                 sc18is600_setup_SPI_DMA（）；//配置SPI的DMA通道
                                 sc18is600_transmit（）；//读函数
                                 sc18is600_tranceive（）//写函数
                                 sc18is600_read_from_register（）；//向寄存器写数据
                                 exti2_irq_handler（）；//清中断
                                 dma1_c4_irq_handler（）；//命令处理？？
   
  
     ——>mcu_periph 文件夹下主要是stm32内部外设的固件库，包括如下
        1 . adc
        2 . can 
        3 . gpio
        4 . i2c
        5 . spi
        6 . sys_time:系统时钟
        7 . uart 
        8 . led
        
        eg: uart.c:
            主要功能函数：uart_periph_set_baudrate();//配置串口模式
                          uart_transmit();//配置发送和接收模式
                          usart_isr();//
                          usart_enable_irq();//配置串口中断优先级
                          uart1_init();//以下为串口1~3,5的初始化配置
                          usart1_isr();
                          uart2_init();
                          usart2_isr();
                          uart3_init();
                          usart3_isr();
                          uart5_init();
			  uart5_isr();
      
     ——>subsystem 文件夹下主要是一些子系统的相关源码，包括如下：
        1 . actuators:执行机构，产生pwm波控制电机
            actuators.c/.h:
            主要功能函数：actuators_pwm_arch_channel_init（）；//pwm通道初始化
                          actuators_pwm_arch_init（）；//定时器3,4,5的初始化
                          actuators_pwm_commit（）；//根据执行机构的值设定PWM占空比
        2 . imu: 惯性测量单元（分两类aspirin 和 crista,由于实际使用的是aspirin，所以只对aspirin分析）
            imu_aspirin_arch.c/.h:陀螺仪使用的是EXTI14&GPIOC；加速度计使用的是EXTI2&GPIOB2
            主要功能函数：imu_aspirin_arch_int_enable();//陀螺仪和加速度计的中断使能
                          imu_aspirin_arch_int_disable();//陀螺仪和加速度计的中断失能
                          imu_aspirin_arch_init();//陀螺仪中断配置（EXTI14,GPIOC14）;加速度计中断配置（EXTI2，GPIOB2）
                          exti15_10_isr();//清外部中断14,对应于陀螺仪
                          exti2_isr();//清外部中断2,对应于加速度计 
        3 . radio_control:RC遥控器（分两类ppm和spektrum,这里使用的是ppm）
            ppm.c/.h: 
            宏定义部分：USE_PPM_TIM2:默认情况PPM使用的是PA_10(原先舵机的引脚)作为输入
                        USE_PPM_TIM1:也可以设定PPM使用的是PA_01(串口1接收)作为输入
            主要的功能函数：ppm_arch_init（）；//PPM使用的定时器的配置
                            tim2_isr（）；//如果使用的是定时器2作为ppm的话，清定时器2中断
                            tim1_up_isr（）；//如果使用的是定时器1作为ppm的话，清定时器1中断
                            tim1_cc_isr（）；//清定时器1和开始ppm数据解码？？
        4 . settings_arch.c/.h:stm32 flash 的相关设置
            主要功能函数：pflash_checksum（）；//
                          flash_detect（）；//检测flash的页大小
                          pflash_program_bytes（）；//
                          persistent_write（）；//永久写？？
 
                          persistent_read（）；//永久读？？
     ——>其它文件：
        1 . mcu_arch.c/.h:
            特殊头文件包含：#include BOARD_CONFIG 
            主要功能函数: mcu_arch_init();//包括中断向量表的地址和外部时钟的设置
        2 . led_hw.c/.h:主要是板载led的设定
                        .h文件里面：写明驱动led的两种模式：LED_STP08 移位寄存器方式；非LED_STP08 普通GPIO方式
        3 . gpio.h : 空
        4 . interrupt_hw.h :空
        5 . link_mcu_hw.h: arm7 处理器链接的简单处理
            主要功能函数：CrcUpdate（）；
        6 . 3个makefile文件：
            lisa-m.ld:  lisa-m 的链接脚本文件，定义它的RAM和ROM区域，且包含了一般的脚本文件libopencm3_stm32f1.ld
            lisa-l.ld:  lisa-l 的链接脚本文件，定义它的RAM和ROM区域，且包含了一般的脚本文件libopencm3_stm32f1.ld
            stm32default.ld: 默认的stm32架构的链接脚本文件，也包含了一般的脚本文件libopencm3_stm32f1.ld
        7 . TIM_usage_list.txt:
            这个文件里面说明了定时器的使用情况。有参考意义。
        
