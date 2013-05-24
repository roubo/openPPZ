openPPZ
=======

Add the MavLink for the open source UAV project Paparazzi，By San412.
---------------------------------------------------------------------
我们在为Paparazzi飞控项目添加MavLink协议
长春理工大学 光电工程学院 创新实验室 412

orignal ppz introduce
=====================


aparazzi is an attempt to develop a free software Unmanned (Air) Vehicle System.
 As of today the system is being used successfuly by a number of hobyists, universities and companies all over the world, on vehicle of various size ( 100g to 25Kg ) and of various nature ( fixed wing, rotorcrafts, boats and surface vehicles).

Up to date information is available in the wiki http://paparazzi.enac.fr

and from the mailing list [paparazzi-devel@nongnu.org] (http://savannah.nongnu.org/mail/?group=paparazzi)
and the IRC channel (freenode, #paparazzi).


Required Software
-----------------

Installation is described in the wiki (http://paparazzi.enac.fr/wiki/Installation).

For Ubuntu users, required packages are available in the [paparazzi-uav PPA] (https://launchpad.net/~paparazzi-uav/+archive/ppa),
Debian users can use http://paparazzi.enac.fr/debian


- **paparazzi-dev** is the meta-package that depends on everything needed to compile and run the ground segment and the simulator.
- **paparazzi-arm-multilib** ARM cross-compiling toolchain for LPC21 and STM32 based boards.
- **paparazzi-omap** toolchain for the optional Gumstix Overo module available on lisa/L.
- **paparazzi-jsbsim** is needed for using JSBSim as flight dynamic model for the simulator.


Directories quick and dirty description:
----------------------------------------

_conf_: the configuration directory (airframe, radio, ... descriptions).

_data_: where to put read-only data (e.g. maps, terrain elevation files, icons)

_doc_: documentation (diagrams, manual source files, ...)

_sw_: software (onboard, ground station, simulation, ...)

_var_: products of compilation, cache for the map tiles, ...


Compilation and demo simulation
-------------------------------

1. type "make" in the top directory to compile all the libraries and tools.

2. "./paparazzi" to run the Paparazzi Center

3. Select the "Microjet" aircraft in the upper-left A/C combo box.
  Select "sim" from upper-middle "target" combo box. Click "Build".
  When the compilation is finished, select "Simulation" from
  the upper-right session combo box and click "Execute".

4. In the GCS, wait about 10s for the aircraft to be in the "Holding point" navigation block.
  Switch to the "Takeoff" block (lower-left blue airway button in the strip).
  Takeoff with the green launch button.

Uploading of the embedded software
----------------------------------

1. Power the flight controller board while it is connected to the PC with the USB cable.

2. From the Paparazzi center, select the "ap" target, and click "Upload".


Flight
------

1.  From the Paparazzi Center, select the flight session and ... do the same than in simulation !

openPPZ分析
===========
分析第一步
-----------
分析代码结构，摸清系统架构。

	data		author		content						path

	2013/05/07	xiaoxin		分析了sw文件夹下的源码，对分析该文件夹		sw/airborne/arch/stm32/ReadMe.txt
					下的源码，参考原英文注释对部分源文件作
					出中文注释，并对该文件夹下的代码架构做
					出了简要的总结
	2013/05/08	chorushe	分析了框架层的架构和部分代码			NULL
	2013/05/08	roubo		确定了代码分析的优先顺序			NULL
	2013/05/11	xiaoxin		分析了旋翼机的主循环，熟悉了基本的执行流程	sw/airborne/firmwares/rotorcraft/README
	2013/05/13 	roubo 		分析了周期性任务运作等 				NULL
	2013/05/16 	roubo 		地理坐标系paper 				doc/pprz_geodetic/headfile.pdf
	2013/05/16 	roubo 		代数计算方法paper 				doc/pprz_algebra/headfile.pdf
	2013/05/17	xiaoxin		分析了各种坐标系的相互转换原理			sw/airborne/math
	2013/05/17	chorushe	分析了算法函数的代数运算方法			sw/airborne/math
	2013/05/24	xiaoxin		分析了imu传感单元的通讯方式			sw/airborne/subsystem/imu/
