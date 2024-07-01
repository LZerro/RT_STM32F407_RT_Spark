# STM32 系列 BSP 制作教程
## 1.下载RT-Thread源文件
> https://github.com/RT-Thread/rt-thread
## 2.在源文件里找到模板工程
![图片](/09_STM32系列BSP制作教程/image/模板工程.png)    
> 复制一份模板工程
## 3.配置CubeMX文件
![图片](/09_STM32系列BSP制作教程/image/CubeMX0.png)  
![图片](/09_STM32系列BSP制作教程/image/CubeMX.png)  
![图片](/09_STM32系列BSP制作教程/image/CubeMX1.png)  
![图片](/09_STM32系列BSP制作教程/image/CubeMX2.png)  
![图片](/09_STM32系列BSP制作教程/image/CubeMX3.png)  
![图片](/09_STM32系列BSP制作教程/image/CubeMX4.png)  
![图片](/09_STM32系列BSP制作教程/image/CubeMX5.png)  
## 4. 拷贝初始化函数
> 在 **board.c** 文件中存放了函数 `SystemClock_Config()` ，该函数负责初始化系统时钟。当使用 CubeMX 工具对系统时钟重新配置的时候，需要更新这个函数。  
该函数由 CubeMX 工具生成，默认存放在`board/CubeMX_Config/Src/main.c` 文件中。但是该文件并没有被包含到我们的工程中，因此需要将这个函数从 main.c 中拷贝到 board.c 文件中。在整个 BSP 的制作过程中，这个函数是唯一要要拷贝的函数，该函数内容如下所示：  
![图片](/09_STM32系列BSP制作教程/image/复制时钟配置.png) 
![图片](/09_STM32系列BSP制作教程/image/复制时钟配置1.png) 
## 5. 配置FLASH 和 RAM 的相关参数
在 **board.h** 文件中配置了 FLASH 和 RAM 的相关参数，这个文件中需要修改的是 `STM32_FLASH_SIZE` 和 `STM32_SRAM_SIZE` 这两个宏控制的参数。本次制作的 BSP 所用的 STM32F407ZGT6 芯片的 flash 大小为 1024k，ram 的大小为 128k，因此对该文件作出如下的修改：  
![图片](/09_STM32系列BSP制作教程/image/board头文件配置.png)  
## 6. 配置board中的Kconfig
![图片](/09_STM32系列BSP制作教程/image/boardKconfig.png)  
## 7.修改链接脚本
![图片](/09_STM32系列BSP制作教程/image/链接脚本.png)  
![图片](/09_STM32系列BSP制作教程/image/链接脚本1.png)  
![图片](/09_STM32系列BSP制作教程/image/链接脚本2.png)  
## 8.修改构建脚本
> **SConscript** 脚本决定 MDK/IAR 工程的生成以及编译过程中要添加文件。
在这一步中需要修改芯片型号以及芯片启动文件的地址，修改内容如下图所示：
![图片](/09_STM32系列BSP制作教程/image/BoardSconscript.png)  
## 9.拷贝rt-thread文件
![图片](/09_STM32系列BSP制作教程/image/新建文件夹.png)  
![图片](/09_STM32系列BSP制作教程/image/复制源文件.png)  
![图片](/09_STM32系列BSP制作教程/image/复制源文件1.png)  
## 10.更改Kconfig文件
![图片](/09_STM32系列BSP制作教程/image/根目录Kconfig.png)  
![图片](/09_STM32系列BSP制作教程/image/rt-threadKconfig.png)  
![图片](/09_STM32系列BSP制作教程/image/KconfigBoard1.png)  
![图片](/09_STM32系列BSP制作教程/image/Kconfig配置完成进入Menuconfig.png)  
## 11.SConstruct指定rt-thread根目录
![图片](/09_STM32系列BSP制作教程/image/指定根目录RTT.png)  
## 12.生成mdk
![图片](/09_STM32系列BSP制作教程/image/生成mdk5.png)  

