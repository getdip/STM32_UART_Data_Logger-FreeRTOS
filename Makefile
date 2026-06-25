CC=arm-none-eabi-gcc
TARGET=cortex-m4
CFLAGS= -c -mcpu=$(TARGET) -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -std=gnu11 -Wall -O0 -DSTM32F407xx -DUSE_HAL_DRIVER
LDFLAGS= -T  STM32F407VGTX_FLASH.ld -Wl,-Map=final.map -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mcpu=cortex-m4 -mthumb

INCLUDES = \
-I./Core/Inc \
-I./CMSIS/Include \
-I./CMSIS/Device/ST/STM32F4xx/Include \
-I./Drivers/STM32F4xx_HAL_Driver/Inc \
-I./Drivers/STM32F4xx_HAL_Driver/Legacy \
-I./ThirdParty/FreeRTOS/ \
-I./ThirdParty/FreeRTOS/include \
-I./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F

CFLAGS += $(INCLUDES)
CFLAGS += -g3

VPATH = \
./Core/Src \
./Core/Startup \
./Drivers/STM32F4xx_HAL_Driver/Src \
./ThirdParty/FreeRTOS/ \
./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F \
./ThirdParty/FreeRTOS/portable/MemMang


#main_T=main.c main.h FreeRTOSConfig.h FreeRTOS.h semphr.h task.h queue.h

core_obj_files=stm32f4xx_hal_msp.o stm32f4xx_hal_timebase_tim.o stm32f4xx_it.o syscalls.o sysmem.o system_stm32f4xx.o
startup_obj_file=startup_stm32f407vgtx.o
drivers_obj_files=stm32f4xx_hal.o stm32f4xx_hal_cortex.o stm32f4xx_hal_dma.o stm32f4xx_hal_dma_ex.o stm32f4xx_hal_exti.o stm32f4xx_hal_flash.o stm32f4xx_hal_flash_ex.o stm32f4xx_hal_flash_ramfunc.o stm32f4xx_hal_gpio.o stm32f4xx_hal_i2c.o stm32f4xx_hal_i2c_ex.o stm32f4xx_hal_pwr.o stm32f4xx_hal_pwr_ex.o stm32f4xx_hal_rcc.o stm32f4xx_hal_rcc_ex.o stm32f4xx_hal_tim.o stm32f4xx_hal_tim_ex.o stm32f4xx_hal_uart.o
FreeRTOS_Kernel_obj_files=croutine.o event_groups.o list.o queue.o stream_buffer.o tasks.o timers.o
FreeRTOS_port_obj_files=port.o
FreeRTOS_heap_obj_files=heap_4.o

all:main.o $(core_obj_files) $(startup_obj_file) $(drivers_obj_files) $(FreeRTOS_Kernel_obj_files) $(FreeRTOS_port_obj_files) $(FreeRTOS_heap_obj_files) final.elf

%.o: %.c
	@$(CC) $(CFLAGS) $< -o $@

%.o: %.s
	@$(CC) $(CFLAGS) $< -o $@


final.elf:main.o $(core_obj_files) $(startup_obj_file) $(drivers_obj_files) $(FreeRTOS_Kernel_obj_files) $(FreeRTOS_port_obj_files) $(FreeRTOS_heap_obj_files)
	@$(CC) $(LDFLAGS) main.o $(core_obj_files) $(startup_obj_file) $(drivers_obj_files) $(FreeRTOS_Kernel_obj_files) $(FreeRTOS_port_obj_files) $(FreeRTOS_heap_obj_files) -o final.elf
	

clean:
	@rm -f *.o *.elf *.map