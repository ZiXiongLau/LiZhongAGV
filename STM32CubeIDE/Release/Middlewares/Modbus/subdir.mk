################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mb.c \
D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mbcommon.c \
D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mbcrc.c \
D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mbpdu.c \
D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mbtcp.c \
D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mbtcpserver.c 

OBJS += \
./Middlewares/Modbus/mb.o \
./Middlewares/Modbus/mbcommon.o \
./Middlewares/Modbus/mbcrc.o \
./Middlewares/Modbus/mbpdu.o \
./Middlewares/Modbus/mbtcp.o \
./Middlewares/Modbus/mbtcpserver.o 

C_DEPS += \
./Middlewares/Modbus/mb.d \
./Middlewares/Modbus/mbcommon.d \
./Middlewares/Modbus/mbcrc.d \
./Middlewares/Modbus/mbpdu.d \
./Middlewares/Modbus/mbtcp.d \
./Middlewares/Modbus/mbtcpserver.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Modbus/mb.o: D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mb.c Middlewares/Modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu99 -DUSE_HAL_DRIVER -DSTM32F767xx -DHAL_IWDG_MODULE_ENABLED -c -I../../Core/Inc -I../../LWIP/App -I../../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -I../../Drivers/CMSIS/Include -I../../App -I../../Middlewares/Third_Party/Modbus -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Modbus/mbcommon.o: D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mbcommon.c Middlewares/Modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu99 -DUSE_HAL_DRIVER -DSTM32F767xx -DHAL_IWDG_MODULE_ENABLED -c -I../../Core/Inc -I../../LWIP/App -I../../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -I../../Drivers/CMSIS/Include -I../../App -I../../Middlewares/Third_Party/Modbus -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Modbus/mbcrc.o: D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mbcrc.c Middlewares/Modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu99 -DUSE_HAL_DRIVER -DSTM32F767xx -DHAL_IWDG_MODULE_ENABLED -c -I../../Core/Inc -I../../LWIP/App -I../../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -I../../Drivers/CMSIS/Include -I../../App -I../../Middlewares/Third_Party/Modbus -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Modbus/mbpdu.o: D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mbpdu.c Middlewares/Modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu99 -DUSE_HAL_DRIVER -DSTM32F767xx -DHAL_IWDG_MODULE_ENABLED -c -I../../Core/Inc -I../../LWIP/App -I../../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -I../../Drivers/CMSIS/Include -I../../App -I../../Middlewares/Third_Party/Modbus -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Modbus/mbtcp.o: D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mbtcp.c Middlewares/Modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu99 -DUSE_HAL_DRIVER -DSTM32F767xx -DHAL_IWDG_MODULE_ENABLED -c -I../../Core/Inc -I../../LWIP/App -I../../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -I../../Drivers/CMSIS/Include -I../../App -I../../Middlewares/Third_Party/Modbus -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Modbus/mbtcpserver.o: D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/Middlewares/Third_Party/Modbus/mbtcpserver.c Middlewares/Modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu99 -DUSE_HAL_DRIVER -DSTM32F767xx -DHAL_IWDG_MODULE_ENABLED -c -I../../Core/Inc -I../../LWIP/App -I../../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -I../../Drivers/CMSIS/Include -I../../App -I../../Middlewares/Third_Party/Modbus -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Modbus

clean-Middlewares-2f-Modbus:
	-$(RM) ./Middlewares/Modbus/mb.cyclo ./Middlewares/Modbus/mb.d ./Middlewares/Modbus/mb.o ./Middlewares/Modbus/mb.su ./Middlewares/Modbus/mbcommon.cyclo ./Middlewares/Modbus/mbcommon.d ./Middlewares/Modbus/mbcommon.o ./Middlewares/Modbus/mbcommon.su ./Middlewares/Modbus/mbcrc.cyclo ./Middlewares/Modbus/mbcrc.d ./Middlewares/Modbus/mbcrc.o ./Middlewares/Modbus/mbcrc.su ./Middlewares/Modbus/mbpdu.cyclo ./Middlewares/Modbus/mbpdu.d ./Middlewares/Modbus/mbpdu.o ./Middlewares/Modbus/mbpdu.su ./Middlewares/Modbus/mbtcp.cyclo ./Middlewares/Modbus/mbtcp.d ./Middlewares/Modbus/mbtcp.o ./Middlewares/Modbus/mbtcp.su ./Middlewares/Modbus/mbtcpserver.cyclo ./Middlewares/Modbus/mbtcpserver.d ./Middlewares/Modbus/mbtcpserver.o ./Middlewares/Modbus/mbtcpserver.su

.PHONY: clean-Middlewares-2f-Modbus
