################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/LWIP/App/lwip.c 

OBJS += \
./Application/User/LWIP/App/lwip.o 

C_DEPS += \
./Application/User/LWIP/App/lwip.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/LWIP/App/lwip.o: D:/LZX/WorkSpace/01_AGV/ufo_project_new/AebToCan/LWIP/App/lwip.c Application/User/LWIP/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu99 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -DHAL_IWDG_MODULE_ENABLED -c -I../../Core/Inc -I../../LWIP/App -I../../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -I../../Drivers/CMSIS/Include -I../../App -I../../Middlewares/Third_Party/Modbus -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-LWIP-2f-App

clean-Application-2f-User-2f-LWIP-2f-App:
	-$(RM) ./Application/User/LWIP/App/lwip.cyclo ./Application/User/LWIP/App/lwip.d ./Application/User/LWIP/App/lwip.o ./Application/User/LWIP/App/lwip.su

.PHONY: clean-Application-2f-User-2f-LWIP-2f-App

