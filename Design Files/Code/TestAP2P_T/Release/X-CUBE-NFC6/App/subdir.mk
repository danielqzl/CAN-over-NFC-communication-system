################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../X-CUBE-NFC6/App/app_x-cube-nfcx.c 

OBJS += \
./X-CUBE-NFC6/App/app_x-cube-nfcx.o 

C_DEPS += \
./X-CUBE-NFC6/App/app_x-cube-nfcx.d 


# Each subdirectory must supply rules for building sources it contributes
X-CUBE-NFC6/App/%.o X-CUBE-NFC6/App/%.su X-CUBE-NFC6/App/%.cyclo: ../X-CUBE-NFC6/App/%.c X-CUBE-NFC6/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DST25R3916B -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../X-CUBE-NFC6/App -I../X-CUBE-NFC6/Target -I../Core/Inc -I../Drivers/BSP/STM32L4xx_Nucleo -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/ST25R3916 -I../Drivers/BSP/NFC08A1 -I../Middlewares/ST/rfal/Inc -I../Middlewares/ST/rfal/Src -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-X-2d-CUBE-2d-NFC6-2f-App

clean-X-2d-CUBE-2d-NFC6-2f-App:
	-$(RM) ./X-CUBE-NFC6/App/app_x-cube-nfcx.cyclo ./X-CUBE-NFC6/App/app_x-cube-nfcx.d ./X-CUBE-NFC6/App/app_x-cube-nfcx.o ./X-CUBE-NFC6/App/app_x-cube-nfcx.su

.PHONY: clean-X-2d-CUBE-2d-NFC6-2f-App

