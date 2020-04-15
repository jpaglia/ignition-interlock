################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/fsl_clock.c \
../drivers/fsl_common.c \
../drivers/fsl_gpio.c \
../drivers/fsl_power.c \
../drivers/fsl_reset.c \
../drivers/fsl_swm.c \
../drivers/fsl_usart.c 

OBJS += \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_gpio.o \
./drivers/fsl_power.o \
./drivers/fsl_reset.o \
./drivers/fsl_swm.o \
./drivers/fsl_usart.o 

C_DEPS += \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_gpio.d \
./drivers/fsl_power.d \
./drivers/fsl_reset.d \
./drivers/fsl_swm.d \
./drivers/fsl_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_LPC802M001JDH20 -DCPU_LPC802M001JDH20_cm0plus -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\board" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\source" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\drivers" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\device" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\CMSIS" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\component\uart" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\utilities" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


