################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../startup/startup_lpc802.c 

OBJS += \
./startup/startup_lpc802.o 

C_DEPS += \
./startup/startup_lpc802.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_LPC802M001JDH20 -DCPU_LPC802M001JDH20_cm0plus -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\board" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\source" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\drivers" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\device" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\CMSIS" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\component\uart" -I"C:\Users\jpagl\Documents\MCUXpressoIDE_11.1.1_3241\workspace\ignition_interlock\utilities" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


