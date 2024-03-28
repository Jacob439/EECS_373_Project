################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/UGUI/arial_9X10.c \
../Drivers/UGUI/ugui.c \
../Drivers/UGUI/ugui_button.c \
../Drivers/UGUI/ugui_checkbox.c \
../Drivers/UGUI/ugui_image.c \
../Drivers/UGUI/ugui_progress.c \
../Drivers/UGUI/ugui_sim.c \
../Drivers/UGUI/ugui_sim_x11.c \
../Drivers/UGUI/ugui_textbox.c 

OBJS += \
./Drivers/UGUI/arial_9X10.o \
./Drivers/UGUI/ugui.o \
./Drivers/UGUI/ugui_button.o \
./Drivers/UGUI/ugui_checkbox.o \
./Drivers/UGUI/ugui_image.o \
./Drivers/UGUI/ugui_progress.o \
./Drivers/UGUI/ugui_sim.o \
./Drivers/UGUI/ugui_sim_x11.o \
./Drivers/UGUI/ugui_textbox.o 

C_DEPS += \
./Drivers/UGUI/arial_9X10.d \
./Drivers/UGUI/ugui.d \
./Drivers/UGUI/ugui_button.d \
./Drivers/UGUI/ugui_checkbox.d \
./Drivers/UGUI/ugui_image.d \
./Drivers/UGUI/ugui_progress.d \
./Drivers/UGUI/ugui_sim.d \
./Drivers/UGUI/ugui_sim_x11.d \
./Drivers/UGUI/ugui_textbox.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/UGUI/%.o Drivers/UGUI/%.su Drivers/UGUI/%.cyclo: ../Drivers/UGUI/%.c Drivers/UGUI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4R5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-UGUI

clean-Drivers-2f-UGUI:
	-$(RM) ./Drivers/UGUI/arial_9X10.cyclo ./Drivers/UGUI/arial_9X10.d ./Drivers/UGUI/arial_9X10.o ./Drivers/UGUI/arial_9X10.su ./Drivers/UGUI/ugui.cyclo ./Drivers/UGUI/ugui.d ./Drivers/UGUI/ugui.o ./Drivers/UGUI/ugui.su ./Drivers/UGUI/ugui_button.cyclo ./Drivers/UGUI/ugui_button.d ./Drivers/UGUI/ugui_button.o ./Drivers/UGUI/ugui_button.su ./Drivers/UGUI/ugui_checkbox.cyclo ./Drivers/UGUI/ugui_checkbox.d ./Drivers/UGUI/ugui_checkbox.o ./Drivers/UGUI/ugui_checkbox.su ./Drivers/UGUI/ugui_image.cyclo ./Drivers/UGUI/ugui_image.d ./Drivers/UGUI/ugui_image.o ./Drivers/UGUI/ugui_image.su ./Drivers/UGUI/ugui_progress.cyclo ./Drivers/UGUI/ugui_progress.d ./Drivers/UGUI/ugui_progress.o ./Drivers/UGUI/ugui_progress.su ./Drivers/UGUI/ugui_sim.cyclo ./Drivers/UGUI/ugui_sim.d ./Drivers/UGUI/ugui_sim.o ./Drivers/UGUI/ugui_sim.su ./Drivers/UGUI/ugui_sim_x11.cyclo ./Drivers/UGUI/ugui_sim_x11.d ./Drivers/UGUI/ugui_sim_x11.o ./Drivers/UGUI/ugui_sim_x11.su ./Drivers/UGUI/ugui_textbox.cyclo ./Drivers/UGUI/ugui_textbox.d ./Drivers/UGUI/ugui_textbox.o ./Drivers/UGUI/ugui_textbox.su

.PHONY: clean-Drivers-2f-UGUI

