################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/step_src/StepCountingAlgo.c \
../Core/Src/step_src/detectionStage.c \
../Core/Src/step_src/filterStage.c \
../Core/Src/step_src/motionDetectStage.c \
../Core/Src/step_src/postProcessingStage.c \
../Core/Src/step_src/preProcessingStage.c \
../Core/Src/step_src/ringbuffer.c \
../Core/Src/step_src/scoringStage.c 

OBJS += \
./Core/Src/step_src/StepCountingAlgo.o \
./Core/Src/step_src/detectionStage.o \
./Core/Src/step_src/filterStage.o \
./Core/Src/step_src/motionDetectStage.o \
./Core/Src/step_src/postProcessingStage.o \
./Core/Src/step_src/preProcessingStage.o \
./Core/Src/step_src/ringbuffer.o \
./Core/Src/step_src/scoringStage.o 

C_DEPS += \
./Core/Src/step_src/StepCountingAlgo.d \
./Core/Src/step_src/detectionStage.d \
./Core/Src/step_src/filterStage.d \
./Core/Src/step_src/motionDetectStage.d \
./Core/Src/step_src/postProcessingStage.d \
./Core/Src/step_src/preProcessingStage.d \
./Core/Src/step_src/ringbuffer.d \
./Core/Src/step_src/scoringStage.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/step_src/%.o Core/Src/step_src/%.su Core/Src/step_src/%.cyclo: ../Core/Src/step_src/%.c Core/Src/step_src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-step_src

clean-Core-2f-Src-2f-step_src:
	-$(RM) ./Core/Src/step_src/StepCountingAlgo.cyclo ./Core/Src/step_src/StepCountingAlgo.d ./Core/Src/step_src/StepCountingAlgo.o ./Core/Src/step_src/StepCountingAlgo.su ./Core/Src/step_src/detectionStage.cyclo ./Core/Src/step_src/detectionStage.d ./Core/Src/step_src/detectionStage.o ./Core/Src/step_src/detectionStage.su ./Core/Src/step_src/filterStage.cyclo ./Core/Src/step_src/filterStage.d ./Core/Src/step_src/filterStage.o ./Core/Src/step_src/filterStage.su ./Core/Src/step_src/motionDetectStage.cyclo ./Core/Src/step_src/motionDetectStage.d ./Core/Src/step_src/motionDetectStage.o ./Core/Src/step_src/motionDetectStage.su ./Core/Src/step_src/postProcessingStage.cyclo ./Core/Src/step_src/postProcessingStage.d ./Core/Src/step_src/postProcessingStage.o ./Core/Src/step_src/postProcessingStage.su ./Core/Src/step_src/preProcessingStage.cyclo ./Core/Src/step_src/preProcessingStage.d ./Core/Src/step_src/preProcessingStage.o ./Core/Src/step_src/preProcessingStage.su ./Core/Src/step_src/ringbuffer.cyclo ./Core/Src/step_src/ringbuffer.d ./Core/Src/step_src/ringbuffer.o ./Core/Src/step_src/ringbuffer.su ./Core/Src/step_src/scoringStage.cyclo ./Core/Src/step_src/scoringStage.d ./Core/Src/step_src/scoringStage.o ./Core/Src/step_src/scoringStage.su

.PHONY: clean-Core-2f-Src-2f-step_src

