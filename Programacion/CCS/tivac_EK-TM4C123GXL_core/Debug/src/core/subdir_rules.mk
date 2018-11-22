################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
src/core/HardwareSerial.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/HardwareSerial.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -fno-rtti $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/IPAddress.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/IPAddress.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -fno-rtti $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/MACAddress.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/MACAddress.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -fno-rtti $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/Print.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/Print.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -fno-rtti $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/Stream.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/Stream.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -fno-rtti $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/Tone.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/Tone.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -fno-rtti $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/WInterrupts.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/WInterrupts.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/WMath.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/WMath.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -fno-rtti $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/WString.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/WString.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -fno-rtti $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/itoa.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/itoa.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/main.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/main.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -fno-rtti $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/new.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/new.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -fno-rtti $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/random.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/random.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/startup_gcc.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/startup_gcc.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/wiring.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/wiring.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/wiring_analog.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/wiring_analog.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/wiring_digital.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/wiring_digital.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/wiring_pulse.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/wiring_pulse.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/core/wiring_shift.o: C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac/wiring_shift.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-exceptions -Dprintf=iprintf -DF_CPU=80000000L -DENERGIA_EK_TM4C123GXL -DENERGIA_ARCH_TIVAC -DENERGIA=21 -DARDUINO=10610 -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/variants/EK-TM4C123GXL" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/driverlib" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system/inc" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/system" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/hardware/tivac/1.0.3/cores/tivac" -I"C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/tivac_EK-TM4C123GXL_core" -I"C:/Users/Andres/AppData/Local/Energia15/packages/energia/tools/arm-none-eabi-gcc/6.3.1-20170620/arm-none-eabi/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -fno-threadsafe-statics --param max-inline-insns-single=500 -mabi=aapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

