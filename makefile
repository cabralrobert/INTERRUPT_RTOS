all: folder configPkg app

FOLDER_USER = /home/$(USER)

CC= $(FOLDER_USER)/ti/gcc-arm-none-eabi-6-2017-q1-update/bin/arm-none-eabi-gcc
OBJCOPY= $(FOLDER_USER)/ti/gcc-arm-none-eabi-6-2017-q1-update/bin/arm-none-eabi-objcopy
APP_NAME=gpio_rtos

XS = $(FOLDER_USER)/ti/xdctools_3_50_03_33_core/xs
BIOS_FOLDER = $(FOLDER_USER)/ti/bios_6_52_00_12/packages
PDK_FOLDER = $(FOLDER_USER)/ti/pdk_am335x_1_0_9/packages
EDMA_FOLDER = $(FOLDER_USER)/ti/edma3_lld_2_12_05_30B/packages
NDK_FOLDER = $(FOLDER_USER)/ti/ndk_2_26_00_08/packages
CROSS_FOLDER = $(FOLDER_USER)/ti/gcc-arm-none-eabi-6-2017-q1-update
CFG_FILE = conf/am335x_app_bbbam335x.cfg
CONF_PKG_FLAGS = xdc.tools.configuro -o configPkg -t gnu.targets.arm.A8F -p ti.platforms.evmAM3359 -r debug

TIIMAGE= $(PDK_FOLDER)/ti/starterware/tools/ti_image/tiimage

CFLAGS= -mtune=cortex-a8 -marm -Dam3359 -DSOC_AM335x -DbbbAM335x -g -gdwarf-3 -gstrict-dwarf -Wall -finstrument-functions -mfloat-abi=hard -Wl,-Map,"bin/$(APP_NAME).map" -nostartfiles -static -Wl,--gc-sections -L"$(BIOS_FOLDER)/gnu/targets/arm/libs/install-native/arm-none-eabi/lib/hard" -Wl,--defsym,STACKSIZE=0x1C000 -Wl,--defsym,HEAPSIZE=0x400 --specs=nano.specs

FLAGS_OBJ= -c -mcpu=cortex-a8 -mtune=cortex-a8 -march=armv7-a -marm -mfloat-abi=hard -Dam3359 -DSOC_AM335x -DbbbAM335x -I"$(FOLDER_USER)/ti/gcc-arm-none-eabi-6-2017-q1-update/arm-none-eabi/include" -I"inc/" -g -gdwarf-3 -gstrict-dwarf -Wall -finstrument-functions -MMD -MP

LINKER_FLAGS= -Wl,-T"configPkg/linker.cmd" -Wl,--start-group -lrdimon -lgcc -lm -lnosys -lc -Wl,--end-group

IMG_LOAD_ADDR = 0x80000000

SRC= $(wildcard src/*.c)
OBJ:= $(SRC:src/%.c=obj/%.o)
LABELS= GPIO_soc \
				GPIO_log \
				I2C_soc \
				GPIO_bbbAM335x_board \
				UART_soc \
				ADC \
				GPIO \
				main_led_blink

configPkg:
	@echo "Generating: $@"
	@+$(XS) --xdcpath="$(BIOS_FOLDER);$(PDK_FOLDER);$(EDMA_FOLDER);$(NDK_FOLDER)" $(CONF_PKG_FLAGS) -c $(CROSS_FOLDER) $(CFG_FILE)

app: $(LABELS)
	@echo "Linking: $(OBJ)"
	@$(CC) $(CFLAGS) -o bin/$(APP_NAME).out $(OBJ) $(LINKER_FLAGS)
	@echo "Generating: bin/$(APP_NAME)_ti.bin"
	@$(OBJCOPY) -O binary bin/$(APP_NAME).out bin/$(APP_NAME)_ti.bin
	@echo "Generating: bin/app"
	@$(TIIMAGE) $(IMG_LOAD_ADDR) NONE bin/$(APP_NAME)_ti.bin bin/app
	@rm -rf ~/pastaRTOS/app
	@cp bin/app ~/pastaRTOS/

GPIO_soc: src/GPIO_soc.c
	@echo "Compiling: $@"
	@$(CC) $(FLAGS_OBJ) -MF"obj/$@.d" -MT"obj/$@.o" -o"obj/$@.o" @"configPkg/compiler.opt" $<

GPIO_log: src/GPIO_log.c
	@echo "Compiling: $@"
	@$(CC) $(FLAGS_OBJ) -MF"obj/$@.d" -MT"obj/$@.o" -o"obj/$@.o" @"configPkg/compiler.opt" $<

I2C_soc: src/I2C_soc.c
	@echo "Compiling: $@"
	@$(CC) $(FLAGS_OBJ) -MF"obj/$@.d" -MT"obj/$@.o" -o"obj/$@.o" @"configPkg/compiler.opt" $<

GPIO_bbbAM335x_board: src/GPIO_bbbAM335x_board.c
	@echo "Compiling: $@"
	@$(CC) $(FLAGS_OBJ) -MF"obj/$@.d" -MT"obj/$@.o" -o"obj/$@.o" @"configPkg/compiler.opt" $<

UART_soc: src/UART_soc.c
	@echo "Compiling: $@"
	@$(CC) $(FLAGS_OBJ) -MF"obj/$@.d" -MT"obj/$@.o" -o"obj/$@.o" @"configPkg/compiler.opt" $<

ADC: src/ADC.c
	@echo "Compiling: $@"
	@$(CC) $(FLAGS_OBJ) -MF"obj/$@.d" -MT"obj/$@.o" -o"obj/$@.o" @"configPkg/compiler.opt" $<

GPIO: src/GPIO.c
	@echo "Compiling: $@"
	@$(CC) $(FLAGS_OBJ) -MF"obj/$@.d" -MT"obj/$@.o" -o"obj/$@.o" @"configPkg/compiler.opt" $<


main_led_blink: src/main_led_blink.c
	@echo "Compiling: $@"
	@$(CC) $(FLAGS_OBJ) -MF"obj/$@.d" -w -MT"obj/$@.o" -o"obj/$@.o" @"configPkg/compiler.opt" $<

clean:
	@rm -rf configPkg/ obj/ bin/

folder:
	@mkdir -p obj/ bin/
