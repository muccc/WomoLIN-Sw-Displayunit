##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.7.1] date: [Sat Apr 11 20:22:26 CEST 2020] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = rfguydisplay


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Src/main.c \
Src/touch.c \
Src/stm32l4xx_it.c \
Src/stm32l4xx_hal_msp.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
Src/system_stm32l4xx.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_ltdc.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_ltdc_ex.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dsi.c \
Drivers/lvgl/src/lv_core/lv_obj.c \
Drivers/lvgl/src/lv_core/lv_group.c \
Drivers/lvgl/src/lv_core/lv_debug.c \
Drivers/lvgl/src/lv_core/lv_disp.c \
Drivers/lvgl/src/lv_core/lv_indev.c \
Drivers/lvgl/src/lv_core/lv_refr.c \
Drivers/lvgl/src/lv_core/lv_style.c \
Drivers/lvgl/src/lv_misc/lv_mem.c \
Drivers/lvgl/src/lv_misc/lv_fs.c \
Drivers/lvgl/src/lv_misc/lv_task.c \
Drivers/lvgl/src/lv_misc/lv_anim.c \
Drivers/lvgl/src/lv_misc/lv_ll.c \
Drivers/lvgl/src/lv_misc/lv_math.c \
Drivers/lvgl/src/lv_theme/lv_area.c \
Drivers/lvgl/src/lv_theme/lv_async.c \
Drivers/lvgl/src/lv_theme/lv_bidi.c \
Drivers/lvgl/src/lv_theme/lv_color.c \
Drivers/lvgl/src/lv_theme/lv_gc.c \
Drivers/lvgl/src/lv_theme/lv_log.c \
Drivers/lvgl/src/lv_theme/lv_printf.c \
Drivers/lvgl/src/lv_theme/lv_templ.c \
Drivers/lvgl/src/lv_theme/lv_txt.c \
Drivers/lvgl/src/lv_theme/lv_txt_ap.c \
Drivers/lvgl/src/lv_theme/lv_utils.c \
Drivers/lvgl/src/lv_themes/lv_theme.c \
Drivers/lvgl/src/lv_themes/lv_theme_material.c \
Drivers/lvgl/src/lv_hal/lv_hal_indev.c \
Drivers/lvgl/src/lv_hal/lv_hal_tick.c \
Drivers/lvgl/src/lv_hal/lv_hal_disp.c \
Drivers/lvgl/src/lv_draw/lv_img_decoder.c \
Drivers/lvgl/src/lv_draw/lv_draw_img.c \
Drivers/lvgl/src/lv_draw/lv_img_cache.c \
Drivers/lvgl/src/lv_draw/lv_draw_mask.c \
Drivers/lvgl/src/lv_draw/lv_draw_rect.c \
Drivers/lvgl/src/lv_draw/lv_draw_blend.c \
Drivers/lvgl/src/lv_draw/lv_draw_label.c \
Drivers/lvgl/src/lv_draw/lv_draw_line.c \
Drivers/lvgl/src/lv_draw/lv_img_buf.c \
Drivers/lvgl/src/lv_font/lv_font_montserrat_16.c \
Drivers/lvgl/src/lv_font/lv_font_fmt_txt.c \
Drivers/lvgl/src/lv_font/lv_font.c \
Drivers/lvgl/src/lv_widgets/lv_arc.c \
Drivers/lvgl/src/lv_widgets/lv_bar.c \
Drivers/lvgl/src/lv_widgets/lv_btn.c \
Drivers/lvgl/src/lv_widgets/lv_btnmatrix.c \
Drivers/lvgl/src/lv_widgets/lv_calendar.c \
Drivers/lvgl/src/lv_widgets/lv_canvas.c \
Drivers/lvgl/src/lv_widgets/lv_chart.c \
Drivers/lvgl/src/lv_widgets/lv_checkbox.c \
Drivers/lvgl/src/lv_widgets/lv_cont.c \
Drivers/lvgl/src/lv_widgets/lv_cpicker.c \
Drivers/lvgl/src/lv_widgets/lv_dropdown.c \
Drivers/lvgl/src/lv_widgets/lv_gauge.c \
Drivers/lvgl/src/lv_widgets/lv_imgbtn.c \
Drivers/lvgl/src/lv_widgets/lv_keyboard.c \
Drivers/lvgl/src/lv_widgets/lv_label.c \
Drivers/lvgl/src/lv_widgets/lv_led.c \
Drivers/lvgl/src/lv_widgets/lv_line.c \
Drivers/lvgl/src/lv_widgets/lv_linemeter.c \
Drivers/lvgl/src/lv_widgets/lv_list.c \
Drivers/lvgl/src/lv_widgets/lv_msgbox.c \
Drivers/lvgl/src/lv_widgets/lv_objmask.c \
Drivers/lvgl/src/lv_widgets/lv_objx_templ.c \
Drivers/lvgl/src/lv_widgets/lv_page.c \
Drivers/lvgl/src/lv_widgets/lv_roller.c \
Drivers/lvgl/src/lv_widgets/lv_slider.c \
Drivers/lvgl/src/lv_widgets/lv_spinbox.c \
Drivers/lvgl/src/lv_widgets/lv_spinner.c \
Drivers/lvgl/src/lv_widgets/lv_switch.c \
Drivers/lvgl/src/lv_widgets/lv_table.c \
Drivers/lvgl/src/lv_widgets/lv_tabview.c \
Drivers/lvgl/src/lv_widgets/lv_textarea.c \
Drivers/lvgl/src/lv_widgets/lv_tileview.c \
Drivers/lvgl/src/lv_widgets/lv_win.c \
Drivers/tft/tft.c

# ASM sources
ASM_SOURCES =  \
startup_stm32l4r9xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32L4R9xx


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32L4xx_HAL_Driver/Inc \
-IDrivers/STM32L4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32L4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/lvgl \
-IDrivers/tft

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32L4R9ZITx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
