#! /bin/bash

PICO_SDK_PATH=../../pico-sdk
PICO_TOOL_PATH=../../pico-tool
FreeRTOS_PATH=../../FreeRTOS-Kernel

if [ ! -d build ]; then
  mkdir build;
fi

cd build

cmake .. -DPICO_SDK_PATH=$PICO_SDK_PATH \
                    -DPICO_BOARD=pico2 \
                    -Dpicotool_DIR=$PICO_TOOL_PATH \
                    -DFREERTOS_KERNEL_PATH=$FreeRTOS_PATH \
                    -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
                    && make