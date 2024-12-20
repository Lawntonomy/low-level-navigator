# Base image
FROM ubuntu:22.04

# Set up environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PICO_SDK_PATH=/pico-sdk
ENV PICO_TOOL_PATH=/pico-tool
ENV FreeRTOS_PATH=/freertos

# Install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    git \
    python3 \
    python3-pip \
    python3-venv \
    libusb-1.0-0-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Clone the Pico SDK
RUN git clone --branch master https://github.com/raspberrypi/pico-sdk.git $PICO_SDK_PATH && \
    cd $PICO_SDK_PATH && \
    git submodule update --init

# Clone Pico tool
RUN git clone --branch master https://github.com/raspberrypi/picotool.git $PICO_TOOL_PATH && \
    cd $PICO_TOOL_PATH && \
    git submodule update --init

# Clone FreeRTOS
RUN git clone --branch main https://github.com/raspberrypi/FreeRTOS-Kernel.git $FreeRTOS_PATH && \
    cd $FreeRTOS_PATH && \
    git submodule update --init

# Set the working directory for the container
WORKDIR /workspace

# Default command
CMD mkdir -p build && cd build && cmake .. -DPICO_SDK_PATH=$PICO_SDK_PATH \
                                        -DPICO_BOARD=pico2 \
                                        -Dpicotool_DIR=$PICO_TOOL_PATH \
                                        -DFREERTOS_KERNEL_PATH=$FreeRTOS_PATH \
                                        -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
                                        && make
