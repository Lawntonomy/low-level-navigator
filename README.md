# Low Level Navigator

![build workflow](https://github.com/atkirschenman/low-level-navigator/actions/workflows/build.yml/badge.svg)

## To Build:

Ensure Docker is installed

' docker build -t pico-builder . '

### Change path to match location
docker run --rm -v ~/Documents/low-level-navigator:/workspace pico-builder

## LLN Architecture 

The LLN is a state machine, meaning that different operations are separated at the core of the software

### Pre-Calibration Idle

No commands
Zero speed
No valid calibration 

### Calibration

Going through calibration process

### Idle Navigation

valid calibration
No guidance commands

### Active Guidance

Actively receiving and executing guidance commands

### Exiting Guidance

No guidance commands recieved
Still executing commands
Bring system to 0
