# ECD 401 High Reliability NFC Battery Connector Project Firmware

This folder contains the code necessary to reconstruct the ECD 401 final project demonstration. 

There are two sets of firmware
- `TestAP2P_T` is the STM32CubeIDE project that implements the "truck" side of the system
- `TestAP2P_B` implements the "battery" side of the system.

> ❗️ Do NOT run the STM32CubeIDE project configuration tool without backing up all code files
> This will cause various driver files to be over-written with their factory default code, removing some necessary modifications for the project

## How to use
- Install STM32CubeIDE
- Import the project into CubeIDE via File -> Import from filesystem
  - All necessary RFAL (RF abstraction layer) drivers and board support files will be imported
- Run project on NUCLEO-L476RG board as usual for any project via Project -> Run
- See accompanying project design files and bill of materials for complete requirements to construct and run the project successfully

## Project Structure
The project is based off of the X-CUBE-NFC-6 software provided by ST and follows a similar file structure.  The files where project code has been added are
- `X-CUBE-NFC6/Target/demo_polling.c`: this is the main implementation of the project application.
- `X-CUBE-NFC6/Target/rfal_defConfig.h`: NFC hardware protocol configuration
- `Core/Src/main.c`: contains main software entry point and configures various hardware aspects
- `Core/Inc/main.h`: configuration and global variables 
- `Core/Src/ring_buffer.c`: implementation of double ring buffer structure used in CAN data transceiving

Various files in `Middlewares/` and `Drivers/` have been modified from their original state to correct errors or add functionality. It is extremely important to back up all project files *before* modifying the `.ioc` file via Pinout and Configuration so that project code isn't overwritten with factory default code. 