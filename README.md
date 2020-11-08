# Lora-Sx1276-7-8
Lora SX1276/7/8 module driver interfacing with STM32F103C8 bluepill. Keil Software code with HAL library.

Initially goto MDK-ARM folder find lora_driver.uvprojx and opens it.
File opens in KEIL IDE.

Load the code to bluepill board

For receiving mode,
Comment below line in the main.c file

#define TX

Uncomment below line in the main.c file

#define RX

Then Buid the code and Load in to the second board.

Finally can observe the text transmitted by device1, In device2 received data is transmitted to the uart1.
Connect the uart port of two devices with laptop to see serial prints of the two devices.

Dowload files from the below link.
https://drive.google.com/drive/folders/1SBMDImwhij1O5k5U2pt1e0vbbq6LR9IR?usp=sharing


Detailed explanation in below link.
https://vinodembedded.wordpress.com/2020/11/07/loralibrary-stm32-keil/
