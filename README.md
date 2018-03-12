## Reliable DMA implementation of Modbus (library FreeModbus with STM32, Keil 5)

`#modbus #reliable #high-rate-modbus #serial_DMA`

DMA based free Modbus implementation and port to newest stm32cube software.

This design works on the half-duplex EIA-485 interface.

Modbus has an important role in many industrial applications. In this work, we concentrate on implementation issues that occurred when porting such library to a new microcontroller.

This code has these **benefits**:

1.   porting to a known library
2.   **DMA implementation of serial** reduce the overhead in high data-rate
3.   ***High reliability due to interrupt reduction*** 
4.   using the last version of the STM32 support package for implementation

DMA implementation of such application meets higher baud-rate in the communication link. many known implementation uses interrupt. Interrupt reduction has a big influence on the reliability of code. DMA as a section of hardware reduces interrupt overhead extremely.

One interrupt will be used in last version, that is occurred when a packet is not complete or with a nested packet.   

version 1: in this version DMA just transfer TX data

version 2: (coming soon) RX , TX  with DMA
