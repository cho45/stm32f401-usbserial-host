USB CDC to UART proxy with STM32F401
====================================

This STM32F401 works as USB CDC Host Device, and accepts CDC device and proxy the communications to raw UART port.

## Build

require `arm-none-eabi-gcc`

```
make
```

## Setting

CDC device serial connection is proxied to USART2:

 * PA0: CTS
 * PA1: RTS
 * PA2: TX
 * PA3: RX

(hardware flow control is enabledt)
