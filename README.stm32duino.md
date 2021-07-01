# MySensors Library readme for [stm32duino](https://github.com/stm32duino/Arduino_Core_STM32)

## Why start this since there is STM32 support of [rogerclarkmelbourne/Arduino_STM32](https://github.com/rogerclarkmelbourne/Arduino_STM32)
- It lacks low power support, which is important for battery powered nodes.
- Rich features of STM32 are limited, such as STM32 has more interrupt than AVR.

## Task status
- [x] STM32F1 basic support
- [ ] Low Power support
- [ ] Extra Interrupt support

## Development Memo
Tested on:
- Hardware: STM32F103C8T6 Bluepill 128KB
  - configuration

    <img src="./Documentation/img/configuration-stm32duino-stm32f1c8t6.png" alt="configuration of stm32f1c8t6 bluepill" width="500"/>

- STM32 Cores by STMicroelectronics Version 1.9.0

# Thanks
- [#Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32) for the rich support of STM32 series.
- [Arduino_STM32](https://github.com/rogerclarkmelbourne/Arduino_STM32) for the first STM32 support on STM32.
- [NodeManager](https://github.com/mysensors/NodeManager) for the fast development on mysensors gateways/nodes.
- [MySensors](https://github.com/mysensors/MySensors) for building the base.
