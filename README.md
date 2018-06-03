## Slide Puzzle 4x4
### Microntrolador Blue Pill stm32f103c8t6 + FreeRTOS
#### Projeto em Attolic TrueStudio + CubeMX
#### Hardware
* STM32 + Gravador STLink V2
* Nokia 5110 Display
* Joystick
* Buzzer + Transistor

#### MCU STM32

* Timer PWM - Buzzer
* ADC+DMA - Joystick

#### FreeRTOS
* 2x Queues
* 3x Semaphores
* 1x Timer

Esquema de Interação Tarefas RTOS
![alt text](https://github.com/Eximmius/SlidePuzzle-STM32/blob/master/TaskDiagram.png?raw=true "FreeRTOS Tasks")

