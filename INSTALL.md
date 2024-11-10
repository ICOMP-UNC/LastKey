# Guía de Instalación y Configuración

Este documento proporciona una guía detallada para compilar, actualizar y ejecutar el firmware en una placa STM32 utilizando las herramientas necesarias.

## Requisitos previos

### Dependencias necesarias:

1. **GNU Toolchain para ARM**: Necesitarás la herramienta de compilación para ARM, como `arm-none-eabi-gcc`, que incluye los compiladores y enlazadores para el microcontrolador STM32.
   
   - En sistemas basados en Linux, puedes instalarlo con:
     ```bash
     sudo apt-get install gcc-arm-none-eabi
     ```

2. **libopencm3**: Esta es la biblioteca de código abierto para manejar el hardware STM32, que ya estás utilizando en tu proyecto. Debes clonar el repositorio de `libopencm3` y compilarlo.

   Para instalarlo:
   ```bash
   git clone https://github.com/libopencm3/libopencm3.git
   cd libopencm3
   make
