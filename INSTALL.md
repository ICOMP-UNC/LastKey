# Guía de Instalación y Configuración

Esta guía proporciona pasos detallados para compilar, instalar y ejecutar el firmware en un sistema basado en el microcontrolador STM32F103C8T6, específicamente para un sistema de control de nivel de agua. La guía incluye instrucciones sobre herramientas necesarias, dependencias y la configuración del entorno de desarrollo en Windows.

# Requisitos previos

## Hardware
Placa STM32F103C8T6

ST-LINK V2 (Para la conexión de la placa con la PC)

Sensor de nivel de agua

Buzzer pasivo

Bomba de agua

LED en PC13

Cables y fuentes de alimentación según el esquema de conexión

## Software
Visual Studio Code (con la extensión PlatformIO).

ST-LINK Utility (para la detección y configuración del ST-LINK).

OpenOCD (para la depuración).

# Instalación y Configuración del Entorno de Desarrollo
1. Instalar Visual Studio Code y PlatformIO: 
Descargar e instala Visual Studio Code.
Abrir Visual Studio Code, buscar e instalra la extensión PlatformIO.

3. Instalar ST-LINK Utility: 
Descargar e instalar ST-LINK Utility desde STMicroelectronics.
Conectar el ST-LINK V2 al PC y verificar que sea detectado por la aplicación.

4. Instalar y Configurar OpenOCD: 
Instalar OpenOCD como parte del entorno de PlatformIO.
En PlatformIO, crear un nuevo proyecto:
Seleccionar la placa: STM32F103C8T6 (BluePill).
Framework: libopencm3 (según el código desarrollado).

# Compilación del Código
1. Abrir Visual Studio Code y cargar el proyecto desde PlatformIO.

2. Configurar dependencias en el archivo platformio.ini:

    ```ini
    [env:bluepill_f103c8]
    
    platform = ststm32
    
    board = bluepill_f103c8
    
    framework = libopencm3
    
    ; change microcontroller
    
    board_build.mcu = stm32f103c8t6
    
    ; change MCU frequency
    
    board_build.f_cpu = 72000000L
    
    build_flags = 
    
        -O0    ; Desactiva optimización
        
        -g     ; Incluye información de depuración
    
    extra_scripts = pre:unlock_flash.py
    
    ; Configuración para el monitor serial
    
    monitor_speed = 9600
    
    monitor_port = COM4
    ```
4. Compilar el proyecto utilizando el comando: PlatformIO: Build

# Instalación del Firmware en el Microcontrolador
1. Conectar la placa STM32F103C8T6 al PC utilizando el ST-LINK V2.

2. Desde PlatformIO, seleccionar la opción: PlatformIO: Upload
