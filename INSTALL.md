# Guía de Instalación y Configuración

Esta guía proporciona pasos detallados para compilar, instalar y ejecutar el firmware en un sistema basado en el microcontrolador STM32F103C8T6, específicamente para un sistema de control de nivel de agua. La guía incluye instrucciones sobre herramientas necesarias, dependencias y la configuración del entorno de desarrollo en Windows.

## Requisitos previos

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

## Instalación y Configuración del Entorno de Desarrollo
1. Instalar Visual Studio Code y PlatformIO: 
Descarga e instala Visual Studio Code desde code.visualstudio.com.
Abre Visual Studio Code y ve al Marketplace.
Busca e instala la extensión PlatformIO.

3. Instalar ST-LINK Utility: 
Descarga e instala ST-LINK Utility desde STMicroelectronics.
Conecta el ST-LINK V2 al PC y verifica que sea detectado por la aplicación.

4. Instalar y Configurar OpenOCD: 
Instala OpenOCD como parte del entorno de PlatformIO.
En PlatformIO, crea un nuevo proyecto:
Selecciona la placa: STM32F103C8T6 (BluePill).
Framework: STM32Cube o Arduino (según el código desarrollado).
