# LastKey

## Descripcion General del Sistema:
Este proyecto implementa un sistema de control de nivel de agua que tiene como objetivo automatizar el manejo de una bomba y activar una alarma si el nivel de agua excede un umbral definido. Para ello, utilizamos diversas tecnologías como ADC, timers, UART, DMA y el uso de un LED indicador de estado. Además, se integra el manejo eficiente de datos y control de hardware mediante interrupciones y funciones periféricas del microcontrolador STM32.


## Resumen del Código:
A continuación se detallan los componentes implementados:

- **ADC**: Utilizado para leer periódicamente el nivel de agua. Configurado para iniciar la conversión por medio del Timer2, logrando sincronización con las interrupciones.

- **Timers y SysTick**:
  - **Timer2**: Configurado para generar una interrupción cada 5 segundos, iniciando una nueva conversión del ADC para monitorear el nivel de agua.
  - **Timer3**: Configurado como generador de PWM para controlar la frecuencia de la alarma (buzzer pasivo).
  - **SysTick**: Controla el LED de estado que indica el estado del sistema (alarma activa/inactiva) mediante diferentes frecuencias de parpadeo.

- **UART y DMA**: 
  - UART: Utilizado para transmitir el nivel de agua leído por el ADC a una consola de monitoreo.
  - DMA: Facilita la transferencia eficiente de datos desde el ADC al UART, reduciendo la carga del procesador.

- **LED de Estado**: Actúa como indicador visual. Parpadea a diferentes frecuencias dependiendo del estado del sistema:
  - Parpadeo rápido: alarma activada.
  - Parpadeo lento: alarma desactivada.

- **Alarma y Bomba de Agua**: Se activan automáticamente cuando el nivel de agua excede un umbral definido. Permanecen activos hasta que el nivel vuelva a estar dentro del rango seguro.


## Requerimientos:
1. **Lectura del sensor de nivel de agua**:
   - Configuración del ADC para convertir las señales del sensor a valores digitales.
   - La lectura del sensor se realiza periódicamente (cada 5 segundos) mediante interrupciones generadas por Timer2.

2. **Control del LED de estado**:
   - Mediante SysTick, el LED cambia su frecuencia de parpadeo dependiendo del estado del sistema:
     - Parpadeo rápido: nivel de agua crítico (alarma activada).
     - Parpadeo lento: nivel de agua normal.

3. **Comunicación UART**:
   - El nivel de agua se transmite a la consola para monitoreo en tiempo real.
   - La transmisión de datos se optimiza utilizando DMA, que transfiere automáticamente los datos desde el buffer del ADC al UART.

4. **Control de la alarma y bomba de agua**:
   - Activación automática al superar un umbral de nivel de agua predefinido.
   - Desactivación cuando el nivel de agua regresa a un rango seguro.

5. **Timers**:
   - **Timer2**: Genera interrupciones cada 5 segundos para iniciar una nueva conversión del ADC.
   - **Timer3**: Utilizado para generar una señal PWM que controla la frecuencia de la alarma.


## Diagrama de Flujo:
![image](https://github.com/user-attachments/assets/c71f6a14-9be3-4130-a8ff-d4e19abba010)
![image](https://github.com/user-attachments/assets/75bb9de7-ba86-4472-89c4-210bcd201e9e)
![image](https://github.com/user-attachments/assets/4ac2a041-2e6c-4246-85ff-6325c36fa42a)
![image](https://github.com/user-attachments/assets/9020fdf5-82ed-4f22-97c8-d752fd37ad43)

