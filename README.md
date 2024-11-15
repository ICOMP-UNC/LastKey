# LastKey
# Resumen del código:
El sistema de control de nivel de agua incluye la activación de una alarma y una bomba cuando el nivel de agua excede un umbral definido. A continuación se detallan los componentes implementados:

ADC: Utilizado para leer periódicamente el nivel de agua.

Timers y SysTick: Controlan el LED de estado y realizan verificaciones periódicas del nivel de agua.

UART: Transmite el nivel de agua a la consola para monitoreo.

DMA: Facilita la transferencia de datos desde el ADC al UART para enviar el nivel de agua sin necesidad de intervención directa del procesador, mejorando la eficiencia.

LED de Estado: Un indicador visual que cambia la frecuencia de parpadeo en función del estado de la alarma.

Alarma y bomba de agua: Se activan automáticamente cuando el nivel de agua supera el umbral definido.


# Requerimientos:
Lectura del sensor de nivel de agua: Se utiliza el ADC para leer el nivel de agua periódicamente. El nivel se compara con un umbral para activar o desactivar la bomba de agua y la alarma.

Control del LED de estado: Mediante SysTick, el LED de estado cambia de frecuencia según el estado de la alarma (rápido si está activa, lento si está desactivada).

Comunicación UART: El nivel de agua leído por el ADC se transmite a la consola a través de UART, utilizando DMA para transferir los datos de manera eficiente.

Control de la alarma y bomba de agua: Si el nivel de agua supera el umbral, se activan tanto la alarma como la bomba. Cuando el nivel de agua desciende por debajo del umbral, se desactivan.

Timers: Un temporizador se utiliza para verificar el nivel de agua cada 5 segundos y gestionar el control de la bomba, activándola o desactivándola según sea necesario.


# Diagrama de Flujo:
![image](https://github.com/user-attachments/assets/c71f6a14-9be3-4130-a8ff-d4e19abba010)
![image](https://github.com/user-attachments/assets/fcf01caf-7593-4f7d-8e1d-322e2c538585)
![image](https://github.com/user-attachments/assets/4ac2a041-2e6c-4246-85ff-6325c36fa42a)
![image](https://github.com/user-attachments/assets/9020fdf5-82ed-4f22-97c8-d752fd37ad43)

