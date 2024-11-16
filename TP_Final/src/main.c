/**
 * @file main.c
 * @brief Sistema de control de nivel de agua con bomba y alarma.
 *
 * Este código implementa un sistema para medir el nivel de agua en un tanque
 * utilizando un sensor de nivel de agua conectado al ADC. Si el nivel de agua
 * supera un umbral crítico, se activa una alarma y la bomba de agua.
 * Además, el nivel de agua se transmite a través de UART.
 *
 * @note Este código utiliza la librería libopencm3 para interactuar con el
 * hardware de STM32.
 */
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/common/adc_common_v1.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include <string.h>

#define ADC_CHANNEL_WATER_SENSOR                                               \
    ADC_CHANNEL0 /**< Canal del ADC para el sensor de nivel de agua */
#define UMBRAL_NIVEL_AGUA 1300 /**< Umbral para el nivel crítico del agua */
#define RELOAD_COUNT                                                           \
    89999 /**< Valor de recarga para el contador de SysTick                    \
           */

/**
 * @brief Definición de las frecuencias mínimas y máximas para la alarma.
 *
 * Se usa para controlar la frecuencia de parpadeo de la alarma y la frecuencia
 * del PWM del buzzer.
 */
#define PERIODO_MINIMO 400 /**< Frecuencia mínima del PWM en Hz (0.5 seg) */
#define PERIODO_MAXIMO 600 /**< Frecuencia máxima del PWM en Hz (1 seg) */
#define INCREMENTO_PERIODO                                                     \
    1 /**< Incremento de frecuencia en Hz por segundo                          \
       */

/**
 * @brief Variables globales para controlar el nivel de agua y la alarma.
 */
volatile uint32_t nivel_agua = 0; /**< Nivel actual de agua */
volatile uint8_t alarma_activa =
    0; /**< Estado de la alarma (1 si está activa) */
volatile uint32_t contador_systick =
    0; /**< Contador para las interrupciones de SysTick */
volatile uint32_t led_blink_delay =
    2000; /**< Retardo para el parpadeo del LED en milisegundos */

/**
 * @brief Variable global para la frecuencia del buzzer (alarma).
 */
uint16_t tim3_period = PERIODO_MINIMO; /**< Periodo inicial del Timer3 para
                                          controlar la frecuencia del buzzer */
uint32_t array[8];
uint32_t indice = 0;
int suma = 0;

/**
 * @brief Buffer para almacenar mensajes que se enviarán por UART.
 */
char uart_buffer[50];

/**
 * @brief Creacion de funciones.
 */
void configurar_puertos(void);
void configurar_systick(void);
void configurar_uart(void);
void configurar_adc(void);
void systemInit(void);
void configurar_dma_uart(void);
void uart_send_level_dma(uint32_t nivel);
void configurar_timer(void);

/**
 * @brief Función que inicializa el sistema.
 *
 * Configura el reloj a 72 MHz y habilita los relojes de los periféricos:
 * GPIO, USART1, ADC1, Timer2, DMA1, Timer3.
 */
void systemInit(void) {
    rcc_clock_setup_pll(
        &rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]); // Configura el reloj del
                                                 // sistema a 72 MHz
    rcc_periph_clock_enable(
        RCC_GPIOC); // Habilita el reloj para GPIOC (LED de estado)
    rcc_periph_clock_enable(
        RCC_GPIOA); // Habilita el reloj para GPIOA (Alarma y UART)
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1); // Habilita el reloj para USART1
    rcc_periph_clock_enable(RCC_ADC1);   // Habilita el reloj para ADC1
    rcc_periph_clock_enable(RCC_TIM2);   // Habilita el reloj para Timer2
    rcc_periph_clock_enable(RCC_DMA1);   // Habilita el reloj para DMA1
    rcc_periph_clock_enable(RCC_TIM3);   // Habilita el reloj para Timer3 (PWM)
}

/**
 * @brief Configura los pines GPIO para el LED de estado, el buzzer (alarma),
 *        la bomba de agua, el UART y el sensor de nivel de agua.
 */
void configurar_puertos(void) {
    // Configuración de GPIOC para el LED de estado (PC13)
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO13);
    gpio_set(GPIOC, GPIO13); // Pone el pin 13 de GPIOC en bajo

    // Configuración de GPIOA para la alarma (buzzer) en PA1
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO1);

    // Configuración de GPIOB para la bomba de agua en PB15
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO12);

    // Configuración de PA9 (TX) para UART1
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_USART1_TX); // UART TX (PA9)

    // Configuración de PA0 para el sensor de nivel de agua (entrada ADC)
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0);
}

/**
 * @brief Configura el SysTick para que genere una interrupción cada 1 ms.
 */
void configurar_systick(void) {
    systick_set_reload(RELOAD_COUNT); // Interrupción cada 1 ms
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_interrupt_enable();
    systick_counter_enable();
}

/**
 * @brief Inicializa la UART1 con una velocidad de transmisión de 115200 bps.
 */
void configurar_uart(void) {
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX); // Solo transmisión en este caso
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable_tx_dma(USART1); // Habilita DMA para UART
    usart_enable(USART1);
}

/**
 * @brief Configura el ADC para leer el sensor de nivel de agua.
 */
void configurar_adc(void) {
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_TIM2_CC2);
    adc_set_sample_time(ADC1, ADC_CHANNEL_WATER_SENSOR, ADC_SMPR_SMP_28DOT5CYC);
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
    adc_enable_eoc_interrupt(ADC1);
    nvic_enable_irq(NVIC_ADC1_2_IRQ);
}

/**
 * @brief Configura el DMA para transmitir datos por UART.
 */
void configurar_dma_uart(void) {
    dma_channel_reset(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_address(
        DMA1, DMA_CHANNEL4,
        (uint32_t)&USART1_DR); // Dirección del registro de datos de USART1
    dma_set_memory_address(
        DMA1, DMA_CHANNEL4,
        (uint32_t)uart_buffer); // Dirección del buffer de transmisión
    dma_set_number_of_data(DMA1, DMA_CHANNEL4,
                           sizeof(uart_buffer)); // Tamaño del buffer
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    dma_enable_channel(DMA1, DMA_CHANNEL4);
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4,
                              DMA_TCIF | DMA_HTIF | DMA_GIF | DMA_TEIF);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);

    usart_enable_tx_dma(USART1); // Habilita DMA para UART
}

/**
 * @brief Configura el Timer para generar una interrupción cada 5 segundos.
 */
void configurar_timer(void) {
    // Configurar el prescaler y el periodo para el timer 2
    timer_set_prescaler(TIM2, 7200 - 1); // 72 MHz / 7200 = 10 kHz
    timer_set_period(TIM2, 20000 - 1);   // 10 kHz / 20000 = 0.5 Hz (2 segundos)
    timer_set_oc_mode(TIM2, TIM_OC2,
                      TIM_OCM_TOGGLE); // Modo toggle (cambia de estado)
    timer_set_oc_value(TIM2, TIM_OC2,
                       20000 - 1); // Valor de comparación a 2 segundos
    timer_enable_oc_output(
        TIM2, TIM_OC2);         // Habilita el Output Compare para el canal 2
    timer_enable_counter(TIM2); // Activa el contador del Timer

    // Configurar el prescaler y el periodo para el timer 3 (PWM)
    timer_set_prescaler(TIM3, 72 - 1); // 72 MHz / 72 = 10000 kHz
    timer_set_period(TIM3,
                     PERIODO_MINIMO - 1); // 1000 kHz / 600 = 1.6K Hz (100% PWM)
    timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC3);
    timer_set_oc_value(TIM3, TIM_OC3,
                       PERIODO_MINIMO / 2 -
                           1); // Valor de comparación a 2 segundos
    timer_enable_irq(TIM3, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM3_IRQ);
}
/**
 * @brief Envia el nivel de agua a través de UART usando DMA.
 *
 * @param nivel Nivel de agua a enviar.
 */
void uart_send_level_dma(uint32_t nivel) {
    dma_disable_channel(DMA1, DMA_CHANNEL4);
    snprintf(uart_buffer, sizeof(uart_buffer), "Nivel de agua: %lu\r\n", nivel);
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, strlen(uart_buffer));
    dma_enable_channel(DMA1, DMA_CHANNEL4);
}

// ------------------------------------ Funciones de interrupción
// ------------------------------------

void dma1_channel4_isr(void) {
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4,
                              DMA_TCIF | DMA_HTIF | DMA_GIF | DMA_TEIF);
}

/**
 * @brief Interrupción del Timer3 (1 segundo).
 * Incrementa la frecuencia del PWM si la alarma está activa.
 */
void tim3_isr(void) {
    timer_clear_flag(TIM3, TIM_SR_UIF);
    static uint16_t cont = 0;
    static int16_t incremento = INCREMENTO_PERIODO;
    cont++;
    if (cont == 10) {
        tim3_period += incremento;

        // Cambiar dirección si se alcanza el límite
        if (tim3_period >= PERIODO_MAXIMO || tim3_period <= PERIODO_MINIMO) {
            incremento = -incremento; // Invertir dirección
        }

        // Actualizar periodo y ciclo de trabajo del PWM
        timer_set_period(TIM3, tim3_period);
        timer_set_oc_value(TIM3, TIM_OC3, tim3_period / 2 - 1);

        // Reiniciar contador
        cont = 0;
    }
}

/**
 * @brief Handler de interrupción del ADC1.
 *
 * Verifica si la conversión ha finalizado, lee el valor del ADC y
 * compara el nivel de agua con el umbral. Si el nivel de agua es mayor
 * que el umbral, activa la alarma y la bomba. De lo contrario, desactiva
 * la alarma y la bomba.
 */
void adc1_2_isr(void) {
    if (adc_eoc(ADC1)) {
        adc_clear_flag(ADC1, ADC_SR_EOC); // Limpia la bandera del ADC
        nivel_agua = adc_read_regular(ADC1);
        if (nivel_agua > UMBRAL_NIVEL_AGUA) {
            alarma_activa = 1;
            timer_enable_counter(TIM3); // Activa el contador del Timer 3
            gpio_set(GPIOB, GPIO12);    // Activar bomba de agua GPIO15
        } else {
            alarma_activa = 0;
            timer_disable_counter(TIM3); // Desctiva el contador del Timer 3
            timer_set_counter(TIM3,
                              0); // Establece el contador del temporizador a 0
            tim3_period = PERIODO_MINIMO; // Frecuencia vuelve a su valor minimo
            gpio_clear(GPIOB, GPIO12);    // Desactivar bomba de agua GPIO15
        }
    }
    uart_send_level_dma(nivel_agua); // Enviar nivel de agua por UART usando DMA
}

/**
 * @brief Manejador de interrupciones de SysTick.
 *
 * Incrementa el contador de SysTick y ajusta la frecuencia de parpadeo
 * del LED en función del estado de la alarma. Si la alarma está activa,
 * el LED parpadea rápidamente; de lo contrario, parpadea lentamente.
 * Al alcanzar el retraso de parpadeo configurado, el estado del LED se
 * alterna y el contador se reinicia.
 */
void sys_tick_handler(void) {
    contador_systick++;

    // Ajustar la frecuencia de parpadeo en función del estado de la alarma
    if (alarma_activa) {
        led_blink_delay =
            500; // Parpadeo rápido (0.5s) si la alarma está activa
    } else {
        led_blink_delay = 2000; // Parpadeo lento (2s) si no hay alarma
    }

    // Control del parpadeo del LED de estado
    if (contador_systick >= led_blink_delay) {
        gpio_toggle(GPIOC, GPIO13); // Cambia el estado del LED
        contador_systick = 0;       // Reiniciar el contador
    }
}

// ------------------------------------ Función principal
// ------------------------------------

/**
 * @brief Inicializa todo el hardware y entra en un bucle infinito.
 */
int main(void) {
    systemInit();
    configurar_puertos();
    configurar_systick();
    configurar_timer();
    configurar_adc();
    configurar_uart();
    configurar_dma_uart();
    while (1) {
        // No hacer nada
        __asm__("wfi");
    }

    return 0;
}