/*
 * @file main.c
 * @brief Sistema de control de nivel de agua con bomba y alarma. 
 * 
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>

#define ADC_CHANNEL_WATER_SENSOR ADC_CHANNEL0
#define UMBRAL_NIVEL_AGUA 2000  // Umbral para el nivel crítico del agua
#define RELOAD_COUNT 89999      // Valor de recarga para el contador de SysTick

// Definir valores para frecuencia mínima y máxima
#define FRECUENCIA_MINIMA 1500  // 1.5 kHz
#define FRECUENCIA_MAXIMA 2500  // 2.5 kHz
#define INCREMENTO_FRECUENCIA 100  // Incremento de 100 Hz por segundo

volatile uint32_t nivel_agua = 0;
volatile uint8_t alarma_activa = 0;
volatile uint32_t contador_systick = 0;
volatile uint32_t led_blink_delay = 2000; // 2 segundos por defecto

// Variable global para la frecuencia del buzzer
uint16_t frecuencia_pwm = FRECUENCIA_MINIMA;  // Comienza en 1.5 kHz

char uart_buffer[50]; // Buffer para almacenar el mensaje de UART


/**
 * @brief Inicializa el sistema: configura el reloj a 72 MHz, habilita los relojes
 *        para GPIOC, GPIOA, USART1, ADC1, Timer4 y DMA1.
 */
void systemInit(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Configuración del reloj a 72 MHz
    rcc_periph_clock_enable(RCC_GPIOC);     // Habilitar reloj para GPIOC (LED de estado)
    rcc_periph_clock_enable(RCC_GPIOA);     // Habilitar reloj para GPIOA (Alarma y UART)
    rcc_periph_clock_enable(RCC_USART1);    // Habilitar reloj para USART1
    rcc_periph_clock_enable(RCC_ADC1);      // Habilitar reloj para ADC
    rcc_periph_clock_enable(RCC_TIM2);      // Habilitar reloj para Timer4
    rcc_periph_clock_enable(RCC_DMA1);      // Habilitar reloj para DMA1
    rcc_periph_clock_enable(RCC_TIM3);      // Habilitar reloj para Timer3 (PWM)
}

void configurar_puertos(void) {
    // Configuración de GPIOC para el LED de estado o alarma
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    
    // Configuración de GPIOA para la alarma (buzzer)
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO1);

    // Configurar el GPIO para la bomba de agua
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15); 
}


/**
 * @brief Configura el SysTick para que genere una interrupción cada 1 ms.
 */
void configurar_systick(void) {
    systick_set_reload(RELOAD_COUNT);  // Interrupción cada 1 ms
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_interrupt_enable();
    systick_counter_enable();
}


void configurar_uart(void) {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}


/**
 * @brief Configura el ADC para leer el sensor de nivel de agua
 */
void configurar_adc(void) {
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_TIM2_CC2);
    adc_set_sample_time(ADC1, ADC_CHANNEL_WATER_SENSOR, ADC_SMPR_SMP_28DOT5CYC);    
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibration(ADC1);
}

/**
 * @brief Configura el DMA para transmitir datos por UART.
 */
void configurar_dma_uart(void) {
    dma_channel_reset(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART1_DR); // Dirección del registro de datos de USART1
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)uart_buffer);     // Dirección del buffer de transmisión
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, sizeof(uart_buffer));       // Tamaño del buffer
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_HIGH);
    dma_enable_channel(DMA1, DMA_CHANNEL4);
    
    usart_enable_tx_dma(USART1); // Habilita DMA para UART
}
/**
 * @brief Configura el Timer para generar una interrupción cada 5 segundos.
 */
void configurar_timer(void) {
    // Configurar el prescaler y el periodo para el timer 2
    timer_set_prescaler(TIM2, 7200 - 1); // 72 MHz / 7200 = 10 kHz
    timer_set_period(TIM2, 20000 - 1);   // 10 kHz / 20000 = 0.5 Hz (2 segundos)

    timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_TOGGLE);  // Modo toggle (cambia de estado)
    timer_set_oc_value(TIM2, TIM_OC2, 20000 - 1);      // Valor de comparación a 2 segundos
    timer_enable_oc_output(TIM2, TIM_OC2);             // Habilita el Output Compare para el canal 1
    timer_enable_counter(TIM2);             // Activa el contador del Timer
    
    // Configurar el prescaler y el periodo para el timer 3
    timer_set_prescaler(TIM3, 7200 - 1); // 72 MHz / 7200 = 10 kHz
    timer_set_period(TIM3, 5000 - 1);   // 10 kHz / 5000 = 2 Hz (0.5 segundos)

    timer_enable_irq(TIM3, TIM_DIER_UIE);   // Habilita la interrupción de actualización
    timer_enable_counter(TIM3);             // Activa el contador del Timer
    nvic_enable_irq(NVIC_TIM3_IRQ);         // Habilita la interrupción en el NVIC
}

/**
 * @brief Configura el PWM en el timer TIM3 para el buzzer.
 */
void configurar_pwm(void) {
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_value(TIM3, TIM_OC1, frecuencia_pwm);  // Valor inicial del PWM
    timer_enable_oc_output(TIM3, TIM_OC1);
}

/**
 * @brief Envia el nivel de agua a través de UART usando DMA.
 *
 * @param nivel Nivel de agua a enviar.
 */
void uart_send_level_dma(uint32_t nivel) {
    snprintf(uart_buffer, sizeof(uart_buffer), "Nivel de agua: %lu\r\n", nivel);
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, strlen(uart_buffer));
    dma_enable_channel(DMA1, DMA_CHANNEL4);
}

// ------------------------------------ Funciones de interrupción ------------------------------------



/**
 * @brief Interrupción del Timer3 (1 segundo).
 * Incrementa la frecuencia del PWM si la alarma está activa.
 */
void tim3_isr(void) {
    if (timer_get_flag(TIM3, TIM_SR_UIF)) {
        timer_clear_flag(TIM3, TIM_SR_UIF);

        if (alarma_activa) {
            frecuencia_pwm += INCREMENTO_FRECUENCIA;  // Incrementar la frecuencia cada segundo
            if (frecuencia_pwm >= FRECUENCIA_MAXIMA) {  // Limitar la frecuencia a 2.5 kHz
                frecuencia_pwm = FRECUENCIA_MAXIMA;
            }
            timer_set_oc_value(TIM3, TIM_OC1, frecuencia_pwm);  // Actualizar valor del PWM
        }
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
        adc_clear_flag(ADC1, ADC_SR_EOC);       // Limpia la bandera del ADC
        nivel_agua = adc_read_regular(ADC1);
    
        if (nivel_agua > UMBRAL_NIVEL_AGUA) {
            alarma_activa = 1;
            timer_set_oc_value(TIM3, TIM_OC1, 100); // Activa el PWM del buzzer (asumiendo 100% duty cycle)
            gpio_set(GPIOB, GPIO15); // Activar bomba de agua
        } else {
            alarma_activa = 0;
            timer_set_oc_value(TIM3, TIM_OC1, 0); // Desactiva el PWM del buzzer
            gpio_clear(GPIOB, GPIO15); // Desactivar bomba de agua
        }
    }
    uart_send_level_dma(nivel_agua);   // Enviar nivel de agua por UART usando DMA
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
        led_blink_delay = 500;  // Parpadeo rápido (0.5s) si la alarma está activa
    } else {
        led_blink_delay = 2000; // Parpadeo lento (2s) si no hay alarma
    }

    // Control del parpadeo del LED de estado
    if (contador_systick >= led_blink_delay) {
        gpio_toggle(GPIOC, GPIO13); // Cambia el estado del LED
        contador_systick = 0; // Reiniciar el contador
    }
}

// ------------------------------------ Función principal ------------------------------------

int main(void) {
    systemInit();
    configurar_puertos();
    configurar_systick();
    configurar_dma_uart();
    configurar_timer();
    configurar_adc();
    configurar_uart();
    configurar_pwm();

    while (1) {
        // No hacer nada
    }

    return 0;
}
