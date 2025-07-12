/*
 * commands.c
 *
 *  Created on: Jul 4, 2025
 *      Author: luisduquefranco
 */


#include "main.h"
// === Prototipos locales ===
static float calculate_sampling_rate(void);

// === Implementación de comandos ===

void Cmd_HandleLEDDelayCmd(const char *arg) {
    uint32_t nuevo = atoi(arg);
    __HAL_TIM_SET_AUTORELOAD(&htim2, nuevo);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    int len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                       "LED delay = %lu ms\r\n", (unsigned long)nuevo);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

void Cmd_HandleSampleFreqCmd(const char *arg) {
    uint32_t fs;
    char option = arg[0];
    switch(option) {
        case '1': fs = 44100; break;
        case '2': fs = 48000; break;
        case '3': fs = 96000; break;
        case '4': fs = 128000; break;
        default: {
            const char *msg = "Opciones válidas para 'fmuestreo=' son: 1, 2, 3, 4\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
            return;
        }
    }
    uint32_t arr = timer_clk / fs - 1;
    uint32_t psc = 0;
    if (arr > 0xFFFF) {
        psc = (arr / 0x10000) + 1;
        arr = (timer_clk / (psc+1) / fs) - 1;
    }

    __HAL_TIM_SET_PRESCALER(&htim3, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Stop(&htim3);
    HAL_TIM_Base_Start(&htim3);

    int len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                       "Sample TIM3 @ %lu Hz (PSC=%lu, ARR=%lu)\r\n",
                       (unsigned long)fs, (unsigned long)psc, (unsigned long)arr);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

void Cmd_HandleRGBCmd(const char *arg) {
    GPIO_PinState R = (strchr(arg,'R') ? GPIO_PIN_SET : GPIO_PIN_RESET);
    GPIO_PinState G = (strchr(arg,'G') ? GPIO_PIN_SET : GPIO_PIN_RESET);
    GPIO_PinState B = (strchr(arg,'B') ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, R);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, G);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, B);
    int len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                       "RGB -> R:%c G:%c B:%c\r\n",
                       R==GPIO_PIN_SET?'1':'0',
                       G==GPIO_PIN_SET?'1':'0',
                       B==GPIO_PIN_SET?'1':'0');
    HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

void Cmd_HandleFFTSizeCmd(const char *arg) {
    char option = arg[0];
    if (option == '1') {
        fft_size = 1024;
    } else if (option == '2') {
        fft_size = 2048;
    } else {
        const char *msg = "Opciones válidas para 'fftSize=' son:\r\n"
                          "1 -> 1024 puntos\r\n"
                          "2 -> 2048 puntos\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
        return;
    }

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, fft_size);

    int len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                       "FFT size set to %u\r\n", fft_size );
    HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

void Cmd_HandleStatusCmd(void) {
    float fs = calculate_sampling_rate();
    float bin_res = fs / fft_size;
    int len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
        "Config:\r\nSample TIM3 @ %.2f Hz (PSC=%lu, ARR=%lu)\r\nFFT size: %u\r\nResolucion espectral: %.2f Hz/bin\r\nCanal ADC: 6\r\nTrigger ADC: TIM3_TRGO\r\n",
        fs, (unsigned long)htim3.Init.Prescaler, (unsigned long)htim3.Init.Period, fft_size, bin_res);
    HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

void Cmd_HandlePrintADC(void) {
    char msg[32];
    for (int i = 0; i < fft_size; i++) {
        float voltage = adc_buffer[i] * (3.3f / 4095.0f);
        int len = snprintf(msg, sizeof(msg), "%.4f\n", voltage);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
    }
}

void Cmd_HandleFreqDisplayCmd(void) {
    char msg[64];
    float suma = 0.0f;
    int count = freq_full ? FREQ_BUFFER_SIZE : freq_index;

    HAL_UART_Transmit(&huart2, (uint8_t*)"Frecuencias IC (Hz):\r\n", 24, 100);
    for (int i = 0; i < count; i++) {
        int idx = (freq_index + i) % FREQ_BUFFER_SIZE;
        suma += freq_buffer[idx];
        int len = snprintf(msg, sizeof(msg), "%.2f\r\n", freq_buffer[idx]);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
    }

    float promedio = (count > 0) ? suma / count : 0.0f;
    int len = snprintf(msg, sizeof(msg), "Promedio: %.2f Hz\r\n", promedio);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
}

void Cmd_HandleClearCmd(void) {
    const char *clear_screen = "\033[2J\033[H";
    HAL_UART_Transmit(&huart2, (uint8_t*)clear_screen, strlen(clear_screen), 1000);
}

void Cmd_HandleHelpCmd(void) {
    const char *help_msg =
        "\r\n========== AYUDA =========="
        "\r\nComandos disponibles:\r\n"
        "  led=<ms>@         - Cambia la frecuencia del LED Blinky\r\n"
        "  fmuestreo=<1|2|3|4>@ - Frecuencia de muestreo del ADC\r\n"
        "  rgb=<RGB>@        - Control de LED RGB, ej: rgb=RG\r\n"
        "  fftSize=<1|2>@    - Tamaño FFT: 1->1024, 2->2048\r\n"
        "  status@           - Mostrar configuración actual\r\n"
        "  print@            - Imprimir datos del ADC\r\n"
        "  freq@             - Historial de frecuencia IC\r\n"
        "  clear@            - Limpiar terminal\r\n"
        "  help@             - Mostrar esta ayuda\r\n"
        "===========================\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)help_msg, strlen(help_msg), 1000);
}

static float calculate_sampling_rate(void) {
    return 84000000.0f / ((htim3.Init.Prescaler + 1) * (htim3.Init.Period + 1));
}

void Cmd_HandleFFTInfo(void) {
    float input_f32[FFT_SIZE_MAX];
    float output_fft[FFT_SIZE_MAX];
    float mag_fft[FFT_SIZE_MAX / 2];

    // Convertir datos ADC a float y eliminar offset DC aproximado (2048)
    for (int i = 0; i < fft_size; i++) {
        input_f32[i] = (float)adc_buffer[i] - 2048.0f;
    }

    // Configurar FFT
    arm_rfft_fast_instance_f32 S;
    arm_rfft_fast_init_f32(&S, fft_size);
    arm_rfft_fast_f32(&S, input_f32, output_fft, 0);

    // Calcular magnitud de cada bin (espectro de magnitudes)
    arm_cmplx_mag_f32(output_fft, mag_fft, fft_size / 2);

    // Buscar bin con mayor magnitud (ignorando el bin 0 = DC)
    uint32_t max_index = 0;
    float max_val = 0.0f;
    arm_max_f32(&mag_fft[1], (fft_size / 2) - 1, &max_val, &max_index);
    max_index += 1;

    // Calcular frecuencia muestreo y frecuencia del bin dominante
    float fs = 84000000.0f / ((htim3.Init.Prescaler + 1) * (htim3.Init.Period + 1));
    float freq_bin = fs / fft_size;
    float freq_detected = freq_bin * max_index;

    // Calcular offset y RMS
    float offset = 0.0f, rms = 0.0f;
    arm_mean_f32((float32_t*)adc_buffer, fft_size, &offset);

    for (int i = 0; i < fft_size; i++) {
        input_f32[i] = (float)adc_buffer[i] - offset;
    }
    arm_rms_f32(input_f32, fft_size, &rms);

    // Convertir magnitud a decibelios
    float db_val = 20.0f * log10f(max_val + 1e-6f);

    // Mostrar resultado por UART
    char msg[160];
    int len = snprintf(msg, sizeof(msg),
        "Info de FFT:\r\n"
        "Frecuencia dominante: %.2f Hz\r\n"
        "Magnitud: %.2f dB\r\n"
        "Offset: %.2f niveles ADC\r\n"
        "RMS: %.2f niveles ADC\r\n",
        freq_detected, db_val, offset, rms);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
}

void Cmd_HandlePrintFFT(void) {
    float input_f32[FFT_SIZE_MAX];
    float output_fft[FFT_SIZE_MAX];

    // Paso 1: Convertir datos del ADC a float y remover offset DC (2048)
    for (int i = 0; i < fft_size; i++) {
        input_f32[i] = (float)adc_buffer[i] - 2048.0f;
    }

    // Paso 2: Inicializar la FFT rápida
    arm_rfft_fast_instance_f32 S;
    if (arm_rfft_fast_init_f32(&S, fft_size) != ARM_MATH_SUCCESS) {
        const char *err = "Error al inicializar FFT\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), 100);
        return;
    }

    // Paso 3: Ejecutar la FFT real
    arm_rfft_fast_f32(&S, input_f32, output_fft, 0);

    // Paso 4: Calcular frecuencia de muestreo efectiva (basado en TIM3)
    float fs = 84000000.0f / ((htim3.Init.Prescaler + 1) * (htim3.Init.Period + 1));
    float bin_res = fs / fft_size;

    // Paso 5: Enviar encabezado al terminal (CoolTerm/Python)
    HAL_UART_Transmit(&huart2, (uint8_t*)"FFT_DATA_START\n", 15, 100);

    // Paso 6: Calcular magnitudes y transmitir como CSV (frecuencia, magnitud)
    for (int i = 1; i < fft_size / 2; i++) {
        float real = output_fft[2 * i];
        float imag = output_fft[2 * i + 1];
        float mag = sqrtf(real * real + imag * imag) / (fft_size / 2);  // Normalizar
        float positive_mag = fabsf(mag); // para evitar valores negativos

        char msg[32];
        int len = snprintf(msg, sizeof(msg), "%.1f,%.4f\n", i * bin_res, positive_mag);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
    }
}
