/* ============================================================================
 * Detector de Anomalias Acústicas — ESP32 + INMP441 + FreeRTOS
 * ----------------------------------------------------------------------------
 * Aplicação escolhida: comando de voz como "anomalia acústica de interesse".
 * O sistema escuta o microfone continuamente e reconhece dois padrões
 * acústicos específicos em meio ao ruído de fundo ("unknown"):
 *
 *      "acende"  -> liga o LED (GPIO 17)
 *      "apaga"   -> desliga o LED (GPIO 17)
 *
 * Justificativa prática: controle de iluminação por voz hands-free é um caso
 * real de "spotting" de palavra-chave (keyword spotting) embarcado, a mesma
 * classe de problema usada em campainhas inteligentes, assistentes de voz de
 * baixo consumo e sistemas de acessibilidade — aqui tratado como detecção de
 * um padrão acústico específico (anomalia) em meio ao áudio ambiente comum.
 *
 * Modelo: rede treinada no Edge Impulse (MFCC + rede neural), classes:
 *      ei_classifier_inferencing_categories = { "acende", "apaga", "unknown" }
 * (ver src/model-parameters/model_variables.h da lib gerada pelo EI)
 *
 * ----------------------------------------------------------------------------
 * ARQUITETURA RTOS (FreeRTOS / ESP32) — 3 tasks concorrentes
 * ----------------------------------------------------------------------------
 *
 *  [Task 1: taskCaptureAudio]      ALTA prioridade
 *      - Lê o INMP441 via I2S continuamente (DMA)
 *      - Preenche um double-buffer (ping-pong) circular
 *      - Ao completar um buffer, entrega o índice via fila (xCaptureQueue)
 *      - Antes de reutilizar um buffer, aguarda ele ser liberado pela
 *        Task 3 (semáforo xBufferFree[i]) -> evita overwrite/race condition
 *
 *  [Task 2: taskFeatureExtraction] MÉDIA prioridade
 *      - Recebe o índice do buffer cheio (xCaptureQueue)
 *      - Calcula RMS (energia) e um estimador leve de Centróide Espectral
 *        (via taxa de cruzamento por zero, ZCR) -> usados como "gate" de
 *        silêncio/ruído e para o relatório de latência
 *      - Repassa o pacote de features + índice do buffer para a Task 3
 *        através da fila xFeatureQueue
 *      (Observação de projeto: a extração dos MFCCs propriamente ditos e a
 *       inferência da rede neural são feitas de forma acoplada dentro da
 *       biblioteca do Edge Impulse chamada pela Task 3 — run_classifier_
 *       continuous() — pois o SDK gerado não expõe as duas etapas
 *       separadamente para modelos de áudio contínuo. Isso é documentado
 *       no relatório técnico como uma decisão de arquitetura.)
 *
 *  [Task 3: taskAnomalyDetection]  BAIXA prioridade
 *      - Recebe o pacote de features (xFeatureQueue)
 *      - Roda o modelo pré-treinado (MFCC + rede neural) sobre o buffer
 *        de áudio indicado
 *      - Usa o suavizador oficial do SDK (ei_classifier_smooth_update),
 *        que exige consenso entre várias fatias seguidas antes de aceitar
 *        um rótulo — evita que uma fatia de transição/ruído dispare a
 *        ação errada no meio da palavra (o "pisca")
 *      - Ação por MUDANÇA DE ESTADO (edge-trigger), não por leitura isolada:
 *           "acende" (e LED ainda apagado) -> LED ON e permanece ligado
 *           "apaga"  (e LED ainda aceso)   -> LED OFF e permanece desligado
 *        (com buzzer de confirmação a cada transição)
 *      - Libera o buffer de volta para a Task 1 (xBufferFree[i])
 *      - Atualiza estatísticas e mede a latência ponta-a-ponta
 *
 * ----------------------------------------------------------------------------
 * MECANISMOS DE SINCRONIZAÇÃO (concorrência) usados
 * ----------------------------------------------------------------------------
 *   - 2x Semáforos binários  xBufferFree[0], xBufferFree[1]
 *        -> controlam a posse dos buffers de áudio (produtor/consumidor com
 *           backpressure): a Task 1 só sobrescreve um buffer depois que a
 *           Task 3 avisa que já terminou de processá-lo. Resolve a condição
 *           de corrida clássica do double-buffer de áudio.
 *   - Fila xCaptureQueue   (Task 1 -> Task 2): índice do buffer pronto
 *   - Fila xFeatureQueue   (Task 2 -> Task 3): pacote de features + índice
 *   - Mutex xStateMutex    : protege a struct system_state_t (estado global:
 *                            último label, confiança, contadores)
 *   - Mutex xHardwareMutex : protege o acesso ao LED/buzzer (seção crítica
 *                            de hardware, mesmo havendo hoje um único
 *                            escritor, previne condição de corrida se novas
 *                            tasks de alerta forem adicionadas)
 *   - Mutex xPrintMutex    : serializa o Serial.print entre tasks, evitando
 *                            intercalamento de mensagens (log ilegível)
 *
 * ----------------------------------------------------------------------------
 * PINAGEM (conforme especificado)
 * ----------------------------------------------------------------------------
 *   LED  (saída digital) ......... GPIO 17
 *   Buzzer (opcional) ............ GPIO 4   (troque/remova se não usar)
 *   INMP441 WS  (LRCLK) ........... GPIO 25
 *   INMP441 SCK (BCLK)  ........... GPIO 32
 *   INMP441 SD  (DOUT)  ........... GPIO 33
 *   INMP441 L/R  .................. GND (canal ESQUERDO) -> ver AUDIO_CHANNEL
 *   INMP441 VDD  .................. 3V3
 *   INMP441 GND  .................. GND
 * ============================================================================
 */

/* ---- Biblioteca gerada pelo Edge Impulse (instalar via .ZIP no Arduino IDE)
 * Sketch > Include Library > Add .ZIP Library... > selecione o .zip exportado
 */
#include <lu.petenazzi-project-1_inferencing.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2s.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"   // ets_delay_us
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Este .ino requer um modelo de áudio (microfone) exportado do Edge Impulse."
#endif

/* =============================== PINOS ================================== */
#define PIN_LED          17
#define PIN_BUZZER       4          // opcional: comente USE_BUZZER para remover
#define USE_BUZZER       1

#define I2S_SCK_PIN      32         // BCLK
#define I2S_WS_PIN       25         // LRCLK / WS
#define I2S_SD_PIN       33         // DOUT do INMP441 -> entrada do ESP32
#define I2S_PORT         I2S_NUM_1

// Muitos módulos INMP441 usam L/R->GND = canal ESQUERDO.
// Se o seu módulo tiver L/R->VDD (canal direito), troque para I2S_CHANNEL_FMT_ONLY_RIGHT.
#define AUDIO_CHANNEL_FORMAT   I2S_CHANNEL_FMT_ONLY_LEFT

/* =========================== PARÂMETROS DE ÁUDIO ========================= */
#define AUDIO_GAIN               8      // ganho digital (INMP441 tem saída baixa)
#define I2S_DMA_READ_SAMPLES     512    // amostras lidas por chamada de i2s_read
#define SILENCE_RMS_THRESHOLD    250.0f // abaixo disso, Task 2 marca como silêncio
#define ACTION_COOLDOWN_MS       800    // tempo mínimo entre duas ações de LED

/* =========================== PRIORIDADES DAS TASKS ======================= */
#define PRIORITY_CAPTURE      (tskIDLE_PRIORITY + 4)   // ALTA
#define PRIORITY_FEATURE      (tskIDLE_PRIORITY + 3)   // MÉDIA
#define PRIORITY_DETECTION    (tskIDLE_PRIORITY + 2)   // BAIXA

#define STACK_CAPTURE   (1024 * 8)
#define STACK_FEATURE   (1024 * 4)
#define STACK_DETECTION (1024 * 32)   // precisa acomodar o arena do modelo TFLite

/* ============================ ESTRUTURAS ================================= */

// Double buffer (ping-pong) preenchido pela Task 1
typedef struct {
    int16_t  *buffers[2];
    uint8_t   buf_select;   // buffer sendo escrito agora
    uint32_t  buf_count;
    uint32_t  n_samples;    // = EI_CLASSIFIER_SLICE_SIZE
} inference_t;

// Pacote entregue por Task 2 -> Task 3
typedef struct {
    uint8_t  buffer_index;
    float    rms;
    float    spectral_centroid_hz;   // estimado via ZCR
    bool     is_silence;
    int64_t  t_capture_done_us;
    int64_t  t_feature_done_us;
} feature_packet_t;

// Estado global compartilhado (protegido por xStateMutex)
typedef struct {
    char     last_label[16];
    float    last_confidence;
    bool     led_on;
    uint32_t total_slices_processed;
    uint32_t total_commands_detected;
    uint32_t last_action_ms;
} system_state_t;

/* ============================= GLOBAIS RTOS =============================== */
static inference_t inference;
static int16_t     i2sReadBuffer[I2S_DMA_READ_SAMPLES];
static volatile bool record_status = true;

static SemaphoreHandle_t xBufferFree[2];   // "posse" de cada buffer (bin. semaphores)
static SemaphoreHandle_t xStateMutex;
static SemaphoreHandle_t xHardwareMutex;
static SemaphoreHandle_t xPrintMutex;

static QueueHandle_t xCaptureQueue;   // Task1 -> Task2 : uint8_t (índice do buffer)
static QueueHandle_t xFeatureQueue;   // Task2 -> Task3 : feature_packet_t

static system_state_t g_state = { "unknown", 0.0f, false, 0, 0, 0 };

// Suavizador oficial do SDK Edge Impulse (edge-impulse-sdk/classifier/ei_classifier_smooth.h,
// já incluído por lu.petenazzi-project-1_inferencing.h). Evita que uma única fatia (slice de
// ~250ms) ruidosa/de transição dispare uma ação errada no meio da pronúncia da palavra —
// exige consenso de várias leituras seguidas antes de aceitar um rótulo como válido.
static ei_classifier_smooth_t g_smooth;

/* =============================== UTILITÁRIOS ============================= */

// Serial.print thread-safe (evita mensagens intercaladas entre as 3 tasks)
static void safePrintf(const char *fmt, ...) {
    char buf[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    xSemaphoreTake(xPrintMutex, portMAX_DELAY);
    Serial.print(buf);
    xSemaphoreGive(xPrintMutex);
}

// Liga/desliga o LED de forma protegida (seção crítica de hardware)
static void setLed(bool on) {
    xSemaphoreTake(xHardwareMutex, portMAX_DELAY);
    digitalWrite(PIN_LED, on ? HIGH : LOW);
#if USE_BUZZER
    // beep curto de confirmação ao mudar o estado
    digitalWrite(PIN_BUZZER, HIGH);
    ets_delay_us(30000); // 30 ms — rápido o suficiente para não travar a task por muito tempo
    digitalWrite(PIN_BUZZER, LOW);
#endif
    xSemaphoreGive(xHardwareMutex);
}

/* ========================= I2S: init / captura ============================ */

static bool i2s_mic_init(uint32_t sample_rate) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = AUDIO_CHANNEL_FORMAT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN
    };

    if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
        safePrintf("ERRO: i2s_driver_install falhou\n");
        return false;
    }
    if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
        safePrintf("ERRO: i2s_set_pin falhou\n");
        return false;
    }
    i2s_zero_dma_buffer(I2S_PORT);
    return true;
}

/* ============================================================================
 * TASK 1 — Captura de Áudio (ALTA prioridade)
 * Lê o I2S continuamente e preenche o double-buffer. Quando um buffer
 * enche, aguarda a confirmação de que ele está livre (Task 3 já processou
 * o ciclo anterior daquele buffer) antes de sobrescrevê-lo, e entrega o
 * buffer pronto para a Task 2 via fila.
 * ==========================================================================*/
static void taskCaptureAudio(void *pvParameters) {
    const size_t bytes_to_read = I2S_DMA_READ_SAMPLES * sizeof(int16_t);
    size_t bytes_read = 0;

    safePrintf("[Task1-Captura] iniciada (prioridade alta)\n");

    while (record_status) {
        esp_err_t r = i2s_read(I2S_PORT, (void *)i2sReadBuffer, bytes_to_read, &bytes_read, portMAX_DELAY);

        if (r != ESP_OK || bytes_read == 0) {
            safePrintf("[Task1] AVISO: falha na leitura I2S (err=%d)\n", (int)r);
            continue;
        }

        int n = bytes_read / sizeof(int16_t);
        for (int i = 0; i < n; i++) {
            i2sReadBuffer[i] = (int16_t)(i2sReadBuffer[i]) * AUDIO_GAIN;
        }

        for (int i = 0; i < n; i++) {
            uint8_t cur = inference.buf_select;
            inference.buffers[cur][inference.buf_count++] = i2sReadBuffer[i];

            if (inference.buf_count >= inference.n_samples) {
                // Buffer 'cur' está cheio -> entrega para a Task 2
                inference.buf_count = 0;
                uint8_t next = cur ^ 1;

                uint8_t ready_idx = cur;
                xQueueSend(xCaptureQueue, &ready_idx, 0);

                // Antes de começar a escrever no outro buffer, garante que a
                // Task 3 já terminou de usá-lo em um ciclo anterior
                // (evita sobrescrever dados que ainda estão sendo classificados).
                if (xSemaphoreTake(xBufferFree[next], pdMS_TO_TICKS(500)) != pdTRUE) {
                    safePrintf("[Task1] AVISO: pipeline mais lento que a captura, "
                               "buffer %d ainda em uso (possível perda de amostras)\n", next);
                }

                inference.buf_select = next;
            }
        }
    }
    vTaskDelete(NULL);
}

/* ============================================================================
 * TASK 2 — Extração de Features (MÉDIA prioridade)
 * Recebe o índice de um buffer cheio, calcula RMS e uma estimativa leve de
 * centróide espectral (via taxa de cruzamento por zero), e repassa o
 * pacote de features + índice do buffer para a Task 3 through fila.
 * ==========================================================================*/
static void taskFeatureExtraction(void *pvParameters) {
    uint8_t buf_idx;
    safePrintf("[Task2-Features] iniciada (prioridade media)\n");

    for (;;) {
        if (xQueueReceive(xCaptureQueue, &buf_idx, portMAX_DELAY) == pdTRUE) {
            int64_t t_start = esp_timer_get_time();

            int16_t *buf = inference.buffers[buf_idx];
            uint32_t n = inference.n_samples;

            // ---- RMS (energia do sinal) ----
            double sum_sq = 0.0;
            for (uint32_t i = 0; i < n; i++) {
                double s = (double)buf[i];
                sum_sq += s * s;
            }
            float rms = sqrtf((float)(sum_sq / n));

            // ---- Estimativa leve de centróide espectral via ZCR ----
            // (um FFT completo por slice seria caro demais para uma task de
            // prioridade média rodando a cada ~250ms; ZCR é um proxy clássico
            // e barato, correlacionado com o "brilho" espectral do sinal,
            // suficiente para gate de silêncio e para o log de latência)
            uint32_t zero_crossings = 0;
            for (uint32_t i = 1; i < n; i++) {
                if ((buf[i - 1] >= 0 && buf[i] < 0) || (buf[i - 1] < 0 && buf[i] >= 0)) {
                    zero_crossings++;
                }
            }
            float zcr = (float)zero_crossings / (float)n;
            float spectral_centroid_approx = zcr * (EI_CLASSIFIER_FREQUENCY / 2.0f);

            int64_t t_end = esp_timer_get_time();

            feature_packet_t pkt;
            pkt.buffer_index          = buf_idx;
            pkt.rms                   = rms;
            pkt.spectral_centroid_hz  = spectral_centroid_approx;
            pkt.is_silence            = (rms < SILENCE_RMS_THRESHOLD);
            pkt.t_capture_done_us     = t_start;
            pkt.t_feature_done_us     = t_end;

            if (xQueueSend(xFeatureQueue, &pkt, pdMS_TO_TICKS(100)) != pdTRUE) {
                safePrintf("[Task2] AVISO: fila de features cheia, pacote descartado "
                           "-> liberando buffer %d\n", buf_idx);
                // se não conseguimos entregar, liberamos o buffer nós mesmos
                xSemaphoreGive(xBufferFree[buf_idx]);
            }
        }
    }
}

/* ============================================================================
 * TASK 3 — Detecção de Anomalia (BAIXA prioridade)
 * Recebe o pacote de features, roda o modelo pré-treinado (MFCC + rede
 * neural, via SDK do Edge Impulse) sobre o buffer de áudio indicado,
 * aplica o threshold de confiança e ativa o alerta (LED conforme comando).
 * Ao final, libera o buffer de volta para a Task 1.
 * ==========================================================================*/

// Callback usado pelo SDK do Edge Impulse para ler o sinal de áudio bruto.
// Lê sempre do buffer atualmente "reservado" para a Task 3 (g_active_buffer).
static volatile uint8_t g_active_buffer = 0;

static int get_audio_signal_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffers[g_active_buffer][offset], out_ptr, length);
    return 0;
}

static void taskAnomalyDetection(void *pvParameters) {
    feature_packet_t pkt;
    bool debug_nn = false;

    safePrintf("[Task3-Deteccao] iniciada (prioridade baixa)\n");

    for (;;) {
        if (xQueueReceive(xFeatureQueue, &pkt, portMAX_DELAY) == pdTRUE) {
            int64_t t_detect_start = esp_timer_get_time();

            // Gate de silêncio: economiza CPU e evita falsos positivos,
            // mas ainda assim precisamos liberar o buffer para a Task 1.
            if (pkt.is_silence) {
                xSemaphoreGive(xBufferFree[pkt.buffer_index]);
                continue;
            }

            g_active_buffer = pkt.buffer_index;

            signal_t signal;
            signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
            signal.get_data = &get_audio_signal_data;

            ei_impulse_result_t result = { 0 };
            EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);

            // Terminamos de ler o buffer -> libera para a Task 1 reutilizar
            xSemaphoreGive(xBufferFree[pkt.buffer_index]);

            if (r != EI_IMPULSE_OK) {
                safePrintf("[Task3] ERRO: run_classifier_continuous falhou (%d)\n", (int)r);
                continue;
            }

            // Encontra a classe de maior confiança NESTA fatia (~250ms) — usada só
            // para log/depuração. A DECISÃO de acionar o LED usa o rótulo
            // suavizado (ei_classifier_smooth_update), não este valor cru,
            // pois uma única fatia de transição pode vir com o rótulo errado.
            int best_idx = 0;
            for (size_t ix = 1; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                if (result.classification[ix].value > result.classification[best_idx].value) {
                    best_idx = ix;
                }
            }
            const char *best_label = result.classification[best_idx].label;
            float best_conf = result.classification[best_idx].value;

            // ---- Suavização oficial do SDK: exige consenso entre várias fatias
            // seguidas antes de aceitar um comando como válido. Retorna
            // "acende" / "apaga" / "unknown" (rótulos do modelo) ou
            // "uncertain" / "anomaly" quando não há consenso suficiente.
            const char *smoothed_label = ei_classifier_smooth_update(&g_smooth, &result);

            int64_t t_detect_end = esp_timer_get_time();

            // ---- Atualiza estado global (protegido por mutex) ----
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            strncpy(g_state.last_label, best_label, sizeof(g_state.last_label) - 1);
            g_state.last_confidence = best_conf;
            g_state.total_slices_processed++;
            bool led_currently_on = g_state.led_on;
            xSemaphoreGive(xStateMutex);

            // ---- Decisão / ação (LED conforme comando de voz, com EDGE-TRIGGER) ----
            // Só liga se o comando suavizado for "acende" E o LED ainda não
            // estiver ligado; só desliga se for "apaga" E o LED ainda estiver
            // ligado. Isso garante que o LED "fica aceso até eu falar apaga",
            // sem repetir a ação (e sem piscar) enquanto a mesma palavra
            // continua sendo reconhecida nas fatias seguintes.
            bool triggered = false;
            if (strcmp(smoothed_label, "acende") == 0 && !led_currently_on) {
                setLed(true);
                triggered = true;
            } else if (strcmp(smoothed_label, "apaga") == 0 && led_currently_on) {
                setLed(false);
                triggered = true;
            }

            if (triggered) {
                xSemaphoreTake(xStateMutex, portMAX_DELAY);
                g_state.led_on = (strcmp(smoothed_label, "acende") == 0);
                g_state.total_commands_detected++;
                g_state.last_action_ms = millis();
                xSemaphoreGive(xStateMutex);
            }

            // ---- Log de latências (ponta-a-ponta) ----
            int64_t total_latency_us = t_detect_end - pkt.t_capture_done_us;
            safePrintf(
                "[Task3] slice=%-8s(%.2f) suavizado=%-9s | RMS=%.1f centroid~%.0fHz | "
                "DSP=%dms NN=%dms | lat.total=%.1fms%s\n",
                best_label, best_conf, smoothed_label, pkt.rms, pkt.spectral_centroid_hz,
                (int)result.timing.dsp, (int)result.timing.classification,
                total_latency_us / 1000.0,
                triggered ? "  <-- ACAO EXECUTADA" : "");
        }
    }
}

/* =============================== SETUP / LOOP ============================ */

static bool startAudioPipeline() {
    inference.n_samples = EI_CLASSIFIER_SLICE_SIZE;
    inference.buf_select = 0;
    inference.buf_count = 0;

    inference.buffers[0] = (int16_t *)malloc(inference.n_samples * sizeof(int16_t));
    inference.buffers[1] = (int16_t *)malloc(inference.n_samples * sizeof(int16_t));
    if (inference.buffers[0] == NULL || inference.buffers[1] == NULL) {
        safePrintf("ERRO: falha ao alocar buffers de audio\n");
        return false;
    }

    if (!i2s_mic_init(EI_CLASSIFIER_FREQUENCY)) {
        return false;
    }

    record_status = true;

    xTaskCreatePinnedToCore(taskCaptureAudio,     "T1_Captura",  STACK_CAPTURE,   NULL, PRIORITY_CAPTURE,   NULL, 1);
    xTaskCreatePinnedToCore(taskFeatureExtraction,"T2_Features", STACK_FEATURE,   NULL, PRIORITY_FEATURE,   NULL, 1);
    xTaskCreatePinnedToCore(taskAnomalyDetection, "T3_Deteccao", STACK_DETECTION, NULL, PRIORITY_DETECTION, NULL, 1);

    return true;
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
#if USE_BUZZER
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
#endif

    xStateMutex    = xSemaphoreCreateMutex();
    xHardwareMutex = xSemaphoreCreateMutex();
    xPrintMutex    = xSemaphoreCreateMutex();

    xBufferFree[0] = xSemaphoreCreateBinary();
    xBufferFree[1] = xSemaphoreCreateBinary();
    // ambos os buffers começam livres
    xSemaphoreGive(xBufferFree[0]);
    xSemaphoreGive(xBufferFree[1]);

    xCaptureQueue = xQueueCreate(2, sizeof(uint8_t));
    xFeatureQueue = xQueueCreate(2, sizeof(feature_packet_t));

    Serial.println("\n=== Detector de Anomalias Acusticas - ESP32 + INMP441 + FreeRTOS ===");
    ei_printf("Comandos reconhecidos: 'acende' (LED ON) / 'apaga' (LED OFF)\n");
    ei_printf("Sample rate: %d Hz | Slice: %d amostras (%d ms) | Threshold: %.2f\n",
        EI_CLASSIFIER_FREQUENCY, EI_CLASSIFIER_SLICE_SIZE,
        (int)((float)EI_CLASSIFIER_SLICE_SIZE * 1000.0f / EI_CLASSIFIER_FREQUENCY),
        (float)EI_CLASSIFIER_THRESHOLD);

    run_classifier_init();

    // 8 leituras (~2s de historico) e exige que pelo menos 5 concordem
    // entre si (maioria) para aceitar um comando -> elimina o "pisca"
    // causado por slices de transicao classificados incorretamente.
    ei_classifier_smooth_init(&g_smooth, /*n_readings=*/8, /*min_readings_same=*/5,
                               /*classifier_confidence=*/(float)EI_CLASSIFIER_THRESHOLD,
                               /*anomaly_confidence=*/0.3f);

    if (!startAudioPipeline()) {
        ei_printf("Falha critica ao iniciar pipeline de audio. Reiniciando em 5s...\n");
        delay(5000);
        ESP.restart();
    }

    ei_printf("Pipeline iniciado. Escutando...\n\n");
}

void loop() {
    // O trabalho pesado roda inteiramente nas 3 tasks acima.
    // O loop() apenas imprime um resumo periódico do estado do sistema,
    // lendo a struct compartilhada de forma protegida por mutex
    // (demonstra um 4o "consumidor" concorrente do estado global).
    static uint32_t last_report = 0;
    uint32_t now = millis();

    if (now - last_report >= 5000) {
        last_report = now;

        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        char label[16];
        strncpy(label, g_state.last_label, sizeof(label));
        float conf = g_state.last_confidence;
        bool led = g_state.led_on;
        uint32_t slices = g_state.total_slices_processed;
        uint32_t commands = g_state.total_commands_detected;
        xSemaphoreGive(xStateMutex);

        safePrintf("---- STATUS: LED=%s | ultimo=%s(%.2f) | slices=%lu | comandos=%lu ----\n",
            led ? "ON" : "OFF", label, conf,
            (unsigned long)slices, (unsigned long)commands);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
}
