/*
 * DETECTOR DE CHOQUES - ESP32 CAM
 */
#include <Collision_Detection_System_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// --- PINES ESP32-CAM AI-THINKER ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// --- CONFIGURACIÓN USUARIO ---
#define CONFIDENCE_THRESHOLD 0.75   // 70% de certeza
#define LABEL_OBJETIVO "choque"     // Tu etiqueta exacta

// --- VARIABLES GLOBALES ---
static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf; 
static camera_fb_t *fb = NULL;

// --- DEFINICIÓN DE FUNCIONES ---
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr);

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Iniciando...");

    if (ei_camera_init() == false) {
        Serial.println("Error: Fallo inicio camara");
        return;
    }
    Serial.println("Camara OK");
}

void loop() {
    // 1. Reservar memoria RAM si no está hecha
    if (!is_initialised) {
        // Reservamos memoria suficiente para la imagen QVGA (320x240) o la del modelo, la que sea mayor
        // Usamos un tamaño seguro para evitar desbordes
        snapshot_buf = (uint8_t*)malloc(320 * 240 * 3); 
        if (!snapshot_buf) {
            Serial.println("Error: RAM insuficiente");
            return;
        }
        is_initialised = true;
    }

    // 2. Capturar y procesar imagen
    if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
        Serial.println("Error captura");
        return;
    }

    // 3. Crear señal para la IA
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_cutout_get_data;

    // 4. Ejecutar clasificación
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        Serial.printf("Error clasificador: %d\n", err);
        return;
    }

    // 5. Verificar Detección
    bool choque = false;
    float prob = 0.0;

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        // Descomentar para debug:
        Serial.printf("%s: %.2f | ", result.classification[ix].label, result.classification[ix].value);
        
        if (String(result.classification[ix].label) == LABEL_OBJETIVO) {
            if (result.classification[ix].value >= CONFIDENCE_THRESHOLD) {
                choque = true;
                prob = result.classification[ix].value;
            }
        }
    }
     Serial.println();

    // 6. Acción
    if (choque) {
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.printf("¡CHOQUE DETECTADO! (%.0f%%)\n", prob * 100);
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        delay(2000);
    }
}

// -----------------------------------------------------------
// FUNCIONES TÉCNICAS (NO MODIFICAR)
// -----------------------------------------------------------

bool ei_camera_init(void) {
    if (is_initialised) return true;

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) return false;

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
    }
    return true;
}

void ei_camera_deinit(void) {
    esp_camera_deinit();
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;
    if (!is_initialised) {
        ei_camera_init();
    }

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return false;
    }

    // Convertir JPEG a RGB888
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    
    // Liberar memoria de la camara
    esp_camera_fb_return(fb);
    fb = NULL;

    if (!converted) return false;

    if ((img_width != EI_CLASSIFIER_INPUT_WIDTH) || (img_height != EI_CLASSIFIER_INPUT_HEIGHT)) {
        do_resize = true;
    }

    if (do_resize) {
        // Esta funcion redimensiona la imagen para que coincida con lo que espera el modelo
        ei::image::processing::crop_and_interpolate_rgb888(
            snapshot_buf,
            320, // Ancho QVGA
            240, // Alto QVGA
            snapshot_buf,
            img_width,
            img_height);
    }
    return true;
}

static int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}                           