/*
 * ESP32-CAM + Bluetooth HFP Audio Gateway + Sarvam STT/TTS + Vision AI
 *
 * Target: original AI-Thinker ESP32-CAM (ESP32, not ESP32-S2/S3/C3)
 * Build: native ESP-IDF
 *
 * Behavior:
 *   1. Pair Realme Buds Wireless 3 Neo with the ESP32-CAM.
 *   2. Press/hold the neckband voice-assistant button.
 *   3. The neckband sends HFP +BVRA; ESP32 starts SCO audio and records the mic.
 *   4. Release the button; ESP32 stops recording and sends the WAV to Sarvam STT.
 *   5. ESP32 captures the camera image.
 *   6. CircuitDigest Vision Cloud receives the user's question + image.
 *   7. Sarvam TTS converts the answer to 8-kHz WAV.
 *   8. Audio is sent back to the neckband through HFP.
 *
 * IMPORTANT:
 *   - Replace WIFI_SSID, WIFI_PASSWORD, VISION_API_KEY, SARVAM_API_KEY.
 *   - This project intentionally uses CVSD/8-kHz HFP for the simplest robust
 *     PCM path. Sarvam supports 8-kHz telephony audio.
 *   - Native ESP-IDF is required for the HFP software-audio path.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_tls.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_ag_api.h"
#include "esp_hf_defs.h"

#include "cJSON.h"
#include "mbedtls/base64.h"

#include "esp_camera.h"

static const char *TAG = "AI_CAM_HFP";

// -----------------------------------------------------------------------------
// USER CONFIGURATION
// -----------------------------------------------------------------------------
#define WIFI_SSID           "Connecting..."
#define WIFI_PASSWORD       "123456789"
#define VISION_API_KEY      "cd_mg1_041026_F8VOCF"
#define SARVAM_API_KEY      "sk_278xxfmd_GKUcjXXZXzvo59yRV1RNhubh"

#define DEVICE_NAME         "ESP32-CAM-AI"

// Capture limits. 8 kHz mono, 16-bit, 15 s = 240 kB.
#define MAX_RECORD_SECONDS  15
#define STT_SAMPLE_RATE     8000
#define TTS_SAMPLE_RATE     8000
#define MAX_AUDIO_BYTES     (STT_SAMPLE_RATE * 2 * MAX_RECORD_SECONDS)

#define VISION_HOST         "www.circuitdigest.cloud"
#define VISION_PATH         "/api/v1/image-to-text/generate"
#define SARVAM_HOST         "api.sarvam.ai"
#define SARVAM_STT_PATH     "/speech-to-text"
#define SARVAM_TTS_PATH     "/text-to-speech"

// AI-Thinker ESP32-CAM pinout.
#define PWDN_GPIO_NUM       32
#define RESET_GPIO_NUM      -1
#define XCLK_GPIO_NUM       0
#define SIOD_GPIO_NUM       26
#define SIOC_GPIO_NUM       27
#define Y9_GPIO_NUM         35
#define Y8_GPIO_NUM         34
#define Y7_GPIO_NUM         39
#define Y6_GPIO_NUM         36
#define Y5_GPIO_NUM         21
#define Y4_GPIO_NUM         19
#define Y3_GPIO_NUM         18
#define Y2_GPIO_NUM         5
#define VSYNC_GPIO_NUM      25
#define HREF_GPIO_NUM       23
#define PCLK_GPIO_NUM       22
#define FLASH_LED_PIN       4

// -----------------------------------------------------------------------------
// GLOBAL STATE
// -----------------------------------------------------------------------------
static esp_bd_addr_t g_hf_addr = {0};
static bool g_hf_connected = false;
static bool g_audio_connected = false;
static esp_hf_sync_conn_hdl_t g_sync_handle = 0;
static uint16_t g_hfp_frame_size = 60; // 30 samples * 2 bytes for CVSD

static volatile bool g_voice_requested = false;
static volatile bool g_recording = false;
static volatile bool g_process_pending = false;
static volatile bool g_abort_recording = false;
static volatile size_t g_recorded_bytes = 0;

static uint8_t *g_record_buffer = nullptr; // PSRAM preferred
static SemaphoreHandle_t g_audio_mutex = nullptr;
static TaskHandle_t g_process_task = nullptr;

static bool g_wifi_connected = false;

// -----------------------------------------------------------------------------
// HELPERS
// -----------------------------------------------------------------------------
static void print_bd_addr(const esp_bd_addr_t addr) {
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

static bool is_nonempty(const char *s) {
    return s && s[0] && strcmp(s, "your_wifi_name") != 0 && strcmp(s, "your_sarvam_API_key") != 0;
}

static std::string json_escape(const char *input) {
    std::string out;
    if (!input) return out;
    for (const char *p = input; *p; ++p) {
        switch (*p) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:   out += *p; break;
        }
    }
    return out;
}

static bool http_collect_response(esp_http_client_handle_t client, std::string &response) {
    response.clear();
    char buf[1024];
    int n;
    while ((n = esp_http_client_read_response(client, buf, sizeof(buf))) > 0) {
        response.append(buf, n);
        if (response.size() > 1024 * 1024) {
            ESP_LOGE(TAG, "HTTP response too large");
            return false;
        }
    }
    return n >= 0;
}

static esp_http_client_handle_t make_https_client(const char *url) {
    esp_http_client_config_t cfg = {};
    cfg.url = url;
    cfg.timeout_ms = 15000;
    cfg.keep_alive_enable = false;
    cfg.skip_cert_common_name_check = true;
    cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    return esp_http_client_init(&cfg);
}

// -----------------------------------------------------------------------------
// WIFI
// -----------------------------------------------------------------------------
static void wifi_event_handler(void *, esp_event_base_t event_base, int32_t event_id, void *) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        ESP_LOGW(TAG, "Wi-Fi disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        g_wifi_connected = true;
        ESP_LOGI(TAG, "Wi-Fi connected");
    }
}

static void wifi_init_sta() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr));

    wifi_config_t wc = {};
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid));
    strncpy((char *)wc.sta.password, WIFI_PASSWORD, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    for (int i = 0; i < 40 && !g_wifi_connected; ++i) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    if (!g_wifi_connected) {
        ESP_LOGE(TAG, "Wi-Fi did not connect");
    }
}

// -----------------------------------------------------------------------------
// CAMERA
// -----------------------------------------------------------------------------
static bool camera_init() {
    camera_config_t config = {};
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
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static camera_fb_t *capture_photo() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) ESP_LOGE(TAG, "Camera capture failed");
    return fb;
}

// -----------------------------------------------------------------------------
// AUDIO RECORDING
// -----------------------------------------------------------------------------
static void reset_recording() {
    if (!g_audio_mutex) return;
    xSemaphoreTake(g_audio_mutex, portMAX_DELAY);
    g_recorded_bytes = 0;
    xSemaphoreGive(g_audio_mutex);
}

static void start_recording() {
    if (!g_record_buffer) return;
    reset_recording();
    g_abort_recording = false;
    g_recording = true;
    ESP_LOGI(TAG, "Recording started");
}

static void stop_recording_and_queue_processing() {
    if (!g_recording) return;
    g_recording = false;
    g_process_pending = true;
    ESP_LOGI(TAG, "Recording stopped: %u bytes", (unsigned)g_recorded_bytes);
}

static void hfp_audio_in_cb(esp_hf_sync_conn_hdl_t sync_conn_hdl, esp_hf_audio_buff_t *audio_buf, bool is_bad_frame) {
    if (!audio_buf) return;

    if (!is_bad_frame && g_recording && audio_buf->data && audio_buf->data_len > 0) {
        size_t left;
        if (xSemaphoreTake(g_audio_mutex, 0) == pdTRUE) {
            left = MAX_AUDIO_BYTES - g_recorded_bytes;
            size_t n = std::min((size_t)audio_buf->data_len, left);
            if (n > 0) {
                memcpy(g_record_buffer + g_recorded_bytes, audio_buf->data, n);
                g_recorded_bytes += n;
            }
            if (g_recorded_bytes >= MAX_AUDIO_BYTES) {
                g_recording = false;
                g_process_pending = true;
            }
            xSemaphoreGive(g_audio_mutex);
        }
    }

    esp_hf_ag_audio_buff_free(audio_buf);
}

// -----------------------------------------------------------------------------
// WAV WRITER
// -----------------------------------------------------------------------------
static void write_le16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void write_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static size_t make_wav(uint8_t *dst, const uint8_t *pcm, size_t pcm_len, uint32_t sample_rate) {
    memcpy(dst, "RIFF", 4);
    write_le32(dst + 4, (uint32_t)(36 + pcm_len));
    memcpy(dst + 8, "WAVE", 4);
    memcpy(dst + 12, "fmt ", 4);
    write_le32(dst + 16, 16);
    write_le16(dst + 20, 1); // PCM
    write_le16(dst + 22, 1); // mono
    write_le32(dst + 24, sample_rate);
    write_le32(dst + 28, sample_rate * 2);
    write_le16(dst + 32, 2);
    write_le16(dst + 34, 16);
    memcpy(dst + 36, "data", 4);
    write_le32(dst + 40, (uint32_t)pcm_len);
    memcpy(dst + 44, pcm, pcm_len);
    return pcm_len + 44;
}

// -----------------------------------------------------------------------------
// SARVAM STT
// -----------------------------------------------------------------------------
static bool sarvam_stt(const uint8_t *wav, size_t wav_len, std::string &transcript, std::string &language) {
    transcript.clear();
    language = "en-IN";
    if (!g_wifi_connected || !wav || wav_len < 46) return false;

    const char *url = "https://api.sarvam.ai/speech-to-text";
    esp_http_client_handle_t client = make_https_client(url);
    if (!client) return false;

    const char *boundary = "----ESP32SarvamBoundary";
    std::string head;
    head += "--"; head += boundary; head += "\r\n";
    head += "Content-Disposition: form-data; name=\"file\"; filename=\"question.wav\"\r\n";
    head += "Content-Type: audio/wav\r\n\r\n";
    std::string tail = "\r\n--" + std::string(boundary) + "--\r\n";

    std::string content_type = "multipart/form-data; boundary=" + std::string(boundary);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "api-subscription-key", SARVAM_API_KEY);
    esp_http_client_set_header(client, "Content-Type", content_type.c_str());
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Content-Length", std::to_string(head.size() + wav_len + tail.size()).c_str());

    esp_err_t err = esp_http_client_open(client, head.size() + wav_len + tail.size());
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sarvam STT open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return false;
    }

    if (esp_http_client_write(client, head.data(), head.size()) < 0 ||
        esp_http_client_write(client, (const char *)wav, wav_len) < 0 ||
        esp_http_client_write(client, tail.data(), tail.size()) < 0) {
        ESP_LOGE(TAG, "Sarvam STT upload failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    esp_err_t fetch_err = esp_http_client_fetch_headers(client);
    if (fetch_err < 0) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    int code = esp_http_client_get_status_code(client);
    std::string response;
    bool ok = http_collect_response(client, response);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    if (!ok || code < 200 || code >= 300) {
        ESP_LOGE(TAG, "Sarvam STT HTTP %d: %s", code, response.c_str());
        return false;
    }

    cJSON *root = cJSON_Parse(response.c_str());
    if (!root) return false;
    cJSON *tr = cJSON_GetObjectItem(root, "transcript");
    cJSON *lc = cJSON_GetObjectItem(root, "language_code");
    if (cJSON_IsString(tr)) transcript = tr->valuestring;
    if (cJSON_IsString(lc) && strlen(lc->valuestring)) language = lc->valuestring;
    cJSON_Delete(root);

    ESP_LOGI(TAG, "STT: [%s] (%s)", transcript.c_str(), language.c_str());
    return !transcript.empty();
}

// -----------------------------------------------------------------------------
// VISION AI
// -----------------------------------------------------------------------------
static std::string language_name(const std::string &code) {
    if (code.rfind("hi", 0) == 0) return "Hindi";
    if (code.rfind("ta", 0) == 0) return "Tamil";
    if (code.rfind("ml", 0) == 0) return "Malayalam";
    if (code.rfind("od", 0) == 0) return "Odia";
    return "English";
}

static bool vision_answer(const uint8_t *jpg, size_t jpg_len, const std::string &question,
                          const std::string &language, std::string &answer) {
    answer.clear();
    if (!g_wifi_connected || !jpg || jpg_len == 0 || question.empty()) return false;

    const char *url = "https://www.circuitdigest.cloud/api/v1/image-to-text/generate";
    esp_http_client_handle_t client = make_https_client(url);
    if (!client) return false;

    const char *boundary = "----ESP32VisionBoundary";
    std::string prompt = "You are a camera assistant. Answer the user's question using the image. "
                         "Be concise, practical, and accurate. Reply only in " + language_name(language) + ". "
                         "Question: " + question;

    std::string p;
    p += "--"; p += boundary; p += "\r\n";
    p += "Content-Disposition: form-data; name=\"prompt\"\r\n\r\n";
    p += prompt;
    p += "\r\n";

    std::string h;
    h += "--"; h += boundary; h += "\r\n";
    h += "Content-Disposition: form-data; name=\"imageFile\"; filename=\"photo.jpg\"\r\n";
    h += "Content-Type: image/jpeg\r\n\r\n";

    std::string tail = "\r\n--" + std::string(boundary) + "--\r\n";
    size_t total = p.size() + h.size() + jpg_len + tail.size();
    std::string content_type = "multipart/form-data; boundary=" + std::string(boundary);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "X-API-Key", VISION_API_KEY);
    esp_http_client_set_header(client, "Content-Type", content_type.c_str());
    esp_http_client_set_header(client, "Content-Length", std::to_string(total).c_str());
    esp_http_client_set_header(client, "Connection", "close");

    if (esp_http_client_open(client, total) != ESP_OK) {
        esp_http_client_cleanup(client);
        return false;
    }

    bool upload_ok =
        esp_http_client_write(client, p.data(), p.size()) >= 0 &&
        esp_http_client_write(client, h.data(), h.size()) >= 0;

    if (upload_ok) {
        const uint8_t *ptr = jpg;
        size_t left = jpg_len;
        while (left) {
            int chunk = (int)std::min((size_t)1024, left);
            if (esp_http_client_write(client, (const char *)ptr, chunk) < 0) {
                upload_ok = false;
                break;
            }
            ptr += chunk;
            left -= chunk;
        }
    }

    if (upload_ok) upload_ok = esp_http_client_write(client, tail.data(), tail.size()) >= 0;

    if (!upload_ok) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    esp_http_client_fetch_headers(client);
    int code = esp_http_client_get_status_code(client);
    std::string response;
    bool read_ok = http_collect_response(client, response);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (!read_ok || code < 200 || code >= 300) {
        ESP_LOGE(TAG, "Vision HTTP %d: %s", code, response.c_str());
        return false;
    }

    cJSON *root = cJSON_Parse(response.c_str());
    if (!root) {
        ESP_LOGE(TAG, "Vision JSON parse failed");
        return false;
    }

    cJSON *v = cJSON_GetObjectItem(root, "generated_text");
    if (!cJSON_IsString(v)) v = cJSON_GetObjectItem(root, "text");
    if (cJSON_IsString(v)) answer = v->valuestring;
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Vision answer: %s", answer.c_str());
    return !answer.empty();
}

// -----------------------------------------------------------------------------
// SARVAM TTS -> HFP
// -----------------------------------------------------------------------------
static std::vector<uint8_t> extract_b64_audio(const std::string &response) {
    std::vector<uint8_t> empty;
    cJSON *root = cJSON_Parse(response.c_str());
    if (!root) return empty;
    cJSON *audios = cJSON_GetObjectItem(root, "audios");
    if (!cJSON_IsArray(audios) || cJSON_GetArraySize(audios) < 1) {
        cJSON_Delete(root);
        return empty;
    }
    cJSON *a0 = cJSON_GetArrayItem(audios, 0);
    if (!cJSON_IsString(a0)) {
        cJSON_Delete(root);
        return empty;
    }
    const char *s = a0->valuestring;
    size_t in_len = strlen(s);
    size_t out_cap = (in_len * 3) / 4 + 8;
    std::vector<uint8_t> data(out_cap);
    size_t out_len = 0;
    int rc = mbedtls_base64_decode(data.data(), data.size(), &out_len,
                                   (const unsigned char *)s, in_len);
    cJSON_Delete(root);
    if (rc != 0) return empty;
    data.resize(out_len);
    return data;
}

static bool hfp_play_pcm_8k(const uint8_t *pcm, size_t pcm_len) {
    if (!g_audio_connected || !pcm || pcm_len < 2 || !g_hf_connected) return false;

    // CVSD HFP is 8 kHz mono PCM. Frame size is supplied by ESP_HF_AUDIO_STATE_EVT.
    size_t frame = g_hfp_frame_size;
    if (frame == 0) frame = 60;

    std::vector<uint8_t> frame_buf(frame);
    for (size_t pos = 0; pos < pcm_len; pos += frame) {
        size_t n = std::min(frame, pcm_len - pos);
        memset(frame_buf.data(), 0, frame);
        memcpy(frame_buf.data(), pcm + pos, n);

        esp_hf_audio_buff_t *ab = esp_hf_ag_audio_buff_alloc(frame);
        if (!ab) return false;
        memcpy(ab->data, frame_buf.data(), frame);
        ab->data_len = frame;

        esp_err_t err = esp_hf_ag_audio_data_send(g_sync_handle, ab);
        if (err != ESP_OK) {
            esp_hf_ag_audio_buff_free(ab);
            ESP_LOGE(TAG, "HFP audio send failed: %s", esp_err_to_name(err));
            return false;
        }
        // CVSD 8-kHz frame period: 30 samples = 3.75 ms.
        vTaskDelay(pdMS_TO_TICKS(4));
    }
    return true;
}

static bool sarvam_tts_and_play(const std::string &text, const std::string &language) {
    if (!g_wifi_connected || text.empty()) return false;

    const char *voice = "shubh";
    if (language.rfind("ta", 0) == 0) voice = "kavitha";
    else if (language.rfind("ml", 0) == 0) voice = "gokul";
    else if (language.rfind("hi", 0) == 0) voice = "shubh";
    else if (language.rfind("od", 0) == 0) voice = "shubh";

    const char *lang_code = language.empty() ? "en-IN" : language.c_str();
    std::string safe_text = json_escape(text.c_str());
    std::string safe_lang = json_escape(lang_code);
    std::string safe_voice = json_escape(voice);

    std::string body = "{";
    body += "\"inputs\":[\"" + safe_text + "\"],";
    body += "\"target_language_code\":\"" + safe_lang + "\",";
    body += "\"speaker\":\"" + safe_voice + "\",";
    body += "\"pace\":1.0,";
    body += "\"speech_sample_rate\":8000,";
    body += "\"model\":\"bulbul:v3\"";
    body += "}";

    std::string url = "https://" + std::string(SARVAM_HOST) + SARVAM_TTS_PATH;
    esp_http_client_handle_t client = make_https_client(url.c_str());
    if (!client) return false;

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "api-subscription-key", SARVAM_API_KEY);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Content-Length", std::to_string(body.size()).c_str());

    if (esp_http_client_open(client, body.size()) != ESP_OK) {
        esp_http_client_cleanup(client);
        return false;
    }
    if (esp_http_client_write(client, body.data(), body.size()) < 0) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    esp_http_client_fetch_headers(client);
    int code = esp_http_client_get_status_code(client);
    std::string response;
    bool ok = http_collect_response(client, response);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (!ok || code < 200 || code >= 300) {
        ESP_LOGE(TAG, "Sarvam TTS HTTP %d: %s", code, response.c_str());
        return false;
    }

    std::vector<uint8_t> wav = extract_b64_audio(response);
    if (wav.size() <= 44 || memcmp(wav.data(), "RIFF", 4) != 0) {
        ESP_LOGE(TAG, "Invalid TTS WAV");
        return false;
    }

    uint16_t channels = wav[22] | (uint16_t(wav[23]) << 8);
    uint32_t rate = uint32_t(wav[24]) | (uint32_t(wav[25]) << 8) |
                    (uint32_t(wav[26]) << 16) | (uint32_t(wav[27]) << 24);
    uint16_t bits = wav[34] | (uint16_t(wav[35]) << 8);
    size_t payload = 44;

    if (channels != 1 || bits != 16 || rate != STT_SAMPLE_RATE) {
        ESP_LOGE(TAG, "Unexpected WAV format: %u ch, %u Hz, %u bits", channels, (unsigned)rate, bits);
        return false;
    }

    if (payload >= wav.size()) return false;
    return hfp_play_pcm_8k(wav.data() + payload, wav.size() - payload);
}

// -----------------------------------------------------------------------------
// HFP GAP + AG CALLBACKS
// -----------------------------------------------------------------------------
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "BT authentication OK");
            } else {
                ESP_LOGE(TAG, "BT authentication failed, status=%d", param->auth_cmpl.stat);
            }
            break;

        case ESP_BT_GAP_PIN_REQ_EVT: {
            esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
            break;
        }

        default:
            break;
    }
}

static void hfp_event_cb(esp_hf_cb_event_t event, esp_hf_cb_param_t *param) {
    if (!param) return;

    switch (event) {
        case ESP_HF_PROF_STATE_EVT:
            ESP_LOGI(TAG, "HFP profile state: %d", param->prof_stat.state);
            break;

        case ESP_HF_CONNECTION_STATE_EVT:
            memcpy(g_hf_addr, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
            ESP_LOGI(TAG, "HFP connection state=%d peer=", param->conn_stat.state);
            print_bd_addr(param->conn_stat.remote_bda);
            printf("\n");
            g_hf_connected = (param->conn_stat.state == ESP_HF_CONNECTION_STATE_SLC_CONNECTED);
            if (!g_hf_connected) {
                g_audio_connected = false;
                g_recording = false;
            } else {
                esp_hf_ag_volume_control(g_hf_addr, ESP_HF_VOLUME_CONTROL_TARGET_MIC, 15);
                esp_hf_ag_volume_control(g_hf_addr, ESP_HF_VOLUME_CONTROL_TARGET_SPK, 15);
            }
            break;

        case ESP_HF_AUDIO_STATE_EVT:
            g_hfp_frame_size = param->audio_stat.preferred_frame_size;
            if (g_hfp_frame_size == 0) g_hfp_frame_size = 60;
            g_sync_handle = param->audio_stat.sync_conn_handle;
            g_audio_connected = (param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED);
            ESP_LOGI(TAG, "HFP audio state=%d frame=%u", param->audio_stat.state, g_hfp_frame_size);
            break;

        case ESP_HF_BVRA_RESPONSE_EVT: {
            esp_hf_vr_state_t vr = param->vra_rep.value;
            ESP_LOGI(TAG, "Neckband voice button: %s", vr == ESP_HF_VR_STATE_ENABLED ? "START" : "STOP");

            // Acknowledge the HFP voice-recognition command.
            esp_hf_ag_vra_control(param->vra_rep.remote_addr, vr);

            if (vr == ESP_HF_VR_STATE_ENABLED) {
                if (g_hf_connected && !g_audio_connected) {
                    esp_hf_ag_audio_connect(param->vra_rep.remote_addr);
                }
                g_voice_requested = true;
                start_recording();
            } else {
                g_voice_requested = false;
                stop_recording_and_queue_processing();
                if (g_audio_connected) {
                    esp_hf_ag_audio_disconnect(param->vra_rep.remote_addr);
                }
            }
            break;
        }

        case ESP_HF_BAC_RESPONSE_EVT:
                    // CVSD is selected by sdkconfig (WBS disabled) for the simple 8-kHz PCM path.
            break;

        default:
            break;
    }
}

static void bluetooth_init() {
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    // HFP software audio over HCI.
    ESP_ERROR_CHECK(esp_bredr_sco_datapath_set(ESP_SCO_DATA_PATH_HCI));

    esp_bluedroid_status_t st = esp_bluedroid_get_status();
    if (st == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        ESP_ERROR_CHECK(esp_bluedroid_init());
    }
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        ESP_ERROR_CHECK(esp_bluedroid_enable());
    }

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(DEVICE_NAME));
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));

    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap)));
    esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
    ESP_ERROR_CHECK(esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin));

    // Register audio callback before starting the HFP profile.
    ESP_ERROR_CHECK(esp_hf_ag_register_audio_data_callback(hfp_audio_in_cb));
    ESP_ERROR_CHECK(esp_hf_ag_register_callback(hfp_event_cb));
    ESP_ERROR_CHECK(esp_hf_ag_init());

    ESP_LOGI(TAG, "Bluetooth HFP AG ready as: %s", DEVICE_NAME);
}

// -----------------------------------------------------------------------------
// PROCESS TASK
// -----------------------------------------------------------------------------
static void process_question() {
    if (!g_wifi_connected || !g_process_pending) return;
    g_process_pending = false;

    size_t pcm_len = 0;
    if (g_audio_mutex) xSemaphoreTake(g_audio_mutex, portMAX_DELAY);
    pcm_len = g_recorded_bytes;
    if (g_audio_mutex) xSemaphoreGive(g_audio_mutex);

    if (pcm_len < 1600) {
        ESP_LOGW(TAG, "Question audio too short");
        return;
    }

    size_t wav_len = pcm_len + 44;
    uint8_t *wav = (uint8_t *)heap_caps_malloc(wav_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!wav) wav = (uint8_t *)malloc(wav_len);
    if (!wav) {
        ESP_LOGE(TAG, "No memory for WAV");
        return;
    }

    make_wav(wav, g_record_buffer, pcm_len, STT_SAMPLE_RATE);

    std::string transcript;
    std::string language;
    if (!sarvam_stt(wav, wav_len, transcript, language)) {
        free(wav);
        return;
    }
    free(wav);

    // Re-open audio only after network processing is complete so the user hears
    // the answer without microphone feedback while STT/Vision are running.
    camera_fb_t *fb = capture_photo();
    if (!fb) return;

    std::string answer;
    bool vision_ok = vision_answer(fb->buf, fb->len, transcript, language, answer);
    esp_camera_fb_return(fb);
    if (!vision_ok) return;

    // HFP audio must be connected before TTS can be heard in the neckband.
    if (!g_audio_connected && g_hf_connected) {
        esp_hf_ag_audio_connect(g_hf_addr);
        for (int i = 0; i < 50 && !g_audio_connected; ++i) vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (g_audio_connected) {
        sarvam_tts_and_play(answer, language);
    }

    if (g_audio_connected && g_hf_connected) {
        esp_hf_ag_audio_disconnect(g_hf_addr);
    }
}

static void processing_task(void *) {
    for (;;) {
        if (g_process_pending) process_question();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// -----------------------------------------------------------------------------
// APP MAIN
// -----------------------------------------------------------------------------
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, " ESP32-CAM Bluetooth AI Voice Vision");
    ESP_LOGI(TAG, "==============================================");

    if (!is_nonempty(WIFI_SSID)) {
        ESP_LOGW(TAG, "Set WIFI_SSID/WIFI_PASSWORD in main.cpp before flashing");
    }
    if (!is_nonempty(SARVAM_API_KEY)) {
        ESP_LOGW(TAG, "Set SARVAM_API_KEY in main.cpp before flashing");
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    gpio_config_t flash_cfg = {};
    flash_cfg.pin_bit_mask = (1ULL << FLASH_LED_PIN);
    flash_cfg.mode = GPIO_MODE_OUTPUT;
    flash_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    flash_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    flash_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&flash_cfg));
    gpio_set_level((gpio_num_t)FLASH_LED_PIN, 0);

    g_audio_mutex = xSemaphoreCreateMutex();
    if (!g_audio_mutex) abort();

    g_record_buffer = (uint8_t *)heap_caps_malloc(MAX_AUDIO_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_record_buffer) g_record_buffer = (uint8_t *)malloc(MAX_AUDIO_BYTES);
    if (!g_record_buffer) abort();
    memset(g_record_buffer, 0, MAX_AUDIO_BYTES);

    wifi_init_sta();
    if (!camera_init()) abort();
    bluetooth_init();

    xTaskCreatePinnedToCore(processing_task, "ai_process", 16384, nullptr, 4, &g_process_task, 1);

    ESP_LOGI(TAG, "READY.");
    ESP_LOGI(TAG, "Pair your Realme Buds Wireless 3 Neo with '%s'.", DEVICE_NAME);
    ESP_LOGI(TAG, "Then press/hold its voice-assistant button, speak, and release.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
