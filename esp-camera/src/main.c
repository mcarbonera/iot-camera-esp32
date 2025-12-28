// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
// comm
#include <stdio.h>
#include <string.h>
// Camera
#include "esp_camera.h"
// WebSocket
#include "esp_websocket_client.h"
#include "esp_event.h"
#include "esp_log.h"
// Wifi
#include "nvs_flash.h"
#include "esp_wifi.h"

// Config websocket
#define WS_OPCODE_TEXT   0x1
static const char *URI = "ws://192.168.0.8:3000";
static const char *TAG = "WS";
// Config wifi
#define WIFI_CONNECTED_BIT BIT0
#define SSID "<<SSID>>"
#define PASSWORD "<<SENHA>>"
static EventGroupHandle_t wifi_event_group;
static const char *WIFI_TAG = "WIFI";
esp_websocket_client_handle_t ws_client;

// Config Camera
#define CAM_WIDTH 160
#define CAM_HEIGHT 96
#define BYTES_PER_PIXEL 2
#define FRAME_SIZE (CAM_WIDTH * CAM_HEIGHT * BYTES_PER_PIXEL)

static char ascii_buf[8192];
static uint8_t frame_buf_a[FRAME_SIZE];
static uint8_t frame_buf_b[FRAME_SIZE];

static uint8_t *volatile cam_write_buf = frame_buf_a;
static uint8_t *volatile ws_send_buf   = frame_buf_b;

// Sincronização leve
static portMUX_TYPE frame_mux = portMUX_INITIALIZER_UNLOCKED;

// Notificação simples (1 frame pendente)
static TaskHandle_t ws_task_handle = NULL;

typedef struct {
  bool is_ascii;
  int  len;
  uint8_t *data;
} ws_packet_t;

//MARK: Estado Camera
typedef enum {
  MODE_STREAM = 0,
  MODE_ASCII
} stream_mode_t;

static volatile stream_mode_t current_mode = MODE_STREAM;

// MARK: Camera Config
camera_config_t config = {
  .pin_pwdn  = -1,
  .pin_reset = -1,
  .pin_xclk = 21, // MCLK
  .pin_sccb_sda = 18,
  .pin_sccb_scl = 19,
  .pin_d0 = 32,
  .pin_d1 = 33,
  .pin_d2 = 34,
  .pin_d3 = 35,
  .pin_d4 = 36, // VP
  .pin_d5 = 39, // VN
  .pin_d6 = 27,
  .pin_d7 = 14,
  .pin_vsync = 25, // VS
  .pin_href = 23, // HS
  .pin_pclk = 22,
  .xclk_freq_hz = 10 * 1000 *1000, // MHz

  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  //YUV422,GRAYSCALE,RGB565,JPEG
  .pixel_format = PIXFORMAT_RGB565,

  //QQVGA,UXGA
  .frame_size = FRAMESIZE_QQVGA,
  .fb_count = 1,
  //CAMERA_GRAB_LATEST. Sets when buffers should be filled
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
  .fb_location = CAMERA_FB_IN_DRAM
};

// MARK: Print ASCII Art
static inline uint16_t get_pixel_rgb565_linear(
  const uint8_t *frame,
  int x,
  int y
) {
  int idx = (y * CAM_WIDTH + x) * 2;
  return frame[idx] | (frame[idx + 1] << 8);
}

int generate_ascii_from_linear_frame(
  const uint8_t *frame,
  char *out,
  int out_size
) {
  //static const char *shades = " .'`^\",:;Il!i><~+_-?][}{1)(|\\/*tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$";
  static const char *shades = " .:-=+*#%@";
  int n = strlen(shades) - 1;

  int pos = 0;

  // 2x2 pixels por caractere (ajusta proporção)
  for (int y = 0; y < CAM_HEIGHT; y += 2) {
    for (int x = 0; x < CAM_WIDTH; x += 1) {

      uint32_t lum_sum = 0;
      int count = 0;

      // média de um bloco 2x2
      for (int dy = 0; dy < 2; dy++) {
        for (int dx = 0; dx < 2; dx++) {
          int px = x + dx;
          int py = y + dy;
          if (px >= CAM_WIDTH || py >= CAM_HEIGHT) continue;

          uint16_t pixel = get_pixel_rgb565_linear(frame, px, py);

          // extrai RGB
          uint8_t r5 = (pixel >> 11) & 0x1F;
          uint8_t g6 = (pixel >> 5)  & 0x3F;
          uint8_t b5 = pixel & 0x1F;

          // expande para 8 bits
          uint8_t r = (r5 << 3) | (r5 >> 2);
          uint8_t g = (g6 << 2) | (g6 >> 4);
          uint8_t b = (b5 << 3) | (b5 >> 2);

          // ajuste luminancia
          lum_sum += (r * 299 + g * 587 + b * 114) / 1000;
          count++;
        }
      }

      uint8_t lum = lum_sum / count;
      lum = (lum * lum) >> 8;

      int idx = (lum * n) / 255;

      if (pos < out_size - 1) {
        out[pos++] = shades[idx];
      }
    }

    if (pos < out_size - 1) {
      out[pos++] = '\n';
    }
  }

  out[pos] = '\0';
  return pos;
}

// MARK: WebSocket
static void websocket_event_handler(
  void *handler_args,
  esp_event_base_t base,
  int32_t event_id,
  void *event_data)
{
  esp_websocket_event_data_t *data = (esp_websocket_event_data_t *) event_data;
  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      ESP_LOGI(TAG, "WebSocket conectado");
      break;

    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "WebSocket desconectado");
      break;

    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGE(TAG, "Erro no WebSocket");
      break;

    case WEBSOCKET_EVENT_DATA:
      if (data->op_code == WS_OPCODE_TEXT) { // Servidor só manda texto
        if (data->data_len > 32) {
          // não é comando, ignora
          return;
        }
        char cmd[64] = {0};
        int len = data->data_len < sizeof(cmd)-1
                  ? data->data_len
                  : sizeof(cmd)-1;

        memcpy(cmd, data->data_ptr, len);

        ESP_LOGI(TAG, "CMD recebido: %s", cmd);

        if (strcmp(cmd, "ASCII_ON") == 0) {
          current_mode = MODE_ASCII;
        }
        else if (strcmp(cmd, "ASCII_OFF") == 0) {
          current_mode = MODE_STREAM;
        }
      }
      break;

    default:
      break;
  }
}

void websocket_init(void)
{
  esp_websocket_client_config_t ws_cfg = {
    .uri = URI,
    .buffer_size = 4096,
    .network_timeout_ms = 10000,
    .reconnect_timeout_ms = 5000,
  };

  ws_client = esp_websocket_client_init(&ws_cfg);
  esp_websocket_register_events(
    ws_client,
    WEBSOCKET_EVENT_ANY,
    websocket_event_handler,
    NULL
  );

  esp_websocket_client_start(ws_client);
}

// MARK: Task de envio
void websocket_tx_task(void *arg)
{
  ws_task_handle = xTaskGetCurrentTaskHandle();

  while (1) {
    // espera novo frame
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!esp_websocket_client_is_connected(ws_client)) {
      continue;
    }

    if (current_mode == MODE_ASCII) {

      int len = generate_ascii_from_linear_frame(
        ws_send_buf,
        ascii_buf,
        sizeof(ascii_buf)
      );

      esp_websocket_client_send_text(
        ws_client,
        ascii_buf,
        len,
        portMAX_DELAY
      );
    } else {
      esp_websocket_client_send_bin(
        ws_client,
        (const char *)ws_send_buf,
        FRAME_SIZE,
        portMAX_DELAY
      );
    }
  }
}

// MARK: Task Camera
void camera_task(void *pvParameters)
{
  TickType_t last = xTaskGetTickCount();
  while (1) {
    if (!esp_websocket_client_is_connected(ws_client)) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW(TAG, "Frame nulo");
      vTaskDelay(pdMS_TO_TICKS(30));
      continue;
    }

    if (fb->format == PIXFORMAT_RGB565) {

      uint8_t *src = fb->buf;
      uint8_t *dst = cam_write_buf;

      int row_bytes = CAM_WIDTH * 2;
      int stride    = fb->len / CAM_HEIGHT;

      for (int y = 0; y < CAM_HEIGHT; y++) {
        memcpy(dst, src, row_bytes);
        dst += row_bytes;
        src += stride;
        src -= 1;
      }

      // troca atômica dos buffers
      portENTER_CRITICAL(&frame_mux);
      uint8_t *tmp   = ws_send_buf;
      ws_send_buf    = cam_write_buf;
      cam_write_buf  = tmp;
      portEXIT_CRITICAL(&frame_mux);

      // avisa o websocket que tem frame novo
      if (ws_task_handle) {
        xTaskNotifyGive(ws_task_handle);
      }
    }

    esp_camera_fb_return(fb);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(160));
  }
}

// MARK: Wifi
static void wifi_event_handler(
  void *arg,
  esp_event_base_t event_base,
  int32_t event_id,
  void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t *event =
      (wifi_event_sta_disconnected_t *) event_data;
    ESP_LOGW(WIFI_TAG, "WiFi desconectado, tentando reconectar...");
    ESP_LOGW(WIFI_TAG, "reason=%d", event->reason);
    esp_wifi_connect();
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(WIFI_TAG, "IP obtido: " IPSTR, IP2STR(&event->ip_info.ip));
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta(void)
{
  wifi_event_group = xEventGroupCreate();

  // 1. Stack TCP/IP
  ESP_ERROR_CHECK(esp_netif_init());

  // 2. Event loop (se ainda não existir)
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // 3. Interface STA
  esp_netif_create_default_wifi_sta();

  // 4. Inicializa Wi-Fi
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // 5. Registra handlers
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
    WIFI_EVENT,
    ESP_EVENT_ANY_ID,
    &wifi_event_handler,
    NULL,
    NULL
  ));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
    IP_EVENT,
    IP_EVENT_STA_GOT_IP,
    &wifi_event_handler,
    NULL,
    NULL
  ));

  // 6. Configura STA
  wifi_config_t wifi_config = {
    .sta = {
      .ssid = SSID,
      .password = PASSWORD,
      .threshold = {
        .authmode = WIFI_AUTH_WPA2_PSK
      },
    }
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(WIFI_TAG, "Conectando ao WiFi...");

  // 7. BLOQUEIA até conectar
  xEventGroupWaitBits(
    wifi_event_group,
    WIFI_CONNECTED_BIT,
    pdFALSE,
    pdTRUE,
    portMAX_DELAY
  );

  ESP_LOGI(WIFI_TAG, "WiFi conectado com sucesso");
}

// MARK: Main
void app_main(void)
{
  // Inicializa NVS - Non Volatile Storage
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
    ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  // Wifi - bloqueia até ter IP
  wifi_init_sta();

  // Inicializa Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    printf("Falha ao inicializar camera: %d\n", err);
    return;
  }
  printf("Camera inicializada com sucesso!\n");

  // Inicializa Websocket
  websocket_init();

  // Cria task de streaming:
  xTaskCreatePinnedToCore(
    websocket_tx_task,
    "ws_tx",
    4096,
    NULL,
    6,
    NULL,
    0
  );

  // Espera para começar a utilizar a camera.
  vTaskDelay(pdMS_TO_TICKS(5000));

  // Cria task para camera
  xTaskCreatePinnedToCore(
    camera_task,
    "camera_task",
    8192,
    NULL,
    5,
    NULL,
    1
  );
}