#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/gpio.h"

// Define LED Pins (ESP32-CAM Ai-Thinker)
#define LED_PROG GPIO_NUM_33  // Custom LED (if connected)
#define LED_CAM  GPIO_NUM_4   // Built-in Camera Flash (Corrected)

// Wi-Fi Configuration (Access Point)
#define WIFI_SSID_AP "mycar"
#define WIFI_PASS_AP "123456789"
#define AP_MAX_CONN 1

// Logging TAG
static const char *TAG = "CAM_WebServer";

// Function to Initialize LEDs
void init_leds() {
    gpio_reset_pin(LED_PROG);
    gpio_reset_pin(LED_CAM);
    gpio_set_direction(LED_PROG, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_CAM, GPIO_MODE_OUTPUT);
}

// Function to Turn LEDs On
void led_on() {
    ESP_LOGI(TAG, "LEDs ON");
    //gpio_set_level(LED_PROG, 1);
    gpio_set_level(LED_CAM, 1);
}

// Function to Turn LEDs Off
void led_off() {
    ESP_LOGI(TAG, "LEDs OFF");
   // gpio_set_level(LED_PROG, 0);
    gpio_set_level(LED_CAM, 0);
}


// MJPEG Stream Handler
static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];

    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "❌ Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf),
                               "--frame\r\n"
                               "Content-Type: image/jpeg\r\n"
                               "Content-Length: %zu\r\n\r\n",
                               fb->len);

        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, "\r\n", 2);
        }

        esp_camera_fb_return(fb);

        // Check if the client has disconnected
        if (httpd_req_recv(req, part_buf, sizeof(part_buf)) != ESP_OK) {
            ESP_LOGI(TAG, "📴 Client disconnected");
            break;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    return res;
}

// Web Server Setup
static httpd_uri_t uri_stream = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };

static httpd_handle_t server = NULL;  // Global server handle

// Start Web Server
static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_stream);
        ESP_LOGI(TAG, "🌐 Web Server Started");
    } else {
        ESP_LOGE(TAG, "❌ Failed to start Web Server");
        server = NULL;
    }
    
    return server;
}

// Stop Web Server
static void stop_webserver(void) {
    if (server) {
        ESP_LOGI(TAG, "🛑 Stopping Web Server...");
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "✅ Web Server Stopped");
    } else {
        ESP_LOGW(TAG, "⚠️ Web Server is not running");
    }
}

// Camera Configuration
#define BOARD_ESP32CAM_AITHINKER 1

#ifdef BOARD_ESP32CAM_AITHINKER
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22
#endif



// Camera Configuration
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG, 
    .frame_size = FRAMESIZE_UXGA,
    .jpeg_quality = 63, 
    .fb_count = 2,  
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};


static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}




// Wi-Fi Event Handler for Access Point (AP) Mode
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "🚀 Wi-Fi AP Started");
                break;

            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
                //ESP_LOGI(TAG, "📶 Client Connected: MAC=" MACSTR ", Total Clients=%d",
                        // MAC2STR(event->mac), event->aid);
                        led_on();
  

                     
                        // Start Web Server
                        start_webserver();
                    
                        //ESP_LOGI(TAG, "📸 Camera Stream Available at: http://"IPSTR"/stream",IP2STR(&event->ip_info.ip));
                break;
            }

            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
               // ESP_LOGW(TAG, "📴 Client Disconnected: MAC=" MACSTR ", Remaining Clients=%d",
                        // MAC2STR(event->mac), event->aid - 1);
                        led_off();
                        stop_webserver();

                         break;

            }

            default:
                break;
        }
    }
}



// Initialize Wi-Fi in Access Point (AP) Mode
static void wifi_init_ap(void) {
    esp_err_t ret;

    // Initialize Network Interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create the AP interface
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    // Initialize Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Configure AP Mode
    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_SSID_AP,
            .ssid_len = strlen(WIFI_SSID_AP),
            .password = WIFI_PASS_AP,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .channel = 1,  // Set Wi-Fi channel (1-13)
            .beacon_interval = 100,  // Default is 100ms
        },
    };

    if (strlen(WIFI_PASS_AP) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // Register Wi-Fi Events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    // Set Wi-Fi Mode and Start AP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Log the AP Information
    if (ap_config.ap.authmode == WIFI_AUTH_OPEN) {
        ESP_LOGI(TAG, "✅ Access Point Started: SSID=%s, Password=None (Open Network)", WIFI_SSID_AP);
    } else {
        ESP_LOGI(TAG, "✅ Access Point Started: SSID=%s, Password=%s", WIFI_SSID_AP, WIFI_PASS_AP);
    }
}


void app_main(void) {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Disable brownout detector

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize LEDs
    init_leds();

    ESP_ERROR_CHECK(init_camera());
    // Initialize Wi-Fi AP Mode
    wifi_init_ap();

}
