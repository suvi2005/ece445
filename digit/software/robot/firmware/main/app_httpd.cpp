#include <string.h>

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "img_converters.h"

static const char *TAG = "camera_httpd";

#define PART_BOUNDARY "123456789000000000000987654321"

static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static const char *INDEX_HTML =
    "<!doctype html><html><head><meta name=\"viewport\" "
    "content=\"width=device-width,initial-scale=1\">"
    "<title>Digit Robot Camera</title>"
    "<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;}"
    "main{min-height:100vh;display:grid;grid-template-rows:auto 1fr;gap:12px;"
    "place-items:center;padding:16px;box-sizing:border-box;}"
    "h1{font-size:18px;font-weight:600;margin:0;color:#fafafa;}"
    "img{max-width:100%;height:auto;background:#000;}"
    "#state{font-size:14px;color:#aaa;}</style></head>"
    "<body><main><h1>Digit Robot Camera</h1>"
    "<div><img id=\"feed\" alt=\"Robot camera feed\"><p id=\"state\"></p></div>"
    "</main><script>"
    "const img=document.getElementById('feed');"
    "const state=document.getElementById('state');"
    "function streamUrl(){return location.protocol+'//'+location.hostname+':81/stream?t='+Date.now();}"
    "function connect(){state.textContent='Connecting...';img.src=streamUrl();}"
    "img.onload=()=>{state.textContent='Live feed';};"
    "img.onerror=()=>{state.textContent='Camera feed is off or reconnecting';setTimeout(connect,1000);};"
    "connect();"
    "</script></body></html>";

static httpd_handle_t s_camera_httpd = NULL;
static httpd_handle_t s_stream_httpd = NULL;
static volatile bool s_camera_feed_enabled = true;

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    if (!s_camera_feed_enabled) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "Camera feed is off");
    }

    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "20");

    int64_t last_log = esp_timer_get_time();
    uint32_t frames = 0;

    while (s_camera_feed_enabled) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        bool must_free = false;

        if (fb->format == PIXFORMAT_JPEG) {
            jpg_buf = fb->buf;
            jpg_len = fb->len;
        } else {
            must_free = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
            if (!must_free) {
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
                break;
            }
        }

        char part_buf[96];
        size_t header_len = snprintf(part_buf, sizeof(part_buf), STREAM_PART,
                                     (unsigned)jpg_len);

        if (httpd_resp_send_chunk(req, STREAM_BOUNDARY,
                                  strlen(STREAM_BOUNDARY)) != ESP_OK ||
            httpd_resp_send_chunk(req, part_buf, header_len) != ESP_OK ||
            httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len) !=
                ESP_OK) {
            res = ESP_FAIL;
        }

        if (must_free) {
            free(jpg_buf);
        }
        esp_camera_fb_return(fb);

        if (res != ESP_OK) {
            break;
        }

        frames++;
        int64_t now = esp_timer_get_time();
        if ((now - last_log) >= 1000000) {
            ESP_LOGI(TAG, "Streaming %lu fps", (unsigned long)frames);
            frames = 0;
            last_log = now;
        }
    }

    return res;
}

extern "C" void setCameraFeedEnabled(bool enabled)
{
    s_camera_feed_enabled = enabled;
    ESP_LOGI(TAG, "Camera feed %s", enabled ? "enabled" : "disabled");
}

extern "C" bool isCameraFeedEnabled(void)
{
    return s_camera_feed_enabled;
}

extern "C" void startCameraServer(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (s_camera_httpd == NULL) {
        httpd_uri_t index_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = index_handler,
            .user_ctx = NULL,
        };

        ESP_LOGI(TAG, "Starting camera page server on port %d",
                 config.server_port);
        if (httpd_start(&s_camera_httpd, &config) == ESP_OK) {
            httpd_register_uri_handler(s_camera_httpd, &index_uri);
        }
    }

    if (s_stream_httpd == NULL) {
        httpd_uri_t stream_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = stream_handler,
            .user_ctx = NULL,
        };

        config.server_port += 1;
        config.ctrl_port += 1;
        ESP_LOGI(TAG, "Starting camera stream server on port %d",
                 config.server_port);
        if (httpd_start(&s_stream_httpd, &config) == ESP_OK) {
            httpd_register_uri_handler(s_stream_httpd, &stream_uri);
        }
    }
}

extern "C" void stopCameraServer(void)
{
    if (s_stream_httpd != NULL) {
        httpd_stop(s_stream_httpd);
        s_stream_httpd = NULL;
    }

    if (s_camera_httpd != NULL) {
        httpd_stop(s_camera_httpd);
        s_camera_httpd = NULL;
    }

    ESP_LOGI(TAG, "Camera server stopped");
}

extern "C" void setupLedFlash(int pin)
{
    (void)pin;
}
