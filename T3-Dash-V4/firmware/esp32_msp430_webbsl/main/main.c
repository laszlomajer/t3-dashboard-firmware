#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "wifi_ap_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sbw/SBW430.h"
#include "sbw/gpio_if.h"

#define MAX_HEX_SIZE (48 * 1024)
#define WIFI_AP_CHANNEL 6
#define WIFI_AP_MAX_CLIENTS 4
#define WIFI_AP_IP "192.168.4.1"
#define MSP430_FLASH_START 0xC000u
#define MSP430_FLASH_BYTES 0x4000u
#define MSP430_FLASH_WORDS (MSP430_FLASH_BYTES / 2u)

static const char *TAG = "t3_msp430";
static QueueHandle_t program_queue;
static unsigned ap_client_count;
static volatile bool upload_busy;
static volatile size_t last_image_size;
static volatile unsigned last_data_records;
static volatile bool last_hex_valid;
static volatile int last_program_result = -1;
static const char *msp_link_state = "Nincs adatkapcsolat konfigurálva";
uint16_t *eprom;
const uint32_t eprom_address[1] = {MSP430_FLASH_START};
const uint32_t eprom_length_of_sections[1] = {MSP430_FLASH_WORDS};
const uint32_t eprom_sections = 1;

static const char INDEX_HTML[] =
    "<!doctype html><html lang='hu'><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>T3 műszer – ESP32</title>"
    "<style>:root{color-scheme:light;--ink:#17212b;--muted:#637384;--line:#d8e0e8;--blue:#1268a8;--bg:#f3f6f9}"
    "*{box-sizing:border-box}body{font:16px system-ui;margin:0;background:var(--bg);color:var(--ink)}"
    "header{background:#142838;color:white;padding:1.2rem max(1rem,calc((100% - 900px)/2))}header h1{margin:.1rem 0;font-size:1.35rem}"
    "header p{margin:.35rem 0;color:#cfdae2}main{max-width:900px;margin:1.5rem auto;padding:0 1rem}"
    "nav{display:flex;gap:.5rem;flex-wrap:wrap;margin-bottom:1rem}.tab,button{border:0;border-radius:8px;padding:.7rem 1rem;font:inherit;cursor:pointer}"
    ".tab{background:#e2e9ef;color:var(--ink)}.tab.active,button.primary{background:var(--blue);color:white}"
    ".card{background:white;border:1px solid var(--line);border-radius:12px;padding:1rem 1.2rem;margin:.8rem 0;box-shadow:0 2px 8px #1428380b}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:.7rem}.label{color:var(--muted);font-size:.88rem}.value{font-weight:600;margin-top:.25rem}"
    ".notice{background:#fff8dd;border-left:4px solid #e0a400;padding:.8rem 1rem;border-radius:6px}input{max-width:100%}#result{white-space:pre-wrap}"
    "[hidden]{display:none!important}</style>"
    "<header><h1>T3 műszer – ESP32 hozzáférési pont</h1><p>V4 vezérlő és karbantartás</p></header><main>"
    "<nav><button class='tab active' data-view='diag'>Diagnosztika</button><button class='tab' data-view='update'>MSP430 firmware-frissítés</button></nav>"
    "<section id='diag'><div class='card'><h2>Rendszerállapot</h2><div class='grid'>"
    "<div><div class='label'>Wi-Fi mód / SSID</div><div class='value' id='wifi'>Betöltés…</div></div>"
    "<div><div class='label'>Webcím</div><div class='value' id='ip'>—</div></div>"
    "<div><div class='label'>Csatlakozott kliensek</div><div class='value' id='clients'>—</div></div>"
    "<div><div class='label'>MSP430 adatkapcsolat</div><div class='value' id='msp'>—</div></div>"
    "<div><div class='label'>SBW programozó</div><div class='value' id='program'>—</div></div>"
    "<div><div class='label'>Uptime</div><div class='value' id='uptime'>—</div></div>"
    "<div><div class='label'>Szabad heap</div><div class='value' id='heap'>—</div></div>"
    "<div><div class='label'>Legutóbbi Intel HEX</div><div class='value' id='hex'>—</div></div>"
    "</div></div><div class='card'><h2>Kapcsolati állapot</h2><p id='links'>I²C, BLE és CAN illesztő még nincs konfigurálva.</p>"
    "<p class='label'>Az MSP430 I²C telemetria és az ESP32 adatküldése a hardver interfész kiválasztása után kapcsolható be.</p></div></section>"
    "<section id='update' hidden><div class='card'><h2>MSP430G2553 frissítése SBW-n</h2>"
    "<p>CCS-ből exportált Intel HEX (.hex / .ihx) fájlt válassz.</p>"
    "<input id='file' type='file' accept='.hex,.ihx'> <button class='primary' onclick='upload()'>Feltöltés és programozás</button>"
    "<p class='notice'>A programozás törli és újraírja az MSP430 teljes 16 KiB fő flash területét. Az információs memória nem változik.</p>"
    "<p class='label'>A CCS .out fájlt előbb Intel HEX formátumba kell exportálni. A nyers BIN címadat nélkül nem használható.</p><p id='result'></p></div></section>"
    "</main><script>document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>{document.querySelectorAll('.tab').forEach(x=>x.classList.toggle('active',x===b));"
    "document.querySelector('#diag').hidden=b.dataset.view!=='diag';document.querySelector('#update').hidden=b.dataset.view!=='update'});"
    "async function status(){try{let r=await fetch('/api/status',{cache:'no-store'});let s=await r.json();"
    "document.querySelector('#wifi').textContent=s.mode+' / '+s.ssid;document.querySelector('#ip').textContent='http://'+s.ip+'/';"
    "document.querySelector('#clients').textContent=s.clients;document.querySelector('#msp').textContent=s.msp_link;"
    "document.querySelector('#program').textContent=s.program_state;document.querySelector('#uptime').textContent=s.uptime_s+' s';"
    "document.querySelector('#heap').textContent=s.free_heap+' / '+s.min_free_heap+' byte (aktuális / minimum)';"
    "document.querySelector('#hex').textContent=s.hex_valid?(s.image_size+' byte, '+s.data_records+' adatrekord'):'Még nincs ellenőrzött fájl';"
    "document.querySelector('#links').textContent=s.links}catch(e){document.querySelector('#wifi').textContent='Állapotlekérés sikertelen'}}"
    "status();setInterval(status,2000);async function upload(){let f=document.querySelector('#file').files[0];if(!f)return;"
    "let p=document.querySelector('#result');p.textContent='Feltöltés…';try{let r=await fetch('/api/firmware',{method:'POST',headers:{'Content-Type':'application/octet-stream','X-Filename':f.name},body:f});"
    "p.textContent=await r.text();status()}catch(e){p.textContent='A feltöltés nem sikerült: '+e}}</script></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char body[640];
    const char *program_state = upload_busy ? "SBW programozás folyamatban" :
        (last_program_result == 0 ? "Programozás és visszaellenőrzés kész" :
         (last_program_result > 0 ? "Programozási hiba" : "Várakozik"));
    snprintf(body, sizeof(body),
             "{\"mode\":\"AP\",\"ssid\":\"%s\",\"ip\":\"%s\",\"clients\":%u,"
             "\"msp_link\":\"%s\",\"program_state\":\"%s\",\"hex_valid\":%s,"
             "\"image_size\":%u,\"data_records\":%u,\"uptime_s\":%u,"
             "\"free_heap\":%u,\"min_free_heap\":%u,"
             "\"links\":\"I2C: nincs illesztve | BLE: nincs beállítva | CAN: nincs beállítva\"}",
             WIFI_AP_SSID, WIFI_AP_IP, ap_client_count, msp_link_state, program_state,
             last_hex_valid ? "true" : "false", (unsigned)last_image_size,
             last_data_records, (unsigned)(esp_timer_get_time() / 1000000LL),
             (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool parse_hex_byte(const char *p, uint8_t *value)
{
    int hi = hex_nibble(p[0]), lo = hex_nibble(p[1]);
    if (hi < 0 || lo < 0) return false;
    *value = (uint8_t)((hi << 4) | lo);
    return true;
}

/* Check Intel HEX syntax, byte counts, checksums, and EOF before queueing. */
static bool parse_ihex_image(const char *data, size_t size, unsigned *data_records,
                             uint16_t **image_words)
{
    uint8_t *image_bytes = malloc(MSP430_FLASH_BYTES);
    if (!image_bytes) return false;
    memset(image_bytes, 0xFF, MSP430_FLASH_BYTES);
    size_t pos = 0;
    bool eof_seen = false;
    unsigned records = 0;
    uint32_t address_base = 0;
    while (pos < size) {
        size_t start = pos;
        while (pos < size && data[pos] != '\n' && data[pos] != '\r') pos++;
        size_t len = pos - start;
        while (pos < size && (data[pos] == '\n' || data[pos] == '\r')) pos++;
        if (!len) continue;
        if (eof_seen || data[start] != ':' || len < 11 || ((len - 1) & 1)) goto invalid;
        uint8_t count, addr_hi, addr_lo, type, checksum;
        if (!parse_hex_byte(data + start + 1, &count) ||
            !parse_hex_byte(data + start + 3, &addr_hi) ||
            !parse_hex_byte(data + start + 5, &addr_lo) ||
            !parse_hex_byte(data + start + 7, &type)) goto invalid;
        if (len != 11 + (size_t)count * 2) goto invalid;
        uint8_t sum = (uint8_t)(count + addr_hi + addr_lo + type);
        for (unsigned i = 0; i < count; i++) {
            uint8_t byte;
            if (!parse_hex_byte(data + start + 9 + i * 2, &byte)) goto invalid;
            sum = (uint8_t)(sum + byte);
        }
        if (!parse_hex_byte(data + start + 9 + count * 2, &checksum)) goto invalid;
        sum = (uint8_t)(sum + checksum);
        if (sum != 0) goto invalid;
        uint16_t offset = (uint16_t)(((uint16_t)addr_hi << 8) | addr_lo);
        if (type == 0x00) {
            uint32_t absolute = address_base + offset;
            if (absolute < MSP430_FLASH_START || absolute + count > 0x10000u) goto invalid;
            if (absolute + count > MSP430_FLASH_START + MSP430_FLASH_BYTES) goto invalid;
            for (unsigned i = 0; i < count; i++) {
                uint8_t byte;
                if (!parse_hex_byte(data + start + 9 + i * 2, &byte)) goto invalid;
                image_bytes[absolute - MSP430_FLASH_START + i] = byte;
            }
            records++;
        } else if (type == 0x01 && count == 0 && offset == 0) {
            eof_seen = true;
        } else if (type == 0x02 && count == 2 && offset == 0) {
            uint8_t hi, lo;
            if (!parse_hex_byte(data + start + 9, &hi) || !parse_hex_byte(data + start + 11, &lo)) goto invalid;
            address_base = (((uint32_t)hi << 8) | lo) << 4;
        } else if (type == 0x04 && count == 2 && offset == 0) {
            uint8_t hi, lo;
            if (!parse_hex_byte(data + start + 9, &hi) || !parse_hex_byte(data + start + 11, &lo)) goto invalid;
            address_base = (((uint32_t)hi << 8) | lo) << 16;
        } else if ((type == 0x03 || type == 0x05) && count == 4 && offset == 0) {
            /* Start-address records are metadata; MSP430 reset vectors are in the data. */
        } else goto invalid;
    }
    if (!eof_seen || records == 0) goto invalid;
    uint16_t *words = malloc(MSP430_FLASH_WORDS * sizeof(uint16_t));
    if (!words) goto invalid;
    for (unsigned i = 0; i < MSP430_FLASH_WORDS; i++)
        words[i] = (uint16_t)image_bytes[i * 2] | ((uint16_t)image_bytes[i * 2 + 1] << 8);
    free(image_bytes);
    *data_records = records;
    *image_words = words;
    return true;

invalid:
    free(image_bytes);
    return false;
}

static esp_err_t upload_handler(httpd_req_t *req)
{
    if (req->content_len == 0 || req->content_len > MAX_HEX_SIZE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "A fájl mérete legyen 1–49152 byte.");
        return ESP_FAIL;
    }
    if (upload_busy) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_send(req, "Már folyamatban van egy feltöltés.", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
    char *image = malloc(req->content_len + 1);
    if (!image) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Nincs elég memória a fájlhoz.");
        return ESP_FAIL;
    }
    size_t received = 0;
    while (received < req->content_len) {
        int n = httpd_req_recv(req, image + received, req->content_len - received);
        if (n == HTTPD_SOCK_ERR_FAIL || n <= 0) {
            free(image);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Megszakadt a feltöltés.");
            return ESP_FAIL;
        }
        received += (size_t)n;
    }
    image[received] = '\0';
    unsigned records = 0;
    uint16_t *words = NULL;
    if (!parse_ihex_image(image, received, &records, &words)) {
        last_hex_valid = false;
        free(image);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Hibás Intel HEX, vagy van benne nem támogatott flashcím. CCS-ből a G2553 fő flash címterületére exportálj.");
        return ESP_FAIL;
    }
    free(image);
    upload_busy = true;
    last_hex_valid = true;
    last_image_size = received;
    last_data_records = records;
    last_program_result = -1;
    if (xQueueSend(program_queue, &words, pdMS_TO_TICKS(100)) != pdTRUE) {
        upload_busy = false;
        free(words);
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_send(req, "A feldolgozó feladat nem elérhető.", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, "A fájl érvényes. Az SBW programozás elkezdődött; az állapot itt frissül.");
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 4;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));
    httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = index_handler};
    httpd_uri_t status = {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler};
    httpd_uri_t upload = {.uri = "/api/firmware", .method = HTTP_POST, .handler = upload_handler};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &upload));
    return server;
}

static void sbw_worker(void *arg)
{
    (void)arg;
    uint16_t *words;
    for (;;) {
        if (xQueueReceive(program_queue, &words, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "MSP430G2553 SBW programozás indul (%u flash szó)", MSP430_FLASH_WORDS);
            eprom = words;
            int result = Program_MSP();
            eprom = NULL;
            last_program_result = result;
            if (result == 0) ESP_LOGI(TAG, "SBW programozás és visszaellenőrzés sikeres");
            else ESP_LOGE(TAG, "SBW programozási hiba, TI kód: %d", result);
            free(words);
            upload_busy = false;
        }
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        if (ap_client_count < WIFI_AP_MAX_CLIENTS) ap_client_count++;
        ESP_LOGI(TAG, "Wi-Fi kliens csatlakozott (%u/%u)", ap_client_count, WIFI_AP_MAX_CLIENTS);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        if (ap_client_count > 0) ap_client_count--;
        ESP_LOGI(TAG, "Wi-Fi kliens lecsatlakozott (%u maradt)", ap_client_count);
    }
}

static void start_access_point(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &wifi_event_handler, NULL));
    wifi_config_t ap = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = sizeof(WIFI_AP_SSID) - 1,
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASSWORD,
            .max_connection = WIFI_AP_MAX_CLIENTS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {.required = false},
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi hozzáférési pont elindult: %s", WIFI_AP_SSID);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }
    program_queue = xQueueCreate(1, sizeof(uint16_t *));
    if (!program_queue) ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    SBW_port_init();
    xTaskCreate(sbw_worker, "sbw_worker", 8192, NULL, 5, NULL);
    start_access_point();
    start_webserver();
    ESP_LOGI(TAG, "Webszerver elérhető: http://%s/", WIFI_AP_IP);
}
