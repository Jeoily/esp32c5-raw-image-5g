#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#define CHANNEL 40
#define PAYLOAD_SIZE 240
#define HEADER_SIZE 32
#define FRAME_SIZE (24 + HEADER_SIZE + PAYLOAD_SIZE)
#define CAPTURE_SIZE 512
static const uint8_t magic[8] = {'C','5','I','M','G','0','0','1'};
static SemaphoreHandle_t output_lock;
static QueueHandle_t captures;
static uint8_t own_mac[6], peer_mac[6];
static bool have_peer;
static uint32_t seen, failed, misc, dropped, invalid_length, foreign_frames;
static uint32_t current_run, total_bytes, next_sequence;
static uint16_t image_width, image_height;
static bool running;
static bool radio_ready;
static const char *boot_failed_step = "not_initialized";
static char output_text[1400];

static uint32_t crc32_bytes(const uint8_t *data, size_t size) {
    uint32_t crc = 0xffffffff;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1)));
    }
    return ~crc;
}

/* Every host record has a CRC, independent of the wireless frame's FCS. */
static void report(const char *format, ...) {
    xSemaphoreTake(output_lock, portMAX_DELAY);
    va_list args;
    va_start(args, format);
    vsnprintf(output_text, sizeof(output_text), format, args);
    va_end(args);
    printf("!%s*%08" PRIx32 "\n", output_text, crc32_bytes((uint8_t *)output_text, strlen(output_text)));
    fflush(stdout);
    xSemaphoreGive(output_lock);
}

static void put32(uint8_t *out, uint32_t value) {
    for (int i = 0; i < 4; ++i) out[i] = value >> (8 * i);
}

static void put16(uint8_t *out, uint16_t value) {
    out[0] = value; out[1] = value >> 8;
}

static void fill_frame(uint8_t *frame, uint32_t sequence, const uint8_t *pixels, unsigned length) {
    memset(frame, 0, FRAME_SIZE);
    frame[0] = 0x08;  /* non-QoS data, ToDS=FromDS=0 */
    memset(frame + 4, 0xff, 6);  /* broadcast: no per-recipient ACK */
    memcpy(frame + 10, own_mac, 6);
    memcpy(frame + 16, own_mac, 6);
    uint8_t *header = frame + 24;
    memcpy(header, magic, 8);
    put32(header + 8, current_run);
    put32(header + 12, sequence);
    put16(header + 16, image_width);
    put16(header + 18, image_height);
    put16(header + 20, length);
    put16(header + 22, (uint16_t)~length);
    put32(header + 24, 0);  /* format 0: RGB888 plaintext */
    put32(header + 28, crc32_bytes(header, 28));
    memcpy(frame + 56, pixels, length);
}

/* PHY metadata available on the C5 (esp_wifi_rxctrl_t, Wi-Fi MAC version 3).
   The older mcs/sig_mode/cwb fields do not exist on this target; the raw
   L-SIG rate, BB format, secondary channel, noise floor and HE-SIGA1 are
   recorded instead so the host can check which rate really went over the air. */
typedef struct {
    uint32_t timestamp;
    uint16_t sig_len, dump_len, copied;
    uint8_t type, state, channel;
    int8_t rssi;
    uint8_t rate, bb_format, second, local_hint;
    int8_t noise_floor;
    uint32_t he_siga1;
    uint8_t bytes[CAPTURE_SIZE];
} capture_t;

/* Only frames that plausibly belong to this experiment enter the output queue.
   Foreign bad frames (beacons, ACKs, unrelated STAs) are counted but never
   printed: they used to flood the UART, starve the IDLE task and trigger the
   task watchdog, which in turn corrupted !RX records. Frames from our own
   transmitter are kept even when rx_state != 0 so damaged payloads can be
   rendered by the host's raw-error image. */
static uint8_t classify_frame(const uint8_t *payload, const capture_t *item) {
    bool mac_full = item->copied >= 16 && have_peer && !memcmp(payload + 10, peer_mac, 6);
    bool magic_ok = item->copied >= 32 && !memcmp(payload + 24, magic, 8);
    bool mac_prefix = item->copied >= 13 && have_peer && !memcmp(payload + 10, peer_mac, 3);
    /* Our MPDU is 24 + 32 + up to 240 bytes; sig_len includes the 4-byte FCS. */
    bool plausible = item->sig_len >= (24 + HEADER_SIZE + 1) && item->sig_len <= FRAME_SIZE + 4;
    /* A badly damaged header may lose the MAC or magic. Keep long failing DATA
       frames as weak candidates; the host applies its own lenient checks. */
    bool weak = item->state != 0 && plausible && item->copied >= 56;
    return (mac_full ? 1 : 0) | (magic_ok ? 2 : 0) |
           ((mac_prefix && plausible) ? 4 : 0) | (weak ? 8 : 0);
}

static void receive_packet(void *buffer, wifi_promiscuous_pkt_type_t type) {
    const wifi_promiscuous_pkt_t *packet = buffer;
    const wifi_pkt_rx_ctrl_t *ctrl = &packet->rx_ctrl;
    __atomic_add_fetch(&seen, 1, __ATOMIC_RELAXED);
    if (ctrl->rx_state) __atomic_add_fetch(&failed, 1, __ATOMIC_RELAXED);
    if (type == WIFI_PKT_MISC) __atomic_add_fetch(&misc, 1, __ATOMIC_RELAXED);
    capture_t item = {0};
    item.timestamp = ctrl->timestamp;
    item.sig_len = ctrl->sig_len;
    item.dump_len = ctrl->dump_len;
    item.state = ctrl->rx_state;
    item.rssi = ctrl->rssi;
    item.channel = ctrl->channel;
    item.type = type;
    item.rate = ctrl->rate;
    item.bb_format = ctrl->cur_bb_format;
    item.second = ctrl->second;
    item.noise_floor = ctrl->noise_floor;
    item.he_siga1 = ctrl->he_siga1;

    /* MISC has no documented payload. Never read beyond its metadata.
       On the tested C5 SDK the reported dump_len is sig_len + 4, opposite
       to the header comment. Accept either FCS-length convention. sig_len
       includes the FCS, so cap the copy at FRAME_SIZE to drop those 4 bytes. */
    bool payload_type = type == WIFI_PKT_DATA || type == WIFI_PKT_MGMT || type == WIFI_PKT_CTRL;
    unsigned smaller = item.dump_len < item.sig_len ? item.dump_len : item.sig_len;
    unsigned difference = item.dump_len > item.sig_len ? item.dump_len - item.sig_len : item.sig_len - item.dump_len;
    if (payload_type && smaller > 0 && (difference == 0 || difference == 4)) {
        unsigned bound = smaller < FRAME_SIZE ? smaller : FRAME_SIZE;
        item.copied = bound < CAPTURE_SIZE ? bound : CAPTURE_SIZE;
        item.local_hint = classify_frame(packet->payload, &item);
        if (!item.local_hint) {
            __atomic_add_fetch(&foreign_frames, 1, __ATOMIC_RELAXED);
            return;
        }
        memcpy(item.bytes, packet->payload, item.copied);
    } else {
        if (payload_type) __atomic_add_fetch(&invalid_length, 1, __ATOMIC_RELAXED);
        __atomic_add_fetch(&foreign_frames, 1, __ATOMIC_RELAXED);
        return;
    }
    if (xQueueSend(captures, &item, 0) != pdTRUE)
        __atomic_add_fetch(&dropped, 1, __ATOMIC_RELAXED);
}

static void capture_output(void *arg) {
    capture_t item;
    char hex[CAPTURE_SIZE * 2 + 1];
    unsigned since_yield = 0;
    const char digits[] = "0123456789abcdef";
    while (1) {
        if (xQueueReceive(captures, &item, portMAX_DELAY) != pdTRUE) continue;
        for (unsigned i = 0; i < item.copied; ++i) {
            hex[2*i] = digits[item.bytes[i] >> 4];
            hex[2*i+1] = digits[item.bytes[i] & 15];
        }
        hex[item.copied * 2] = 0;
        /* The first ten fields are unchanged for compatibility with old
           records.jsonl. Trailing PHY fields are optional on the host side. */
        report("RX %" PRIu32 " %u %u %d %u %u %u %u %s %u %u %u %d %" PRIu32 " %u",
               item.timestamp, (unsigned)item.type, (unsigned)item.state, (int)item.rssi,
               (unsigned)item.channel, (unsigned)item.sig_len, (unsigned)item.dump_len,
               (unsigned)item.copied, item.copied ? hex : "-",
               (unsigned)item.rate, (unsigned)item.bb_format, (unsigned)item.second,
               (int)item.noise_floor, item.he_siga1, (unsigned)item.local_hint);
        /* Single-core C5: a continuously non-empty queue used to starve the
           IDLE task (task_wdt) and the 1 s counters. CONFIG_FREERTOS_HZ is
           100, so vTaskDelay(1) costs up to 10 ms; yielding after every frame
           would throttle output below the sender's pace. Yielding once every
           16 frames still gives IDLE and the counters a slot roughly every
           100 ms, far inside the 5 s watchdog, at ~0.3 ms/frame amortized. */
        if (++since_yield >= 16) {
            since_yield = 0;
            vTaskDelay(1);
        }
    }
}

/* Independent task: STATS must keep reporting even while capture_output is
   busy. Previously it lived in app_main and went silent for the whole 39 s
   of a 16-round loop, making every bob_queue_drops value a false zero. */
static void stats_output(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        report("STATS %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32,
            __atomic_load_n(&seen, __ATOMIC_RELAXED), __atomic_load_n(&failed, __ATOMIC_RELAXED),
            __atomic_load_n(&misc, __ATOMIC_RELAXED), __atomic_load_n(&dropped, __ATOMIC_RELAXED),
            __atomic_load_n(&invalid_length, __ATOMIC_RELAXED),
            __atomic_load_n(&foreign_frames, __ATOMIC_RELAXED));
    }
}

static void info(void) {
    if (!radio_ready) {
        report("ERROR BOOT_FAILED %s", boot_failed_step);
        return;
    }
    uint8_t channel;
    wifi_second_chan_t secondary;
    ESP_ERROR_CHECK(esp_wifi_get_channel(&channel, &secondary));
    /* Keep the identity string: the host accepts old and new firmware alike,
       and reports missing PHY metadata instead of refusing to start. */
    report("INFO %s %u %02x%02x%02x%02x%02x%02x %s raw_image_v1", ROLE_ALICE ? "Alice" : "Bob",
           channel, own_mac[0], own_mac[1], own_mac[2], own_mac[3], own_mac[4], own_mac[5], esp_get_idf_version());
}

/* A rejected driver setting must leave a readable error, not a reboot loop. */
#define BOOT_CHECK(call) do { \
    boot_failed_step = #call; \
    report("BOOT %s", #call); \
    esp_err_t boot_error = (call); \
    if (boot_error != ESP_OK) { \
        report("ERROR BOOT_FAILED %s %s %d", #call, esp_err_to_name(boot_error), boot_error); \
        return false; \
    } \
} while (0)

static bool wifi_begin(void) {
    BOOT_CHECK(esp_netif_init());
    BOOT_CHECK(esp_event_loop_create_default());
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    BOOT_CHECK(esp_wifi_init(&init));
    BOOT_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    BOOT_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    BOOT_CHECK(esp_wifi_set_country_code("CN", false));
    BOOT_CHECK(esp_wifi_start());
    BOOT_CHECK(esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY));
    BOOT_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11A));
    BOOT_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW20));
    BOOT_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    BOOT_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
    /* Keep the tested C5 order. Moving this before esp_wifi_start() triggers
       the BOOT_FAILED observed on this hardware. The new PHY rate metadata
       lets us verify the resulting rate without guessing. */
    if (ROLE_ALICE) {
        wifi_tx_rate_config_t rate = {.phymode = WIFI_PHY_MODE_11A, .rate = WIFI_PHY_RATE_6M};
        BOOT_CHECK(esp_wifi_config_80211_tx(WIFI_IF_STA, &rate));
    }
    BOOT_CHECK(esp_wifi_get_mac(WIFI_IF_STA, own_mac));
    if (!ROLE_ALICE) {
        wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_ALL};
        BOOT_CHECK(esp_wifi_set_promiscuous_filter(&filter));
        BOOT_CHECK(esp_wifi_set_promiscuous_rx_cb(receive_packet));
        BOOT_CHECK(esp_wifi_set_promiscuous(true));
    }
    return true;
}

static int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void command(char *line) {
    /* Host command CRC prevents serial corruption becoming a radio test error. */
    char *star = strrchr(line, '*');
    if (line[0] != '!' || !star || strlen(star + 1) != 8) {
        report("ERROR command_crc_format"); return;
    }
    uint32_t checksum = 0;
    for (int i = 1; i <= 8; ++i) {
        int nibble = hex_digit(star[i]);
        if (nibble < 0) { report("ERROR command_crc_format"); return; }
        checksum = (checksum << 4) | nibble;
    }
    *star = 0;
    ++line;
    if (crc32_bytes((uint8_t *)line, strlen(line)) != checksum) {
        report("ERROR command_crc_mismatch"); return;
    }
    unsigned run, width, height, power, sequence;
    char extra;
    if (!strcmp(line, "INFO")) { info(); return; }
    if (!strcmp(line, "STOP")) {
        running = false;
        report("STOPPED %" PRIu32 " %" PRIu32, current_run, next_sequence);
        return;
    }
    if (!radio_ready) { report("ERROR BOOT_FAILED %s", boot_failed_step); return; }
    if (!ROLE_ALICE && !strncmp(line, "PEER ", 5) && strlen(line + 5) == 12) {
        uint8_t parsed[6];
        for (int i = 0; i < 6; ++i) {
            char pair[3] = {line[5 + 2*i], line[6 + 2*i], 0};
            char *end;
            unsigned long value = strtoul(pair, &end, 16);
            if (*end) { report("ERROR bad_peer"); return; }
            parsed[i] = value;
        }
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
        memcpy(peer_mac, parsed, 6); have_peer = true;
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
        report("PEER_OK"); return;
    }
    if (ROLE_ALICE && sscanf(line, "BEGIN %u %u %u %u %c", &run, &width, &height, &power, &extra) == 4) {
        if (running || !run || width < 1 || width > 640 || height < 1 || height > 480 || power < 8 || power > 40) {
            report("ERROR bad_run_or_busy"); return;
        }
        esp_err_t err = esp_wifi_set_max_tx_power((int8_t)power);
        if (err != ESP_OK) { report("ERROR tx_power_%s", esp_err_to_name(err)); return; }
        int8_t actual;
        ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&actual));
        current_run = run; image_width = width; image_height = height;
        total_bytes = width * height * 3; next_sequence = 0;
        running = true;
        report("START %u %u %u %d", run, width, height, actual);
        return;
    }
    if (ROLE_ALICE && !strncmp(line, "BLOCK ", 6)) {
        char hex[PAYLOAD_SIZE * 2 + 1];
        if (sscanf(line, "BLOCK %u %u %480s %c", &run, &sequence, hex, &extra) != 3 ||
            !running || run != current_run || sequence != next_sequence || sequence >= (total_bytes + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE) {
            report("ERROR block_order_or_session"); return;
        }
        unsigned remaining = total_bytes - sequence * PAYLOAD_SIZE;
        unsigned length = remaining < PAYLOAD_SIZE ? remaining : PAYLOAD_SIZE;
        if (strlen(hex) != length * 2) { report("ERROR block_length"); return; }
        uint8_t pixels[PAYLOAD_SIZE], frame[FRAME_SIZE];
        for (unsigned i = 0; i < length; ++i) {
            int hi = hex_digit(hex[2*i]), lo = hex_digit(hex[2*i+1]);
            if (hi < 0 || lo < 0) { report("ERROR block_hex"); return; }
            pixels[i] = (hi << 4) | lo;
        }
        fill_frame(frame, sequence, pixels, length);
        /* Exactly one API submission per block. A serial ACK never triggers
           a radio retry. Increment even on API failure to prevent resubmission. */
        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, frame, 56 + length, true);
        next_sequence++;
        report("TX %u %u %d", run, sequence, err);
        return;
    }
    if (ROLE_ALICE && sscanf(line, "END %u %c", &run, &extra) == 1) {
        if (!running || run != current_run || next_sequence != (total_bytes + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE) {
            report("ERROR end_incomplete"); return;
        }
        running = false;
        report("DONE %u %" PRIu32, run, next_sequence);
        return;
    }
    report("ERROR unknown_command");
}

void app_main(void) {
    output_lock = xSemaphoreCreateMutex();
    captures = xQueueCreate(64, sizeof(capture_t));
    configASSERT(output_lock && captures);
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0));
    esp_err_t err = nvs_flash_init();
    /* Preserve existing NVS rather than silently erasing it on a version error. */
    if (err == ESP_OK) radio_ready = wifi_begin();
    else {
        boot_failed_step = "nvs_flash_init";
        report("ERROR BOOT_FAILED nvs_flash_init %s %d", esp_err_to_name(err), err);
    }
    if (!ROLE_ALICE && radio_ready) {
        xTaskCreate(capture_output, "capture_output", 8192, NULL, 2, NULL);
        /* Higher than capture_output so the 1 s counters are never delayed
           by a busy output queue (mutex priority inheritance handles report()). */
        xTaskCreate(stats_output, "stats_output", 3072, NULL, 3, NULL);
    }
    info();
    static char line[640]; size_t used = 0;
    bool discard_line = false;
    while (1) {
        uint8_t byte;
        if (uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(10)) == 1) {
            if (byte == '\n') {
                line[used] = 0;
                if (!discard_line) command(line);
                used = 0; discard_line = false;
            }
            else if (byte != '\r') {
                if (discard_line) continue;
                if (used < sizeof(line)-1) line[used++] = byte;
                else { discard_line = true; report("ERROR command_too_long"); }
            }
        }
    }
}
