#include <string.h>
#include <ctype.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"

#define WIFI_SSID "SSID_AQUI"
#define WIFI_PASSWORD "SENHA_WIFI_AQUI"
#define MQTT_USERNAME "USUARIO_MQTT"
#define MQTT_PASSWORD "SENHA_MQTT"

#ifndef TEMPERATURE_UNITS
#define TEMPERATURE_UNITS 'C'
#endif

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

#define STATE_TOPIC "homeassistant/state"
#define LED_CMD_TOPIC "homeassistant/led"
#define LED_STATE_TOPIC "homeassistant/led/state"
#define AVAIL_TOPIC "homeassistant/online"

typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

#define TEMP_WORKER_TIME_S 10
#define MQTT_KEEP_ALIVE_S 120
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 1
#define MQTT_WILL_TOPIC AVAIL_TOPIC
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

static float read_onboard_temperature(const char unit);
static void pub_request_cb(__unused void *arg, err_t err);
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);
static void publish_temperature(MQTT_CLIENT_DATA_T *state);
static void sub_request_cb(void *arg, err_t err);
static void unsub_request_cb(void *arg, err_t err);
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t temperature_worker = { .do_work = temperature_worker_fn };
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
static void start_client(MQTT_CLIENT_DATA_T *state);
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

int main(void) {
    stdio_init_all();
    INFO_printf("mqtt client starting\n");

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    static MQTT_CLIENT_DATA_T state;

    if (cyw43_arch_init()) {
        panic("Failed to initialize CYW43");
    }

    extern cyw43_t cyw43_state;
    cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);

    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for (int i = 0; i < (int)sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;

    INFO_printf("Device name %s\n", client_id_buf);
    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif

    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        panic("Failed to connect");
    }

    INFO_printf("\nConnected to Wifi\n");

    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) {
        panic("dns request failed");
    }

    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(100));
    }

    INFO_printf("mqtt client exiting\n");
    return 0;
}

static float read_onboard_temperature(const char unit) {
    const float conversionFactor = 3.3f / (1 << 12);
    float adc_v = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc_v - 0.706f) / 0.001721f;
    if (unit == 'C' || unit != 'F') return tempC;
    if (unit == 'F') return tempC * 9 / 5 + 32;
    return -1.0f;
}

static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) ERROR_printf("pub_request_cb failed %d", err);
}

static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}

static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    const char* message = on ? "On" : "Off";
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0);
    mqtt_publish(state->mqtt_client_inst, full_topic(state, LED_STATE_TOPIC),
                 message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN,
                 pub_request_cb, state);
}

static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float last_sent = -1000.0f;
    float t = read_onboard_temperature(TEMPERATURE_UNITS);
    if (fabsf(t - last_sent) >= 0.2f) {
        last_sent = t;
        char payload[64];
        snprintf(payload, sizeof(payload), "{\"temperature\": %.2f}", t);
        INFO_printf("Publishing %s to %s\n", payload, STATE_TOPIC);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, STATE_TOPIC),
                     payload, strlen(payload), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN,
                     pub_request_cb, state);
    }
}

static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) panic("subscribe request failed %d", err);
    state->subscribe_count++;
}

static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) panic("unsubscribe request failed %d", err);
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, LED_CMD_TOPIC), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "homeassistant/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "homeassistant/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "homeassistant/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';
    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);

    if (strcmp(basic_topic, LED_CMD_TOPIC) == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);
    } else if (strcmp(basic_topic, "homeassistant/print") == 0) {
        INFO_printf("%.*s\n", len, data);
    } else if (strcmp(basic_topic, "homeassistant/ping") == 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "homeassistant/uptime"),
                     buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (strcmp(basic_topic, "homeassistant/exit") == 0) {
        state->stop_client = true;
        sub_unsub_topics(state, false);
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    publish_temperature(state);
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;

    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true;
        sub_unsub_topics(state, true);

        mqtt_publish(state->mqtt_client_inst, full_topic(state, AVAIL_TOPIC), "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);

        const char *cid = state->mqtt_client_info.client_id;

        const char *cfg_temp_topic = "homeassistant/sensor/pico_temperature/config";
        char cfg_temp_payload[512];
        snprintf(cfg_temp_payload, sizeof(cfg_temp_payload),
                 "{\"name\":\"Pico Temperature\",\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
                 "\"payload_available\":\"1\",\"payload_not_available\":\"0\",\"unit_of_measurement\":\"°C\","
                 "\"device_class\":\"temperature\",\"state_class\":\"measurement\",\"value_template\":\"{{ value_json.temperature }}\","
                 "\"unique_id\":\"pico_temp_%s\",\"device\":{\"identifiers\":[\"pico_%s\"],\"name\":\"Pico W\","
                 "\"model\":\"RP2040\",\"manufacturer\":\"Raspberry Pi\"}}",
                 STATE_TOPIC, AVAIL_TOPIC, cid, cid);

        const char *cfg_led_topic = "homeassistant/switch/pico_led/config";
        char cfg_led_payload[512];
        snprintf(cfg_led_payload, sizeof(cfg_led_payload),
                 "{\"name\":\"Pico LED\",\"command_topic\":\"%s\",\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
                 "\"payload_available\":\"1\",\"payload_not_available\":\"0\",\"payload_on\":\"On\",\"payload_off\":\"Off\","
                 "\"unique_id\":\"pico_led_%s\",\"device\":{\"identifiers\":[\"pico_%s\"],\"name\":\"Pico W\","
                 "\"model\":\"RP2040\",\"manufacturer\":\"Raspberry Pi\"}}",
                 LED_CMD_TOPIC, LED_STATE_TOPIC, AVAIL_TOPIC, cid, cid);

        mqtt_publish(state->mqtt_client_inst, cfg_temp_topic, cfg_temp_payload, strlen(cfg_temp_payload), 1, 1, pub_request_cb, state);
        mqtt_publish(state->mqtt_client_inst, cfg_led_topic, cfg_led_payload, strlen(cfg_led_payload), 1, 1, pub_request_cb, state);

        mqtt_publish(state->mqtt_client_inst, full_topic(state, LED_STATE_TOPIC), "Off", 3, 1, 1, pub_request_cb, state);

        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0);

    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) panic("Failed to connect to mqtt server");
    } else {
        panic("Unexpected status");
    }
}

static void start_client(MQTT_CLIENT_DATA_T *state) {
    const int port = MQTT_PORT;
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) panic("MQTT client instance creation error");

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK)
        panic("MQTT broker connection error");
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        panic("dns request failed");
    }
}

