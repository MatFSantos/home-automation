#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"            // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"        // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         // Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "lwip/apps/mqtt.h"      // Biblioteca LWIP MQTT -  fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h" // Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"            // Biblioteca que fornece funções e recursos suporte DNS:
#include "lwip/altcp_tls.h"      // Biblioteca que fornece funções e recursos para conexões seguras usando TLS:

#include "lib/pwm.h"
#include "lib/led.h"
#include "lib/push_button.h"
#include "lib/ssd1306.h"

// WIFI credentials
#include "credenciais.h"


/** pin definitions */
#define LM35 26                       /* Y axis */
#define LED_PIN CYW43_WL_GPIO_LED_PIN /* LED of the rp pico w board */
#define BUZZER 21
#define PWM_DIVISER 20
#define PWM_WRAP 2000 /* aprox 3.5kHz freq */

/** global variables */
static volatile double temperature = 0.0;
ssd1306_t ssd;

/** Definições para comunicação MQTT */
#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// Este arquivo inclui seu certificado de cliente para autenticação do servidor cliente
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

// Dados do cliente MQTT
typedef struct
{
    mqtt_client_t *mqtt_client_inst;
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
#define DEBUG_printf printf
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

#define TEMP_WORKER_TIME_S 5 // Temporização da coleta de temperatura - how often to measure our temperature
#define MQTT_KEEP_ALIVE_S 60 // Manter o programa ativo - keep alive in seconds

// QoS - mqtt_subscribe
// No máximo uma vez (QoS 0)
// Ao menos uma vez (QoS 1)
// Exatamente uma vez (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// Tópico usado para: last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

// Definir como 1 para adicionar o nome do cliente aos tópicos, para suportar vários dispositivos que utilizam o mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */

/**
 * @brief Requisição para publicar
 */
static void pub_request_cb(__unused void *arg, err_t err);

/**
 * @brief Topico MQTT
 */
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);

/**
 * @brief Controle do LED
 */
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);

/**
 * @brief Publicar temperatura
 */
static void publish_temperature(MQTT_CLIENT_DATA_T *state);

/**
 * @brief Requisição de Assinatura - subscribe
 */
static void sub_request_cb(void *arg, err_t err);

/**
 * @brief Requisição para encerrar a assinatura
 */
static void unsub_request_cb(void *arg, err_t err);

/**
 * @brief Tópicos de assinatura
 */
static void sub_unsub_topics(MQTT_CLIENT_DATA_T *state, bool sub);

/**
 * @brief Dados de entrada MQTT
 */
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

/**
 * @brief Dados de entrada publicados
 */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);

/**
 * @brief Publicar temperatura
 */
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t temperature_worker = {.do_work = temperature_worker_fn};

/**
 * @brief Conexão MQTT
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

/**
 * @brief Inicializar o cliente MQTT
 */
static void start_client(MQTT_CLIENT_DATA_T *state);

/**
 * @brief Call back com o resultado do DNS
 */
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

/**
 * @brief Initialize the SSD1306 display
 */
void init_display(void);

/**
 * @brief Initialize the all GPIOs that will be used in project
 */
void init_gpio(void);

/**
 * @brief Update the display informations
 *
 * @param message the message that will be ploted in display OLED
 * @param y position on vertical that the message will be ploted
 * @param clear if the display must be cleaned
 */
void update_display(char *message, int16_t x, uint8_t y, bool clear);

/**
 * @brief Function to read the ADC conversor
 *
 * @param channel analog channel to read
 * @return value in percentage readded in selected channel
 */
double read_sensor(uint8_t channel);

/**
 * @brief Executa beeps no buzzer de acordo com a quantidade que foi passada.
 */
void beep(uint8_t times);

float read_temperature(uint8_t channel);

int main() {
    //Inicializa todos os tipos de bibliotecas stdio padrão presentes que estão ligados ao binário.
    stdio_init_all();
    init_gpio();

    // init adc channels
    adc_init();
    adc_gpio_init(LM35);

    // get ws and ssd struct
    init_display();
    update_display("MQTT CLIENT",-1, 18, true);
    update_display("Starting", -1, 28, false);

    // Cria registro com os dados do cliente
    static MQTT_CLIENT_DATA_T state;

    //Inicializa a arquitetura do cyw43
    while (cyw43_arch_init()) {
        update_display("Falha ao", -1, 18, true);
        update_display("iniciar WiFi", -1, 28, false);
        update_display("Aguarde...", -1, 38, false);
        sleep_ms(1000);
    }
    // GPIO do CI CYW43 em nível baixo
    cyw43_arch_gpio_put(LED_PIN, 0);

    // Usa identificador único da placa
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for (int i = 0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Gera nome único, Ex: pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
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
    #if LWIP_ALTCP && LWIP_ALTCP_TLS
        // TLS enabled
    #ifdef MQTT_CERT_INC
        static const uint8_t ca_cert[] = TLS_ROOT_CERT;
        static const uint8_t client_key[] = TLS_CLIENT_KEY;
        static const uint8_t client_cert[] = TLS_CLIENT_CERT;
        // This confirms the indentity of the server and the client
        state.mqtt_client_info.tls_config =
            altcp_tls_create_config_client_2wayauth(
                ca_cert,
                sizeof(ca_cert),
                client_key,
                sizeof(client_key),
                NULL,
                0,
                client_cert,
                sizeof(client_cert)
            );
    #if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
        WARN_printf("Warning: tls without verification is insecure\n");
    #endif
    #else
        state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
        WARN_printf("Warning: tls without a certificate is insecure\n");
    #endif
    #endif

    // Ativa o Wi-Fi no modo Station, de modo a que possam ser feitas ligações a outros pontos de acesso Wi-Fi.
    cyw43_arch_enable_sta_mode();

    // Conectar à rede WiFI
    int i = 0; // para verificar tentativas.
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000)) {
        if (i > 3) {
            update_display("Failed to", -1, 18, true);
            update_display("connect WiFi", -1, 28, false);
            update_display("Reset Pico", -1, 48, false);
            panic("Failed to connect");
        }
        update_display("WiFi", -1, 18, true);
        update_display("Conectando", -1, 28, false);
        update_display("Aguarde...", -1, 38, false);
        sleep_ms(200);
        i++;
    }
    update_display("Conectado WiFi", -1, 18, true);

    if (netif_default)
        update_display(ipaddr_ntoa(&netif_default->ip_addr), -1, 28, false);

    update_display("name:", 3, 38, false);
    update_display(client_id_buf, 43, 38, false);

    // Faz um pedido de DNS para o endereço IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se tiver o endereço, inicia o cliente
    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        update_display("Failed to", -1, 18, true);
        update_display("request DNS", -1, 28, false);
        update_display("name", -1, 38, false);
        update_display("Reset Pico", -1, 48, false);
        panic("dns request failed");
    }
    sleep_ms(2000);
    update_display("MQTT CLIENT ON", -1, 18, true);

    // Loop condicionado a conexão mqtt
    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));
    }

    update_display("mqtt client", -1, 28, true);
    update_display("exiting", -1, 38, false);

    //Desligar a arquitetura CYW43.
    cyw43_arch_deinit();
    return 0;
}

// ============================================================================================================= //
// ======================================== FUNCTIONS CODE ===================================================== //
// ============================================================================================================= //

void init_display() {
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, I2C_ADDRESS, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
}

void init_gpio() {
    /** initialize i2c communication */
    int baudrate = 400 * 1000; // 400kHz baud rate for i2c communication
    i2c_init(I2C_PORT, baudrate);

    // set GPIO pin function to I2C
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);

    /** initialize BLUE LED */
    init_rgb_led();

    /** initialize buttons */
    init_push_button(PIN_BUTTON_A);
    init_push_button(PIN_BUTTON_B);
    init_push_button(PIN_BUTTON_J);

    /** initialize buzzer */
    pwm_init_(BUZZER);
    pwm_setup(BUZZER, PWM_DIVISER, PWM_WRAP);
    pwm_start(BUZZER, 0);
}

void update_display(char *message, int16_t x, uint8_t y, bool clear) {
    if (clear)
        ssd1306_fill(&ssd, false);
    ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
    uint8_t x_pos;
    if (x < 0)
        x_pos = 64 - (strlen(message) * 4);
    else
        x_pos = x;
    ssd1306_draw_string(&ssd, message, x_pos, y);
    ssd1306_send_data(&ssd); // update display
}

void beep(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        pwm_set_gpio_level(BUZZER, PWM_WRAP / 8);
        sleep_ms(50);
        pwm_set_gpio_level(BUZZER, 0);
        sleep_ms(50);
    }
}

double read_sensor(uint8_t channel) {
    if (channel < 26)
        return 0.0;
    adc_select_input(channel - 26);
    double value = (double)adc_read();
    return value * (100.0f / 4095.0f);
}

float read_temperature(uint8_t channel) {
    adc_select_input(channel);
    double value = (double)adc_read();
    /*é adicionado um shift positivo de 1.4V, para que os valores em simulação sejam reais usando o potenciometro dos joysticks */
    value = value - (1400 * 4095 / 3300);
    if (value < 0)
        value = 0;
    return (value * (3300.0f / 4095.0f)) / 10; /* converte a leitura para a temperatura em °C*/
}

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        ERROR_printf("pub_request_cb failed %d", err);
    }
}

//Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
    #if MQTT_UNIQUE_TOPIC
        static char full_topic[MQTT_TOPIC_LEN];
        snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
        return full_topic;
    #else
        return name;
    #endif
}

// Controle do LED 
static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    // Publish state on /state topic and on/off led board
    const char* message = on ? "On" : "Off";
    if (on)
        rgb_led_manipulate(true, true,true);
    else
        rgb_led_manipulate(false, false, false);

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

// Publicar temperatura
static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature;
    const char *temperature_key = full_topic(state, "/temp");
    float temperature = read_temperature(0);
    if (temperature != old_temperature) {
        old_temperature = temperature;
        // Publish temperature on /temp topic
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        INFO_printf("Publishing %s to %s\n", temp_str, temperature_key);
        mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        update_display("sub request", -1, 28, true);
        update_display("failed", -1, 38, false);
        panic("subscribe request failed %d", err);
    }
    state->subscribe_count++;
}

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        update_display("unsub request", -1, 28, true);
        update_display("failed", -1, 38, false);
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/beep"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

// Dados de entrada MQTT
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
    if (strcmp(basic_topic, "/led") == 0)
    {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);
    } else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (strcmp(basic_topic, "/beep") == 0) {
        beep(atoi((const char *)state->data));
    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true; // stop the client when ALL subscriptions are stopped
        sub_unsub_topics(state, false); // unsubscribe
    }
}

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

// Publicar temperatura
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    publish_temperature(state);
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true;
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Publish temperature every 10 sec if it's changed
        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0);
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) {
            update_display("Failed to", -1, 28, true);
            update_display("connect to", -1, 38, false);
            update_display("mqtt server", -1, 48, false);
            panic("Failed to connect to mqtt server");
        }
    }
    else {
        update_display("Unexpected", -1, 28, true);
        update_display("status", -1, 38, false);
        panic("Unexpected status");
    }
}

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state) {
    #if LWIP_ALTCP && LWIP_ALTCP_TLS
        const int port = MQTT_TLS_PORT;
        INFO_printf("Using TLS\n");
    #else
        const int port = MQTT_PORT;
        INFO_printf("Warning: Not using TLS\n");
    #endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        update_display("MQTT client", -1, 28, true);
        update_display("instance", -1, 38, false);
        update_display("creation error", -1, 48, false);
        panic("MQTT client instance creation error");
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        update_display("MQTT broker", -1, 28, true);
        update_display("connection", -1, 38, false);
        update_display("error", -1, 48, false);
        panic("MQTT broker connection error");
    }
    #if LWIP_ALTCP && LWIP_ALTCP_TLS
        // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
        mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
    #endif
        mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
        cyw43_arch_lwip_end();
}

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        update_display("dns request", -1, 28, true);
        update_display("failed", -1, 38, false);
        panic("dns request failed");
    }
}