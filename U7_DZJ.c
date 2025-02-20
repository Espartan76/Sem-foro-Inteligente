#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "ws2818b.pio.h"  // Biblioteca gerada pelo arquivo .pio durante compilação.


// Variáveis globais
volatile uint8_t count_car_interrupt = 0;       //referentes a interrupção
volatile bool flag_interrupt = false;           //referentes a interrupção
volatile bool on_off_traffic_ligth = false;     //referente ao semáforo (ON/OFF)
volatile bool flag_traffic_off = false;         //referente ao pisca do semaforo


//////////////////////////////////////////////////////////////
///////////////////////// Wi Fi //////////////////////////////
//////////////////////////////////////////////////////////////

#define LED_GREEN 11                 // Define o pino do LED GREEN
#define LED_RED 13                   // Define o pino do LED RED
#define WIFI_SSID "Prometheus 2.4G"  // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASS "2024oi2024"       // Substitua pela senha da sua rede Wi-Fi

// Buffer para respostas HTTP
#define HTTP_RESPONSE "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" \
                      "<!DOCTYPE html><html><body>" \
                      "<h1>Controle do Semaforo</h1>" \
                      "<p><a href=\"/led/on\">Ligar Semaforo</a></p>" \
                      "<p><a href=\"/led/off\">Desligar Semaforo</a></p>" \
                      "</body></html>\r\n"



// Função de callback para processar requisições HTTP
static err_t http_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        // Cliente fechou a conexão
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Processa a requisição HTTP
    char *request = (char *)p->payload;

    if (strstr(request, "GET /led/on")) {
        on_off_traffic_ligth = true;
        gpio_put(LED_GREEN, 1);  // Liga o LED GREEN
        gpio_put(LED_RED, 0);  // Liga o LED RED
        printf("Semaforo ligado: %d\n", gpio_get(LED_GREEN));
    } else if (strstr(request, "GET /led/off")) {
        on_off_traffic_ligth = false;
        gpio_put(LED_GREEN, 0);  // Desliga o LED GREEN
        gpio_put(LED_RED, 1);    // Liga o LED RED
        printf("Semaforo Desligado: %d\n", gpio_get(LED_GREEN));
    }

    // Envia a resposta HTTP
    tcp_write(tpcb, HTTP_RESPONSE, strlen(HTTP_RESPONSE), TCP_WRITE_FLAG_COPY);

    // Libera o buffer recebido
    pbuf_free(p);

    return ERR_OK;
}

// Callback de conexão: associa o http_callback à conexão
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_callback);  // Associa o callback HTTP
    return ERR_OK;
}

// Função de setup do servidor TCP
static void start_http_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("Erro ao criar PCB\n");
        return;
    }

    // Liga o servidor na porta 80
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }

    pcb = tcp_listen(pcb);  // Coloca o PCB em modo de escuta
    tcp_accept(pcb, connection_callback);  // Associa o callback de conexão

    printf("Servidor HTTP rodando na porta 80...\n");
}

//////////////////////////////////////////////////////
/////////CONFIG MODULO SEGUIDOR DE LINHA /////////////
//////////////////////////////////////////////////////

// Defina o pino GPIO onde o TCRT5000 está conectado
#define TCRT5000_PIN 20
// Função para inicializar o sensor
void tcr5000_init() {
    gpio_init(TCRT5000_PIN); // Inicializa o pino GPIO
    gpio_set_dir(TCRT5000_PIN, GPIO_IN); // Configura o pino como entrada
    gpio_pull_down(TCRT5000_PIN); // Habilita o pull-down
}

// Interrupção para contar carros
void count_car_road(uint gpio, uint32_t events){
    static bool flag_car_detected = false;
    static bool car_detected = false;

    if(gpio == TCRT5000_PIN && on_off_traffic_ligth){
        bool car_detected = gpio_get(TCRT5000_PIN);
        if(car_detected && !flag_car_detected){        
            count_car_interrupt++;
            flag_car_detected = true;
            flag_interrupt = true;  
        }else if(!car_detected && flag_car_detected){
            flag_car_detected = false;            
        }
    }
    if(!on_off_traffic_ligth){
        count_car_interrupt = 0;
        flag_interrupt = false;
    }
}


////////////////////////////////////////////////////
/////////////CONFIG MODULO DISPLAY LCD /////////////
////////////////////////////////////////////////////
// Definições do módulo I2C
#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define I2C_BAUDRATE 100000 // 100 kHz é mais comum para LCDs
// Endereço I2C do módulo LCD (verifique no datasheet do seu módulo)
#define LCD_ADDR 0x27

// Função para inicializar o I2C
void i2c_init_lcd() {
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

// Função para enviar dados via I2C
int i2c_write_byte(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    int ret = i2c_write_blocking(I2C_PORT, LCD_ADDR, buf, 2, false);
    if (ret < 0) {
        printf("Erro I2C: %d\n", ret);
    }
    return ret;
}

// Função para enviar comandos ao LCD
void lcd_send_command(uint8_t cmd) {
    uint8_t data_upper = (cmd & 0xF0) | 0x08; // Backlight ON, RS=0, RW=0
    uint8_t data_lower = ((cmd << 4) & 0xF0) | 0x08;

    i2c_write_byte(0x00, data_upper);
    sleep_us(1000);
    i2c_write_byte(0x00, data_upper | 0x04); // Enable pulse
    sleep_us(1000);
    i2c_write_byte(0x00, data_upper);

    i2c_write_byte(0x00, data_lower);
    sleep_us(1000);
    i2c_write_byte(0x00, data_lower | 0x04); // Enable pulse
    sleep_us(1000);
    i2c_write_byte(0x00, data_lower);
}

// Função para enviar caracteres ao LCD
void lcd_send_data(uint8_t data) {
    uint8_t data_upper = (data & 0xF0) | 0x09; // Backlight ON, RS=1, RW=0
    uint8_t data_lower = ((data << 4) & 0xF0) | 0x09;

    i2c_write_byte(0x00, data_upper);
    sleep_us(1000);
    i2c_write_byte(0x00, data_upper | 0x04); // Enable pulse
    sleep_us(1000);
    i2c_write_byte(0x00, data_upper);

    i2c_write_byte(0x00, data_lower);
    sleep_us(1000);
    i2c_write_byte(0x00, data_lower | 0x04); // Enable pulse
    sleep_us(1000);
    i2c_write_byte(0x00, data_lower);
}

// Função para inicializar o LCD
void lcd_init() {
    sleep_ms(100); // Aguarda o LCD inicializar

    lcd_send_command(0x33); // Inicialização
    lcd_send_command(0x32); // Modo de 4 bits
    lcd_send_command(0x28); // 2 linhas, fonte 5x8
    lcd_send_command(0x0C); // Display ON, Cursor OFF, Blink OFF
    lcd_send_command(0x06); // Incrementa cursor
    lcd_send_command(0x01); // Limpa o display
}

// Função para limpar o display
void lcd_clear() {
    lcd_send_command(0x01);
    sleep_ms(2);
}

// Função para posicionar o cursor
void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    lcd_send_command(0x80 | addr);
}

// Função para escrever uma string no LCD
void lcd_write_string(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}


////////////////////////////////////////////////////
/////////////Config Semaforo de leds////////////////
////////////////////////////////////////////////////

#define RED_PIN 4
#define YELLOW_PIN 9
#define GREEN_PIN 8

// Função para acender os leds do semaforo
bool repeating_timer_callback(struct repeating_timer *t){
    
    static uint count_traffic = 0;
    if(on_off_traffic_ligth){
        count_traffic++;
        if(count_traffic <= 5){
            gpio_put(RED_PIN, false);
            gpio_put(YELLOW_PIN, false);
            gpio_put(GREEN_PIN, true);
            //printf("LED verde aceso\n");
        }else if(5 < count_traffic && count_traffic <= 8){
            gpio_put(RED_PIN, false);
            gpio_put(YELLOW_PIN, true);
            gpio_put(GREEN_PIN, false);
            //printf("LED amarelo aceso\n");
        }else if(8 < count_traffic && count_traffic <= 13){
            gpio_put(RED_PIN, true);
            gpio_put(YELLOW_PIN, false);
            gpio_put(GREEN_PIN, false);
            //printf("LED vermelho aceso\n");        
        }
        if(count_traffic == 13){
            count_traffic = 0;
        }
    }else if(!on_off_traffic_ligth){
        static bool flag_traff_off = false;
        count_traffic = 0;

        gpio_put(RED_PIN, false);
        gpio_put(YELLOW_PIN, flag_traff_off);
        gpio_put(GREEN_PIN, false);
        flag_traff_off = !flag_traff_off;
    } 
    return true;

}

//////////////////////////////////////////////////////////////
//////////////// Confiuração do NeoPixel /////////////////////
//////////////////////////////////////////////////////////////
// Definição do número de LEDs e pino.
#define LED_COUNT 25
#define LED_PIN 7

// Definição de pixel GRB
struct pixel_t {
  uint8_t G, R, B; // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.

// Declaração do buffer de pixels que formam a matriz.
npLED_t leds[LED_COUNT];

// Variáveis para uso da máquina PIO.
PIO np_pio;
uint sm;

/**
 * Inicializa a máquina PIO para controle da matriz de LEDs.
 */
void npInit(uint pin) {

  // Cria programa PIO.
  uint offset = pio_add_program(pio0, &ws2818b_program);
  np_pio = pio0;

  // Toma posse de uma máquina PIO.
  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
  }

  // Inicia programa na máquina PIO obtida.
  ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

  // Limpa buffer de pixels.
  for (uint i = 0; i < LED_COUNT; ++i) {
    leds[i].R = 0;
    leds[i].G = 0;
    leds[i].B = 0;
  }
}

/**
 * Atribui uma cor RGB a um LED.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
  leds[index].R = r;
  leds[index].G = g;
  leds[index].B = b;
}

/**
 * Limpa o buffer de pixels.
 */
void npClear() {    
  for (uint i = 0; i < LED_COUNT; ++i)
    npSetLED(i, 0, 0, 0);
}

/**
 * Escreve os dados do buffer nos LEDs.
 */
void npWrite() {
  // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
  for (uint i = 0; i < LED_COUNT; ++i) {
    pio_sm_put_blocking(np_pio, sm, leds[i].G);
    pio_sm_put_blocking(np_pio, sm, leds[i].R);
    pio_sm_put_blocking(np_pio, sm, leds[i].B);
  }
  sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}


////////////////////////////////////////////////////////
///////////Configuração display OLED////////////////////
////////////////////////////////////////////////////////
const uint I2C_SDA_OLED = 14;
const uint I2C_SCL_OLED = 15;

void setup_i2c_oled()
{
    // Inicialização do i2c
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA_OLED, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_OLED, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_OLED);
    gpio_pull_up(I2C_SCL_OLED);
}


// Preparar área de renderização para o display (ssd1306_width pixels por ssd1306_n_pages páginas)
struct render_area frame_area = {
    start_column : 0,
    end_column : ssd1306_width - 1,
    start_page : 0,
    end_page : ssd1306_n_pages - 1
};

//////////////////////////////////////////////////////////
/////////////Função para inicializar os gpios/////////////
//////////////////////////////////////////////////////////
void setup_gpios(){
    //GPIOs do wifi
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_put(LED_RED, true);

    //GPIOs do semáforo
    gpio_init(RED_PIN);
    gpio_set_dir(RED_PIN, GPIO_OUT);
    gpio_init(YELLOW_PIN);
    gpio_set_dir(YELLOW_PIN, GPIO_OUT);
    gpio_init(GREEN_PIN);
    gpio_set_dir(GREEN_PIN, GPIO_OUT);
}

// extern char __StackLimit;  // Limite da pilha (definido no linker script)
// extern char __heap_start;  // Início da heap
// extern char __heap_end;    // Fim da heap

// size_t get_free_memory() {
//     return &__StackLimit - (char*)sbrk(0);
// }


//////////////////////////////////////////////////////////////
///////////////////////// Main ///////////////////////////////
//////////////////////////////////////////////////////////////
int main() {

    stdio_init_all();    
    tcr5000_init(); // Inicializa o sensor TCRT5000
    setup_gpios();
    static bool traffic_on_off = false;
    traffic_on_off = gpio_get(LED_GREEN);
    static bool lcd_on = false;     // Flag para o display LCD -> Semaforo ON
    static bool lcd_off = false;    // Flag para o display LCD -> Semaforo OFF
    static bool  flag_clear_leds = false;
    static uint32_t j = 0, k = 0;


    // Inicializa o Wi-Fi
    if (cyw43_arch_init()) {
        printf("Erro ao inicializar o Wi-Fi\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Falha ao conectar ao Wi-Fi\n");
        return 1;
    }else {
        printf("Connected.\n");
        // Read the ip address in a human readable way
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("Endereço IP %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }

    printf("Wi-Fi conectado!\n");
    // Inicia o servidor HTTP
    start_http_server();
    sleep_ms(5000);

    //Inicializa o LCD
    i2c_init_lcd();
    lcd_init();

    lcd_clear();
    lcd_set_cursor(0, 4);
    lcd_write_string("SISTEMAS");    

    lcd_set_cursor(1, 3);
    lcd_write_string("EMBARCADOS");
    sleep_ms(4000);  

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_write_string("SEMAFORO OFF");
    lcd_set_cursor(1, 0);
    lcd_write_string("car: ");  

    // Inicializa matriz de LEDs NeoPixel.
    npInit(LED_PIN);
    npClear();    
    
    // Inicializa o display OLED
    setup_i2c_oled();
    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();
    // Calcula o tamanho do buffer de renderização
    calculate_render_area_buffer_length(&frame_area);
    
    //Struct para timer
    struct repeating_timer timer;

    // Configura o segundo timer (executa a cada 500 ms)
    add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);
    
    //Interrupção para contar carros na via
    gpio_set_irq_enabled_with_callback(TCRT5000_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &count_car_road);


    while (true) {

        cyw43_arch_poll();  // Necessário para manter o Wi-Fi ativo

        ////////////////////////////////////
        ////////////Semáforo ON/////////////
        ///////////////////////////////////
        if(on_off_traffic_ligth){            
            
            //Limpa o display OLED após transição da sinallização semafórica
            // de ON para OFF
            if(!flag_clear_leds){
                flag_clear_leds = true;
                for(uint m = 0; m < 25; m++){
                    npSetLED(m, 0, 0, 0);
                }
                npWrite();
                printf("npClear()\n");
            }
            if(!lcd_on){
                lcd_on = true;
                lcd_off = false;
                lcd_clear();
                lcd_set_cursor(0, 0);
                lcd_write_string("SEMAFORO ON");
                lcd_set_cursor(1, 0);
                lcd_write_string("car: "); 
            }           
            if(flag_interrupt){
                char buffer_count_car_interrupt[16];
                snprintf(buffer_count_car_interrupt, sizeof(buffer_count_car_interrupt), "%.2hhu", count_car_interrupt);
                lcd_set_cursor(1, 4);
                lcd_write_string(buffer_count_car_interrupt);
            }

            j++;
            k = 0;
    
            // Parte do código para exibir a mensagem no display
            if(j == 1){
                // zera o display inteiro
                uint8_t ssd[ssd1306_buffer_length];
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);        
                char *text[] = {
                    " _____________ ",
                    "|             |",
                    "|  TRANSITO   |",
                    "|    LEVE     |",
                    "|_____________|"
                };
                int y = 0;
                for (uint i = 0; i < count_of(text); i++)
                {
                    ssd1306_draw_string(ssd, 5, y, text[i]);
                    y += 8;
                }
                render_on_display(ssd, &frame_area);
                // Seta siga em frente
                npClear();
                npSetLED(22, 0, 255, 0);     
                npSetLED(18, 0, 255, 0); 
                npSetLED(17, 0, 255, 0); 
                npSetLED(16, 0, 255, 0); 
                npSetLED(14, 0, 255, 0); 
                npSetLED(12, 0, 255, 0); 
                npSetLED(10, 0, 255, 0); 
                npSetLED(7, 0, 255, 0); 
                npSetLED(2, 0, 255, 0);
                npWrite(); // Escreve os dados nos LEDs.                         
            }
            if(j==500){
                // zera o display inteiro
                uint8_t ssd[ssd1306_buffer_length];
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);        
                char *text[] = {
                    " _____________ ",
                    "|             |",
                    "|  TRANSITO   |",
                    "|  MODERADO   |",
                    "|_____________|"
                };
                int y = 0;
                for (uint i = 0; i < count_of(text); i++)
                {
                    ssd1306_draw_string(ssd, 5, y, text[i]);
                    y += 8;
                }
                render_on_display(ssd, &frame_area);
                //Seta siga à direita
                npClear(); 
                npSetLED(22, 255, 0, 0);     
                npSetLED(18, 255, 0, 0);
                npSetLED(10, 255, 0, 0);
                npSetLED(11, 255, 0, 0);
                npSetLED(12, 255, 0, 0);
                npSetLED(13, 255, 0, 0);
                npSetLED(14, 255, 0, 0);
                npSetLED(8, 255, 0, 0);
                npSetLED(2, 255, 0, 0);
                npWrite(); // Escreve os dados nos LEDs.                           
            }
            if(j==1000){                
                printf("Contador j: %d\n", j);
                // zera o display inteiro
                uint8_t ssd[ssd1306_buffer_length];
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);        
                char *text[] = {
                    " _____________ ",
                    "|             |",
                    "|  TRANSITO   |",
                    "|   PESADO    |",
                    "|_____________|"
                };
                int y = 0;
                for (uint i = 0; i < count_of(text); i++)
                {
                    ssd1306_draw_string(ssd, 5, y, text[i]);
                    y += 8;
                }
                render_on_display(ssd, &frame_area);
                // Seta siga à esquerda
                npClear();
                npSetLED(22, 255, 0, 0);     
                npSetLED(16, 255, 0, 0);
                npSetLED(10, 255, 0, 0);
                npSetLED(11, 255, 0, 0);
                npSetLED(12, 255, 0, 0);
                npSetLED(13, 255, 0, 0);
                npSetLED(14, 255, 0, 0);
                npSetLED(6, 255, 0, 0);
                npSetLED(2, 255, 0, 0);
                npWrite(); // Escreve os dados nos LEDs.                        
            }
            if(j==1500){
                j=0;
            }            
    
        }

        ///////////////////////////////////
        ///////////Semáforo OFF////////////
        ///////////////////////////////////
        if(!on_off_traffic_ligth){
            static bool flag_led = false;
            flag_clear_leds = false;

            j = 0;
            if(k == 0 || k == 100){
                if(k == 100){
                    k = 0;
                }
                flag_led = !flag_led;
            }
            if(flag_led){
                for(uint m = 0; m < 25; m++){
                    npSetLED(m, 0, 0, 0);
                }
                npWrite();
                printf("npClear()\n");
            }else if(!flag_led){
                npClear();
                for(uint m = 0; m < 25; m++){
                    npSetLED(m, 255, 255, 0);
                }
                npWrite();
            } 
            k++;
            
            count_car_interrupt = 0;
            if(!lcd_off){
                lcd_off = true;
                lcd_on = false;
                lcd_clear();
                lcd_set_cursor(0, 0);
                lcd_write_string("SEMAFORO OFF");
                lcd_set_cursor(1, 0);
                lcd_write_string("car: ");                 
                printf("!flag_interrupt\n");            
                // zera o display inteiro OLED
                uint8_t ssd[ssd1306_buffer_length];
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);        
                char *text[] = {
                    " _____________ ",
                    "|             |",
                    "|  SEMAFORO   |",
                    "|  DESLIGADO  |",
                    "|_____________|"
                };
                // Exibe a mensagem no display OLED
                int y = 0;
                for (uint i = 0; i < count_of(text); i++)
                {
                    ssd1306_draw_string(ssd, 5, y, text[i]);
                    y += 8;
                }
                render_on_display(ssd, &frame_area); 
            }
        }
        // printf("Memória livre: %u bytes\n", get_free_memory());             
    sleep_ms(10);
    }
    return 0;
}