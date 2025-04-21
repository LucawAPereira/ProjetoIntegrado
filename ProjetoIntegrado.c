#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "pico/bootrom.h"
#include <time.h>  
#include "ws2812.pio.h"


#define buzzer1 21  
#define buzzer2 10 
#define LEDb 12
#define LEDr 13
#define LEDg 11
#define BotaoA 5
#define BotaoB 6
#define VRY 26
#define VRX 27
#define wrap 4096
#define div 40
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define IS_RGBW false
#define NUM_PIXELS 25
#define WS2812_PIN 7

bool pwm_enabled=0;
bool cor = true;
bool centralizado = true;
bool ativacao_buzzer=false;
float conversao_x = 0;
float conversao_y = 0;
ssd1306_t ssd;
uint slice_r, slice_b, slice_g;
uint8_t led_r = 30; // Intensidade do vermelho
uint8_t led_g = 0; // Intensidade do verde
uint8_t led_b = 0; // Intensidade do azul

void gpio_irq_handler(uint gpio, uint32_t events);
void quadradinho_aleatorio(int* x, int* y);
void alerta_de_colisao();
void play_note(int frequency, int duration, int pin); //funcao que ativa os buzzers
static volatile uint32_t last_time = 0;

bool led_buffer0[NUM_PIXELS] = { // ascende os leds na colisao
    1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1,                      // [0,1,2,3,4,
    1, 1, 1, 1, 1,                      //  5,6,7,8,9
    1, 1, 1, 1, 1,                      //  10,11,12,13,14   ---> como realmente é a ordem da matriz de leds
    1, 1, 1, 1, 1                       //  15,16,17,18,19
}; 

static inline void put_pixel(uint32_t pixel_grb)  // protocolo WS18
{
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) //  protoclo WS2812
{
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

void set_one_led(uint8_t r, uint8_t g, uint8_t b, int buffer_index)  // funcao para escrever o numero na matriz de leds
{
    bool* led_buffer[] = {led_buffer0}; // vetor desenho numeros

    // Define a cor com base nos parâmetros fornecidos
    uint32_t color = urgb_u32(r, g, b);
    bool* buffer = led_buffer[buffer_index]; // armazena o array na variavel buffer, que será enviado para o vetor de 25 leds correspondente ao desenho 

    // Define todos os LEDs com a cor especificada
    
        for (int i = 0; i < NUM_PIXELS; i++)
        {
        if (buffer[i])
        {
        put_pixel(color); // Liga o LED
        }
        else
        {
        put_pixel(0);  // Desliga o LED
        }
    }

}

int main() {
    stdio_init_all();
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    gpio_set_irq_enabled_with_callback(BotaoA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BotaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(buzzer1);
    gpio_set_dir(buzzer1, GPIO_OUT);
    gpio_init(buzzer2);
    gpio_set_dir(buzzer2, GPIO_OUT);
    gpio_init(BotaoA); 
    gpio_set_dir(BotaoA, GPIO_IN); 
    gpio_pull_up(BotaoA);
    gpio_init(BotaoB); 
    gpio_set_dir(BotaoB, GPIO_IN); 
    gpio_pull_up(BotaoB);

    gpio_set_function(LEDr, GPIO_FUNC_PWM);
    gpio_set_function(LEDb, GPIO_FUNC_PWM);
    gpio_set_function(LEDg, GPIO_FUNC_PWM);

    slice_r = pwm_gpio_to_slice_num(LEDr);
    slice_b = pwm_gpio_to_slice_num(LEDb);
    slice_g = pwm_gpio_to_slice_num(LEDg);

    pwm_set_clkdiv(slice_r, div); pwm_set_wrap(slice_r, wrap); pwm_set_enabled(slice_r, true);
    pwm_set_clkdiv(slice_b, div); pwm_set_wrap(slice_b, wrap); pwm_set_enabled(slice_b, true);
    pwm_set_clkdiv(slice_g, div); pwm_set_wrap(slice_g, wrap); pwm_set_enabled(slice_g, true);

    adc_init();
    adc_gpio_init(VRY);
    adc_gpio_init(VRX);

    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    adc_init();
    adc_gpio_init(28);
    adc_select_input(2);
    sleep_ms(1);
    uint16_t noise = adc_read(); // ler um ruido aleatorio do pino para gerar valores aleatorios

    // mistura os bits para “espalhar” a entropia
    uint32_t seed = noise ^ (noise << 8) ^ (noise << 16);
    srand(seed);

    // agora seu rand() vai variar a cada boot
    int random_x, random_y;
    quadradinho_aleatorio(&random_x, &random_y);
    
    ssd1306_fill(&ssd, !cor);
    ssd1306_rect(&ssd, random_y + 4215, random_x, 8, 8, 1, 0); // desenha o quadradinho aleatorio
    ssd1306_rect(&ssd, 0, 0, 128, 62, cor, !cor);
    ssd1306_send_data(&ssd);

    while (true) {
        adc_select_input(0); int VRY_value = adc_read();
        adc_select_input(1); int VRX_value = adc_read();
    
        conversao_y = (4095 - VRY_value * 56) / 4093;
        conversao_x = VRX_value * 116 / 4096;

        if (centralizado == true) // comeca semprem com o quadrado no centro do display
        {
            ssd1306_fill(&ssd, !cor); // Limpa o display
            ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); // Desenha um retângulo 
            ssd1306_rect(&ssd, 28, 60, 8, 8, 1, 1);
            ssd1306_send_data(&ssd); // Atualiza o display
            centralizado = false;
            sleep_ms(20);
            set_one_led(0, 0, 0, 0);
            sleep_ms(20);
        }

        if(VRX_value >= 1900 && VRX_value <= 2200 && VRY_value >= 1900 && VRY_value <= 2200 )
        {
            pwm_set_gpio_level(LEDr, 0);
            pwm_set_gpio_level(LEDb, 0);
            pwm_set_gpio_level(LEDg, 0);
            ssd1306_fill(&ssd, !cor); // Limpa o display   
            ssd1306_rect(&ssd, 3, 3, 8, 8, 1, 1);
            ssd1306_rect(&ssd, 28, 60, 8, 8, 1, 1); // (, y, x, tam_y, tam_x, 1,1) mantem o quadrado no centro do display
        }
        if (VRY_value <= 1900 || VRY_value >= 2200) {
            pwm_set_gpio_level(LEDr, VRY_value);
            pwm_set_gpio_level(LEDg, VRY_value);

        }
        if (VRX_value <= 1900 || VRX_value >= 2200){
            pwm_set_gpio_level(LEDb, VRX_value);
            pwm_set_gpio_level(LEDg, VRY_value);
        }
    
        // Atualiza a tela
        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, conversao_y + 4215, conversao_x, 8, 8, 1, 1);
        ssd1306_rect(&ssd, 0, 0, 128, 62, cor, !cor);
        ssd1306_send_data(&ssd);

        ssd1306_rect(&ssd, random_y + 4215, random_x, 8, 8, 1, 0); // Desenha o quadradinho aleatório novamente  
        ssd1306_rect(&ssd, 0, 0, 128, 62, cor, !cor);  
        ssd1306_send_data(&ssd);

        if (random_x >= 56){ // colisao para a parte direta da tela, se dividirmos a mesma ao meio
            if (conversao_x > random_x + 1) {
            alerta_de_colisao();
    
             // Congela o sistema
            while (true) {
            set_one_led(led_r, led_g, led_b, 0);
            ssd1306_fill(&ssd, !cor);
            ssd1306_draw_string(&ssd,"COLISAO!", 20, 15);
            ssd1306_draw_string(&ssd,"Reinicie o jogo", 4, 35);
            ssd1306_send_data(&ssd);
            play_note(440, 150, buzzer1);
            sleep_ms(300);
            play_note(440, 150, buzzer1);
            sleep_ms(800);
            }
        }
    } else { // colisao para a parte esquerda da tela, se dividirmos a mesma ao meio
        if (conversao_x == random_x + 5 || conversao_x == random_x + 4 || conversao_x == random_x + 6 || conversao_x == random_x + 3) {
            alerta_de_colisao();
    
             // Congela o sistema
            while (true) {
            set_one_led(led_r, led_g, led_b, 0);
            ssd1306_fill(&ssd, !cor);
            ssd1306_draw_string(&ssd,"COLISAO", 20, 15);
            ssd1306_draw_string(&ssd,"Reinicie o jogo", 4, 35);
            ssd1306_send_data(&ssd);
            play_note(440, 150, buzzer1);
            sleep_ms(300);
            play_note(440, 150, buzzer1);
            sleep_ms(800);
            }
        }

    }
    
        printf("conversao X = %f\n", conversao_x); // diferença de 7 pasa considerar toque na borda
        printf("conversao Y = %f\n", conversao_y);
        printf("quadradinho aleatorio,posicao_x%d\n", random_x); // diferenca de 1 para cima do conversao_x, conversao_x esta com 1 para cima 
        printf("quadradinho aleatorio, posicao_y %d\n", random_y); // diferenca de 24 do conversao_y

        sleep_ms(20); // um pouco maior para deixar o piscar visível
    }
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    if (current_time - last_time >= 350000)
    {
        last_time = current_time;


        if(gpio == BotaoA)
        {
            pwm_enabled = !pwm_enabled; // Alterna o estado do PWM 
            ativacao_buzzer = !ativacao_buzzer; 

            if (pwm_enabled) {  
                // Ativa o PWM  
                pwm_set_enabled(slice_r, true);
                pwm_set_enabled(slice_b, true);
                pwm_set_enabled(slice_g, true);

            } else {  
                // Desativa o PWM  
                pwm_set_enabled(slice_r, false);
                pwm_set_enabled(slice_b, false);
                pwm_set_enabled(slice_g, false);

            } 

        }
        if (gpio == BotaoB)
        {
            reset_usb_boot(0,0);
            
        }
    }

}

void quadradinho_aleatorio(int* x, int* y) {  
    *x = rand() % (120 - 8); // Novos valores aleatórios  
    *y = rand() % (55 - 3);  // Novos valores aleatórios  
} 

void play_note(int frequency, int duration, int pin) // funcao ativacao dos buzzers
{
    if (frequency == 0) {
        sleep_ms(duration);  // Pausa (silêncio)
        return;
    }

    int delay = 1000000 / frequency / 2; // Meio ciclo da frequência
    int cycles = (frequency * duration) / 1000;

    for (int i = 0; i < cycles; i++) {
        gpio_put(pin, 1);
        sleep_us(delay);
        gpio_put(pin, 0);
        sleep_us(delay);
    }
}

void alerta_de_colisao() 
{  // ativa o buzzer quando ha a colisao dos quadrados
    if(ativacao_buzzer){
        play_note(440, 50, buzzer1); // Oscilação do som
    } 
}