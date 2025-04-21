// ---------------- Bibliotecas - Início ----------------

// Biblioteca padrão de entrada e saída do C (Foi usada para debugging)
#include <stdio.h>

// Bibliotecas do pico SDK de mais alto nível
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

// Bibliotecas do pico SDK de hardware
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "inc/ssd1306.h" // Header para controle do display OLED

#include "ws2812.pio.h"  // Header para controle dos LEDs WS2812

// ---------------- Bibliotecas - Fim ----------------



// ---------------- Definições - Início ----------------

// Configurações do I2C para comunicação com o display OLED
#define I2C_PORT i2c1 // Porta I2C
#define I2C_SDA 14    // Pino de dados
#define I2C_SCL 15    // Pino de clock
#define ADDRESS 0x3C  // Endereço do display

// Definições da matriz de LEDs
#define LED_COUNT 25  // Número total de LEDs na matriz
#define MATRIX_PIN 7  // Pino da matriz de LEDs
struct pixel_t {   // Estrutura para armazenar as cores de um LED WS2812
  uint8_t G, R, B; // Componentes de cor (verde, vermelho e azul)
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;
npLED_t leds[LED_COUNT];
PIO np_pio; // Instância do PIO
uint sm;    // State machine para controle dos LEDs

//  Configuração do LED
#define GREEN_LED 11 // Pino do LED verde

// Configuração do joystick
#define JSK_SEL 22 // Pino do botão do joystick
#define JSK_Y 26   // Pino do eixo Y do joystick
#define JSK_X 27   // Pino do eixo X do joystick

// Configuração dos botões
#define BUTTON_A 5 // Pino do botão A
#define BUTTON_B 6 // Pino do botão B

// Configuração dos buzzers
#define WRAP_VALUE 4095 // Valor do WRAP
#define DIV_VALUE 1.0   // Valor do divisor de clock
#define BUZZER_A 21 // Pino do buzzer A
#define BUZZER_B 10 // Pino do buzzer B

// ---------------- Definições - Fim ----------------



// ---------------- Variáveis - Início ----------------

// Variáveis para controle de interrupções e bebug
static volatile uint32_t last_time = 0; // Armazena o último tempo registrado nas interrupções dos botões
static volatile uint32_t debug_time = 0; // Armazena o último tempo registrado no envio das mensagens de debug

static volatile bool calibration_flag = false; // Flag para calibrar o joystick
static volatile bool strenght_flag = true; // Flag para desativar a detecção de "força" do eixo na matriz de LEDs
static volatile bool axis_flag = true; // Flag mudar o eixo monitorado do joystick
static volatile bool change_flag = false; // Flag para indicar a mudança de estado em flags
static volatile bool y_edge_flag = false; // Flag para indicar que a borda está sendo tocada no eixo y
static volatile bool x_edge_flag = false; // Flag para indicar que a borda está sendo tocada no eixo x

// Variáveis para o joystick
static volatile uint16_t y_high=4095, y_low=0, y_middle_high=2047, y_middle_low=2047; // Limites do eixo Y (Calibração)
static volatile uint16_t x_high=4095, x_low=0, x_middle_high=2047, x_middle_low=2047; // Limites do eixo X (Calibração)
static volatile uint16_t x_value=2047, y_value=2047; // Valores capturados pelo joystick
static volatile int x_scaled = 0, y_scaled = 0;      // Valores do joystick convertidos para os valores do display

// ---------------- Variáveis - Fim ----------------



// ---------------- Inicializações - Início ----------------

// Inicializa o display OLED via I2C
void init_display(ssd1306_t *ssd) {
    i2c_init(I2C_PORT, 400 * 1000); // Inicializa o I2C com frequência de 400 kHz

    // Configura os pinos SDA e SCL como I2C e habilita pull-ups
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa e configura o display
    ssd1306_init(ssd, WIDTH, HEIGHT, false, ADDRESS, I2C_PORT);
    ssd1306_config(ssd);
    ssd1306_send_data(ssd);
    
    // Limpa o display
    ssd1306_fill(ssd, false);
    ssd1306_send_data(ssd);
}

// Inicializa o LED RGB
void init_rgb() {
    // Configura o pino do LED verde como saída digital
    gpio_init(GREEN_LED);
    gpio_set_dir(GREEN_LED, GPIO_OUT);
    gpio_put(GREEN_LED, 0);
}

// Inicializa os buzzers com PWM
void init_buzzers() {
    uint slice;

    // Configuração do Buzzer A
    gpio_set_function(BUZZER_A, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(BUZZER_A);
    pwm_set_clkdiv(BUZZER_A, 125);
    pwm_set_wrap(BUZZER_A, 3822);
    pwm_set_gpio_level(BUZZER_A, 0);
    pwm_set_enabled(slice, true);

    // Configuração do Buzzer B
    gpio_set_function(BUZZER_B, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(BUZZER_B);
    pwm_set_clkdiv(BUZZER_B, 125);
    pwm_set_wrap(BUZZER_B, 2024);
    pwm_set_gpio_level(BUZZER_B, 0);
    pwm_set_enabled(slice, true);
}

// Inicializa o joystick
void init_joystick() {

    // Inicializa o botão do joystick
    gpio_init(JSK_SEL);
    gpio_set_dir(JSK_SEL, GPIO_IN);
    gpio_pull_up(JSK_SEL);

    // Inicializa o ADC do eixo Y e X e joystick
    adc_init();
    adc_gpio_init(JSK_Y);
    adc_gpio_init(JSK_X);
}

// Inicializa os botões A e B
void init_buttons() {
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
}

// -------- Matriz - Início --------

// Inicializa a máquina PIO para controle da matriz de LEDs
void npInit(uint pin) {

    // Carrega o programa PIO para controle dos LEDs
    uint offset = pio_add_program(pio0, &ws2812_program);
    np_pio = pio0;

    // Obtém uma máquina de estado PIO disponível
    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true);
    }

    // Inicializa a máquina de estado com o WS2812.pio
    ws2812_program_init(np_pio, sm, offset, pin, 800000.f);

    // Limpa o buffer de pixels
    for (uint i = 0; i < LED_COUNT; ++i) {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

// Atribui uma cor RGB a um LED específico na matriz
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

// Limpa todos os LEDs na matriz
void npClear() {
    for (uint i = 0; i < LED_COUNT; ++i) {
        npSetLED(i, 0, 0, 0);
    }
}

// Escreve os dados do buffer para os LEDs
void npWrite() {
    // Escreve cada dado de 8 bits dos pixels em sequência no buffer da máquina PIO
    for (uint i = 0; i < LED_COUNT; ++i) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100); // Espera 100us para o reset
}

// Função para facilitar o desenho na matriz utilizando 3 matrizes cos os valores RGB
void npDraw(uint8_t vetorR[5][5], uint8_t vetorG[5][5], uint8_t vetorB[5][5]) {
  int i, j,idx,col;
    for (i = 0; i < 5; i++) {
        idx = (4 - i) * 5; // Calcula o índice base para a linha
        for (j = 0; j < 5; j++) {
            col = (i % 2 == 0) ? (4 - j) : j; // Inverte a ordem das colunas nas linhas pares
            npSetLED(idx + col, vetorR[i][j], vetorG[i][j], vetorB[i][j]); // Preenche o buffer com os valores das matrizes
        }
    }
}

// -------- Matriz - Fim --------

// ---------------- Inicializações - Fim ----------------



// ---------------- Desenhos - Início ----------------

// -------- Matriz - Início --------

// Função para exibir o símbolo de seta para cima na matriz de LEDs para a calibração
void seta_cima() {
    // Matrizes que representam os LEDs vermelhos, verdes e azuis
    uint8_t vetorG[5][5] = {
        {  0  ,  0  ,  1  ,  0  ,  0  },
        {  0  ,  1  ,  1  ,  1  ,  0  },
        {  1  ,  0  ,  1  ,  0  ,  1  },
        {  0  ,  0  ,  1  ,  0  ,  0  },
        {  0  ,  0  ,  1  ,  0  ,  0  }
    };
      uint8_t vetorRB[5][5] = {
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  }
    };
    npDraw(vetorRB,vetorG,vetorRB); // Carrega os buffers
    npWrite();                      // Escreve na matriz de LEDs
    npClear();                      // Limpa os buffers (não necessário, mas por garantia)
}

// Função para exibir o símbolo de seta para baixo na matriz de LEDs para a calibração
void seta_baixo() {
    // Matrizes que representam os LEDs vermelhos, verdes e azuis
    uint8_t vetorG[5][5] = {
        {  0  ,  0  ,  1  ,  0  ,  0  },
        {  0  ,  0  ,  1  ,  0  ,  0  },
        {  1  ,  0  ,  1  ,  0  ,  1  },
        {  0  ,  1  ,  1  ,  1  ,  0  },
        {  0  ,  0  ,  1  ,  0  ,  0  }
    };
      uint8_t vetorRB[5][5] = {
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  }
    };
    npDraw(vetorRB,vetorG,vetorRB); // Carrega os buffers
    npWrite();                      // Escreve na matriz de LEDs
    npClear();                      // Limpa os buffers (não necessário, mas por garantia)
}

// Função para exibir o símbolo de seta para a esquerda na matriz de LEDs para a calibração
void seta_esquerda() {
    // Matrizes que representam os LEDs vermelhos, verdes e azuis
    uint8_t vetorG[5][5] = {
        {  0  ,  0  ,  1  ,  0  ,  0  },
        {  0  ,  1  ,  0  ,  0  ,  0  },
        {  1  ,  1  ,  1  ,  1  ,  1  },
        {  0  ,  1  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  1  ,  0  ,  0  }
    };
      uint8_t vetorRB[5][5] = {
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  }
    };
    npDraw(vetorRB,vetorG,vetorRB); // Carrega os buffers
    npWrite();                      // Escreve na matriz de LEDs
    npClear();                      // Limpa os buffers (não necessário, mas por garantia)
}

// Função para exibir o símbolo de seta para a direita na matriz de LEDs para a calibração
void seta_direita() {
    // Matrizes que representam os LEDs vermelhos, verdes e azuis
    uint8_t vetorG[5][5] = {
        {  0  ,  0  ,  1  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  1  ,  0  },
        {  1  ,  1  ,  1  ,  1  ,  1  },
        {  0  ,  0  ,  0  ,  1  ,  0  },
        {  0  ,  0  ,  1  ,  0  ,  0  }
    };
      uint8_t vetorRB[5][5] = {
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  }
    };
    npDraw(vetorRB,vetorG,vetorRB); // Carrega os buffers
    npWrite();                      // Escreve na matriz de LEDs
    npClear();                      // Limpa os buffers (não necessário, mas por garantia)
}

// Função para exibir o símbolo que representa o meio na matriz de LEDs para a calibração
void meio() {
    // Matrizes que representam os LEDs vermelhos, verdes e azuis
    uint8_t vetorG[5][5] = {
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  1  ,  1  ,  1  ,  0  },
        {  0  ,  1  ,  0  ,  1  ,  0  },
        {  0  ,  1  ,  1  ,  1  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  }
    };
      uint8_t vetorRB[5][5] = {
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  },
        {  0  ,  0  ,  0  ,  0  ,  0  }
    };
    npDraw(vetorRB,vetorG,vetorRB); // Carrega os buffers
    npWrite();                      // Escreve na matriz de LEDs
    npClear();                      // Limpa os buffers (não necessário, mas por garantia)
}

// Função para ligar de 0 a 25 LEDs na matriz com base no valor lido respeitando os limites passados
int jsk_strength(uint16_t value, uint16_t min, uint16_t max) {
    int idx, leds_acesos;
    float fracao;

    // Garante que o valor esteja no intervalo
    if (value < min) {
        value = min;
    }
    if (value > max) {
        value = max;
    }

    // Calcula a fração do valor no intervalo
    fracao = (float)(value - min) / (max - min);
    
    // Calcula quantos LEDs devem estar acesos (0 a 25)
    leds_acesos = (int)(fracao * LED_COUNT + (float)0.5); // Arredondamento para melhor representação

    for(idx=0;idx<LED_COUNT;idx++){
        if(idx<leds_acesos) {
            npSetLED(idx,0,0,1);
        }
        else {
            npSetLED(idx,0,0,0);
        }
    }
    npWrite();

    return leds_acesos;
}

// -------- Matriz - Fim --------

// ---------------- Desenhos - Fim ----------------



// ---------------- Funções - Início ----------------

// Leitura do eixo y do joystick
uint16_t read_y() {
    adc_select_input(0);
    return adc_read();
}

// Leitura do eixo x do joystick
uint16_t read_x() {
    adc_select_input(1);
    return adc_read();
}

// Função para converter escalas
int scale(int min1, int max1, int min2, int max2,int x1) {
    return ( (((x1-min1)*(max2-min2))/(max1-min1))+min2 );
}

// Função para calibrar o eixo y do joystick
void calibrate_jsk_y_values() {
    uint16_t value, temp;
    int i;
    adc_select_input(0);

    // Indica ao usuário através da matriz de LEDs para colocar o joystick para cima e espera 2 segundos
    seta_cima();
    sleep_ms(2000);

    // Lê repetidamente o valor do eixo y e guarda o menor valor registrado no topo
    value = 4095;
    for(i=0;i<300;i++) {
        temp = adc_read();
        if(temp < value) {
            value = temp;
        }
        sleep_ms(10);
    }
    y_high = value;

    // Indica ao usuário através da matriz de LEDs para colocar o joystick para o meio e espera 2 segundos
    meio();
    sleep_ms(2000);

    // Lê repetidamente o valor do eixo y e guarda o maior valor registrado no meio
    value = 0;
    for(i=0;i<300;i++) {
        temp = adc_read();
        if(temp > value) {
            value = temp;
        }
        sleep_ms(10);
    }
    y_middle_high = value;

    // Indica ao usuário através da matriz de LEDs para colocar o joystick para baixo e espera 2 segundos
    seta_baixo();
    sleep_ms(2000);

    // Lê repetidamente o valor do eixo y e guarda o maior valor registrado na base
    value = 0;
    for(i=0;i<300;i++) {
        temp = adc_read();
        if(temp > value) {
            value = temp;
        }
        sleep_ms(10);
    }
    y_low = value;

    // Indica ao usuário através da matriz de LEDs para colocar o joystick para o meio e espera 2 segundos
    meio();
    sleep_ms(2000);

    // Lê repetidamente o valor do eixo y e guarda o menor valor registrado no meio
    value = 4095;
    for(i=0;i<300;i++) {
        temp = adc_read();
        if(temp < value) {
            value = temp;
        }
        sleep_ms(10);
    }
    y_middle_low = value;
}

// Função para calibrar o eixo x do joystick
void calibrate_jsk_x_values() {
    uint16_t value, temp;
    int i;
    adc_select_input(1);

    // Indica ao usuário através da matriz de LEDs para colocar o joystick para a direita e espera 2 segundos
    seta_direita();
    sleep_ms(2000);

    // Lê repetidamente o valor do eixo x e guarda o menor valor registrado na direita
    value = 4095;
    for(i=0;i<300;i++) {
        temp = adc_read();
        if(temp < value) {
            value = temp;
        }
        sleep_ms(10);
    }
    x_high = value;

    // Indica ao usuário através da matriz de LEDs para colocar o joystick para o meio e espera 2 segundos
    meio();
    sleep_ms(2000);

    // Lê repetidamente o valor do eixo x e guarda o maior valor registrado no meio
    value = 0;
    for(i=0;i<300;i++) {
        temp = adc_read();
        if(temp > value) {
            value = temp;
        }
        sleep_ms(10);
    }
    x_middle_high = value;

    // Indica ao usuário através da matriz de LEDs para colocar o joystick para a esquerda e espera 2 segundos
    seta_esquerda();
    sleep_ms(2000);

    // Lê repetidamente o valor do eixo x e guarda o maior valor registrado na esquerda
    value = 0;
    for(i=0;i<300;i++) {
        temp = adc_read();
        if(temp > value) {
            value = temp;
        }
        sleep_ms(10);
    }
    x_low = value;

    // Indica ao usuário através da matriz de LEDs para colocar o joystick para o meio e espera 2 segundos
    meio();
    sleep_ms(2000);

    // Lê repetidamente o valor do eixo x e guarda o menor valor registrado no meio
    value = 4095;
    for(i=0;i<300;i++) {
        temp = adc_read();
        if(temp < value) {
            value = temp;
        }
        sleep_ms(10);
    }
    x_middle_low = value;
}

// -------- Joystick - Fim --------



// -------- Buzzers - Início --------

// Emite um som alternando entre dois buzzers por um tempo determinado
void beep(uint tempo) {
    // Ativa o BUZZER A
    pwm_set_gpio_level(BUZZER_A, 1911);
    sleep_ms(tempo/4);

    // Desativa o BUZZER A e ativa o BUZZER B
    pwm_set_gpio_level(BUZZER_A, 0);
    pwm_set_gpio_level(BUZZER_B, 1012);
    sleep_ms(tempo/4);

    // Ativa o BUZZER A e desativa o BUZZER B
    pwm_set_gpio_level(BUZZER_B, 0);
    pwm_set_gpio_level(BUZZER_A, 1911);
    sleep_ms(tempo/4);

    // Desativa o BUZZER A e ativa o BUZZER B
    pwm_set_gpio_level(BUZZER_A, 0);
    pwm_set_gpio_level(BUZZER_B, 1012);
    sleep_ms(tempo/4);

    // Desativa o BUZZER B
    pwm_set_gpio_level(BUZZER_B, 0);
}

// -------- Buzzers - Fim --------



// -------- Callback - Início --------

// Função de callback para tratar as interrupções dos botões
void gpio_irq_callback(uint gpio, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time()); // Obtém o tempo atual em ms

    // Debounce de 200 ms
    if( (current_time - last_time) > 200 ) {
        last_time = current_time;

        // Alterna a flag da matriz de LEDs quando o botão A é pressionado
        if(gpio == BUTTON_A){
            strenght_flag = !strenght_flag;
            change_flag = true;
        }

        // Entra no modo de calibração ao pressionar o botão B
        if(gpio == BUTTON_B){
            calibration_flag = 1;

            // Desabilita interrupções temporariamente para evitar bugs
            gpio_set_irq_enabled(BUTTON_A, GPIO_IRQ_EDGE_FALL, false);
            gpio_set_irq_enabled(BUTTON_B, GPIO_IRQ_EDGE_FALL, false);
            gpio_set_irq_enabled(JSK_SEL, GPIO_IRQ_EDGE_FALL, false);
        }

        // Alterna o eixo monitorado (X ou Y) ao pressionar o botão do joystick
        if(gpio == JSK_SEL){
            axis_flag = !axis_flag;
        }

    }
}

// -------- Callback - Fim --------



// ---------------- Main - Início ----------------

int main() {
    ssd1306_t ssd;         // Estrutura que representa o display OLED
    uint32_t current_time; // Variável para debug no terminal
    int leds;              // Quantidade de LEDs acesos na matriz

    stdio_init_all(); // Inicializa as entradas e saídas padrões

    init_display(&ssd); // Inicializa o display OLED
    ssd1306_rect(&ssd, 0, 0, 128, 64, true, false); // Desenha a borda externa
    ssd1306_rect(&ssd, 2, 2, 124, 60, true, false); // Desenha a borda interna

    npInit(MATRIX_PIN); // Inicializa e limpa a matriz de LEDs
    npClear();
    npWrite();

    init_joystick(); // Inicializa o joystick
    init_rgb();      // Inicializa o LED RGB
    init_buttons();  // Inicializa os botões A e B
    init_buzzers();  // Inicializa os buzzers

    // Configura as interrupções para os botões
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
    gpio_set_irq_enabled_with_callback(JSK_SEL, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);

    while (true) {
        sleep_ms(20); // Pequena pausa para evitar uso excessivo da CPU (simulação no Wokwi)

        // Entra no modo de calibração caso a flag seja habilitada
        if(calibration_flag) {
            beep(256);                // Sinal sonoro de início 
            calibrate_jsk_x_values(); // Calibra o eixo X do joystick
            calibrate_jsk_y_values(); // Calibra o eixo Y do joystick

            // Mostra os valores calibrados no terminal
            printf("x_min: %d / x_middle_low: %d / x_middle_high: %d / x_max: %d\ny_low: %d / y_middle_low: %d / y_middle_high: %d / y_high: %d\n",x_low,x_middle_low,x_middle_high,x_high,y_low,y_middle_low,y_middle_high,y_high);

            // Limpa a matriz e finaliza a calibração
            npClear();
            npWrite();
            beep(256); // Sinal sonoro de fim

            // Reativa interrupções após calibração
            gpio_set_irq_enabled(BUTTON_A, GPIO_IRQ_EDGE_FALL, true);
            gpio_set_irq_enabled(BUTTON_B, GPIO_IRQ_EDGE_FALL, true);
            gpio_set_irq_enabled(JSK_SEL, GPIO_IRQ_EDGE_FALL, true);

            // Reinicia a flag
            calibration_flag = 0;
        }

        // Lê os valores analógicos do joystick
        y_value = read_y();
        x_value = read_x();
        // Escala os valores lidos para as coordenadas do display OLED
        y_scaled = scale(y_low,y_high,53,3,y_value);
        x_scaled = scale(x_low,x_high,3,117,x_value);
        // Limita os valores escalados dentro dos intervalos desejados
        if(y_scaled > 53) {
            y_scaled = 53;
        }else
        if(y_scaled < 3) {
            y_scaled = 3;
        }
        if(x_scaled > 117) {
            x_scaled = 117;
        }else
        if(x_scaled < 3) {
            x_scaled = 3;
        }

        // Desenha o quadrado com base na posição do joystick
        ssd1306_rect(&ssd,y_scaled,x_scaled,8,8,true,true);
        ssd1306_send_data(&ssd);
        // Limpa o quadrado na posição anterior
        ssd1306_rect(&ssd,y_scaled,x_scaled,8,8,false,true);

        // Verifica se o quadrado está tocando as bordas do display
        if(y_scaled == 3 || y_scaled == 53) {
            y_edge_flag = true;
            pwm_set_gpio_level(BUZZER_A, 1911); // Buzzer A ativo para o eixo Y
        }
        if(x_scaled == 3 || x_scaled == 117) {
            x_edge_flag = true;
            pwm_set_gpio_level(BUZZER_B, 1012); // Buzzer B ativo para o eixo X
        }
        // Se tocou qualquer borda, acende LED verde e reinicia flags
        if(y_edge_flag || x_edge_flag) {
            gpio_put(GREEN_LED,1);
            y_edge_flag = false;
            x_edge_flag = false;
        }
        else {
            // Desliga buzzers e o LED se não está nas bordas
            pwm_set_gpio_level(BUZZER_A, 0);
            pwm_set_gpio_level(BUZZER_B, 0);
            gpio_put(GREEN_LED,0);
        }

        // Reseta o funcionamento da matriz de LEDs cajo haja mudança de estado na flag da matriz
        if(change_flag) {
            npClear();
            npWrite();
            leds = 0;
            change_flag = false;
        }

        // Acende LEDs na matriz com base na "força" do eixo do joystick selecionado
        if(strenght_flag) {
            if(axis_flag) {
                leds = jsk_strength(y_value,y_low,y_high);
            }
            else {
                leds = jsk_strength(x_value,x_low,x_high);
            }
        }

        // Debug a cada 500 ms: mostra valores lidos e posição
        current_time = to_ms_since_boot(get_absolute_time());
        if( (current_time - debug_time) > 500 ) {
            debug_time = current_time;
            printf("Eixo X: %d / Eixo Y: %d / ",x_value,y_value);
            printf("Posição X: %d / Posição Y: %d / ",x_scaled,y_scaled);
            printf("LEDs acesos: %d\n", leds);
        }

    }
}

// ---------------- Main - Fim ----------------