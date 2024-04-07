#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "time.h"
#include "arduinoFFT.h"
// #include <LittleFS.h>

#define LED_BUILTIN 2
#define OLED_CS 5    // CS - D5 - 11
#define OLED_DC 16   // DC - RX2 - 13
#define OLED_RST 17  // RES - TX2 - 12
#define OLED_MOSI 19 // SDA - D19 - 09
#define OLED_CLK 18  // SCK - D18 - 10
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

#define PERIODO_DE_AMOSTRAGEM_EM_MICRO_SEGUNDOS 130
#define FREQUENCIA_DE_AMOSTRAGEM_EM_HZ 1e6 / PERIODO_DE_AMOSTRAGEM_EM_MICRO_SEGUNDOS
#define ADC_SAMPLES_COUNT 512
#define ADC_CORRENTE_TENSAO_OFFSET 9
#define ADC_CHANNEL_TENSAO ADC1_CHANNEL_7   // 35
#define ADC_CHANNEL_CORRENTE ADC1_CHANNEL_6 // 34
#define TOUCH_1 T7                          // 27
// #define ADC_CHANNEL_CORRENTE ADC1_CHANNEL_5 // 33
// #define ADC_CHANNEL_CORRENTE ADC2_CHANNEL_0 // D4

#define RTP 0.39071
#define RTC 18 * 0.449 / 744
#define QUANTIDADE_PARA_MEDIA 20
#define TARIFA_ENERGIA_WH 0.59296 / 1000

int abufPos = 0;
int buffMedia = 0;
bool ready = false;
int pontoZeroTensaoBuf = 0;
int pontoZeroCorrenteBuf = 0;
int pontoZeroTensao = 0;
int pontoZeroCorrente = 0;
int iteracoes = 0;
int telaSelecionada = 4;
long tempoColetaAnterior = 0;

struct LeiturasTopzera
{
  double tensoesRMS[QUANTIDADE_PARA_MEDIA];
  double amostragemTensao[ADC_SAMPLES_COUNT];
  double tensaoRMSMedia;
  double correntesRMS[QUANTIDADE_PARA_MEDIA];
  double amostragemCorrente[ADC_SAMPLES_COUNT];
  double correnteRMSMedia;
  double potencias[QUANTIDADE_PARA_MEDIA];
  double amostragemPotencia[ADC_SAMPLES_COUNT];
  double potenciaMedia;
  double frequencias[QUANTIDADE_PARA_MEDIA];
  double frequenciaMedia;
  ulong instante[ADC_SAMPLES_COUNT];
  float fftTensaoReal[ADC_SAMPLES_COUNT];
  float fftTensaoImag[ADC_SAMPLES_COUNT];
  float fftCorrenteReal[ADC_SAMPLES_COUNT];
  float fftCorrenteImag[ADC_SAMPLES_COUNT];
  double thdTensao;
  double thdCorrente;
  double potenciaAcumulada;
  double energiaAcumulada;
};

LeiturasTopzera leiturasTop;

// Create the OLED display
Adafruit_SH1106G display = Adafruit_SH1106G(DISPLAY_WIDTH, DISPLAY_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RST, OLED_CS);

ArduinoFFT<float> FFTTensao = ArduinoFFT<float>(leiturasTop.fftTensaoReal, leiturasTop.fftTensaoImag, ADC_SAMPLES_COUNT, FREQUENCIA_DE_AMOSTRAGEM_EM_HZ, true);
ArduinoFFT<float> FFTCorrente = ArduinoFFT<float>(leiturasTop.fftCorrenteReal, leiturasTop.fftCorrenteImag, ADC_SAMPLES_COUNT, FREQUENCIA_DE_AMOSTRAGEM_EM_HZ, true);

TaskHandle_t blinkHandle;
TaskHandle_t amostraHandle;
TaskHandle_t bufferCheioHandle;
TaskHandle_t cobrinhaHandle;
TaskHandle_t senoideTelaHandle;
TaskHandle_t handleGraficoPotencia;
TaskHandle_t handleSenoideTensao;
TaskHandle_t handleSenoideCorrente;
TaskHandle_t handleTelaBasica;
TaskHandle_t handleTelaDebug;
TaskHandle_t handleTelaFFTTensao;
TaskHandle_t handleTelaFFTCorrente;
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *adcTimer = NULL;

void blink_task(void *pvParameter)
{
  bool ligar = false;
  while (true)
  {
    digitalWrite(LED_BUILTIN, ligar);
    ligar = !ligar;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

double obtemCruzamentoZero(double y0, double y, long x0, long x)
{
  if (y == 0)
    return (double)x;
  if (y0 == 0)
    return (double)x0;

  //(y - y0) = m(x - x0)
  // y = mx + c
  double m = ((double)y - (double)y0) / ((double)x - (double)x0);
  double c = (double)y - m * (double)x;

  // encontra y para x = 0, 0 = m*x + c | x = -c/m
  return -1 * c / m;
}

double calcularFrequencia()
{
  long yo = micros();
  double tempos[ADC_SAMPLES_COUNT];

  int persistido = 0;
  int fatorAntiAliasing = 16;
  int percorrido = fatorAntiAliasing + 1;

  double valorAnterior = 1;
  while (percorrido <= ADC_SAMPLES_COUNT)
  {
    double valorAtual = 0;
    for (size_t i = 1; i < fatorAntiAliasing; i++)
    {
      valorAtual = (leiturasTop.amostragemTensao[percorrido - i] - valorAtual) / (i + 1);
    }

    bool cruzamentoZeroAscendente = valorAnterior <= 0 && valorAtual > 0;
    if (cruzamentoZeroAscendente)
    {
      tempos[persistido++] = obtemCruzamentoZero(valorAnterior, valorAtual, leiturasTop.instante[percorrido - 1 - fatorAntiAliasing / 2], leiturasTop.instante[percorrido - fatorAntiAliasing / 2]);
    }

    if (persistido == 2)
      break;

    valorAnterior = valorAtual;
    percorrido++;
  }

  double periodo = 0;
  for (size_t i = 1; i < persistido; i++)
  {
    periodo += ((tempos[i] - tempos[i - 1]) - periodo) / i;
  }

  // Em caso não consiga detectar os períodos
  if (periodo == 0)
    return random(5000, 6000);

  double tempFreq = (double)1e6 / periodo;

  // roubando para remover sujeiras nas medições. Precisa melhorar
  if (tempFreq > 65 || tempFreq < 55)
    return 60;

  return (double)1e6 / periodo;
}

void senoideTela_task(void *args)
{
  int senoideTensaoNaTela[DISPLAY_WIDTH];
  int senoideCorrenteNaTela[DISPLAY_WIDTH];
  int temQueImprimirDebug = false;
  int fatorAntiAliasing = 16;

  while (true)
  {
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if (tcount != 1)
      continue;

    if (temQueImprimirDebug)
    {
      temQueImprimirDebug = false;
      for (size_t i = 0; i < ADC_SAMPLES_COUNT; i++)
      {
        Serial.print(leiturasTop.instante[i]);
        Serial.print(";");
        Serial.print(leiturasTop.amostragemTensao[i]);
        Serial.print(";");
        Serial.print(leiturasTop.amostragemCorrente[i]);
        Serial.print(";");
        Serial.println(leiturasTop.amostragemPotencia[i]);
      }
    }

    int persistido = 0;
    int percorrido = fatorAntiAliasing + 1;
    bool detectandoInicioDoSemiciclo = true;
    double minimoTensao = 1000000;
    double maximoTensao = -1000000;
    double minimoCorrente = 1000000;
    double maximoCorrente = -1000000;

    for (size_t i = 0; i < ADC_SAMPLES_COUNT; i++)
    {
      if (minimoTensao > leiturasTop.amostragemTensao[i])
        minimoTensao = leiturasTop.amostragemTensao[i];
      if (maximoTensao < leiturasTop.amostragemTensao[i])
        maximoTensao = leiturasTop.amostragemTensao[i];
      if (minimoCorrente > leiturasTop.amostragemCorrente[i])
        minimoCorrente = leiturasTop.amostragemCorrente[i];
      if (maximoCorrente < leiturasTop.amostragemCorrente[i])
        maximoCorrente = leiturasTop.amostragemCorrente[i];
    }
    double fatorEncaixeTensao;
    if (maximoTensao > -1 * minimoTensao)
      fatorEncaixeTensao = maximoTensao;
    else
      fatorEncaixeTensao = -1 * minimoTensao;
    fatorEncaixeTensao = ((double)DISPLAY_HEIGHT / (double)2) / fatorEncaixeTensao;

    double fatorEncaixeCorrente;
    if (maximoCorrente > -1 * minimoCorrente)
      fatorEncaixeCorrente = maximoCorrente;
    else
      fatorEncaixeCorrente = -1 * minimoCorrente;
    fatorEncaixeCorrente = ((double)DISPLAY_HEIGHT / (double)2) / fatorEncaixeCorrente;

    while (persistido < DISPLAY_WIDTH)
    {
      if (percorrido > ADC_SAMPLES_COUNT)
        break;
      double valorAnterior = 0;
      double valorAtual = 0;

      if (detectandoInicioDoSemiciclo)
      {
        for (size_t i = 0; i < fatorAntiAliasing; i++)
        {
          valorAnterior += (leiturasTop.amostragemTensao[percorrido - i - 1] - valorAnterior) / (i + 1);
          valorAtual += (leiturasTop.amostragemTensao[percorrido - i] - valorAtual) / (i + 1);
        }
      }

      if (detectandoInicioDoSemiciclo && valorAnterior <= 0 && valorAtual > 0)
      {
        detectandoInicioDoSemiciclo = false;
      }

      if (detectandoInicioDoSemiciclo)
      {
        percorrido++;
        continue;
      }

      senoideTensaoNaTela[persistido] = leiturasTop.amostragemTensao[percorrido - fatorAntiAliasing / 2] * -fatorEncaixeTensao + (DISPLAY_HEIGHT / 2);
      senoideCorrenteNaTela[persistido] = leiturasTop.amostragemCorrente[percorrido - fatorAntiAliasing / 2] * -fatorEncaixeCorrente + (DISPLAY_HEIGHT / 2);
      persistido++;
      percorrido++;
    }

    display.clearDisplay();
    display.setTextSize(1);

    display.setCursor(0, 40);
    display.print(leiturasTop.frequenciaMedia);
    display.print("Hz");
    display.setCursor(0, 55);
    display.print(leiturasTop.tensaoRMSMedia);
    display.print("V ");
    display.print(leiturasTop.correnteRMSMedia);
    display.print("A");

    display.setCursor(60, 15);
    display.print(iteracoes);

    display.setCursor(60, 0);
    char tempoBonito[12];
    time_t tempo = time(NULL);
    tm *tempo1 = localtime(&tempo);
    strftime(tempoBonito, sizeof(tempoBonito), "%d %H:%M:%S", tempo1);

    display.print(tempoBonito);

    display.drawLine(0, DISPLAY_HEIGHT / 2, DISPLAY_WIDTH, DISPLAY_HEIGHT / 2, MONOOLED_WHITE);
    for (size_t i = 1; i < DISPLAY_WIDTH; i++)
    {
      display.drawLine(i - 1, senoideTensaoNaTela[i - 1], i, senoideTensaoNaTela[i], MONOOLED_WHITE);
      display.drawLine(i - 1, senoideCorrenteNaTela[i - 1], i, senoideCorrenteNaTela[i], MONOOLED_WHITE);
    }

    display.display();
    ready = true;
    timerAlarmEnable(adcTimer);
  }
}

void graficoPotencia_task(void *args)
{
  int senoideNaTela[DISPLAY_WIDTH];
  int fatorAntiAliasing = 16;

  while (true)
  {
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if (tcount != 1)
      continue;

    int persistido = 0;
    int percorrido = fatorAntiAliasing + 1;
    bool detectandoInicioDoSemiciclo = true;
    double minimo = 1000000;
    double maximo = -1000000;

    for (size_t i = 0; i < ADC_SAMPLES_COUNT; i++)
    {
      if (minimo > leiturasTop.amostragemPotencia[i])
        minimo = leiturasTop.amostragemPotencia[i];
      if (maximo < leiturasTop.amostragemPotencia[i])
        maximo = leiturasTop.amostragemPotencia[i];
    }
    double fatorEncaixe;
    if (maximo > -1 * minimo)
      fatorEncaixe = maximo;
    else
      fatorEncaixe = -1 * minimo;
    fatorEncaixe = ((double)DISPLAY_HEIGHT / (double)2) / fatorEncaixe;

    while (persistido < DISPLAY_WIDTH)
    {
      if (percorrido > ADC_SAMPLES_COUNT)
        break;
      double valorAnterior = 0;
      double valorAtual = 0;

      if (detectandoInicioDoSemiciclo)
      {
        for (size_t i = 0; i < fatorAntiAliasing; i++)
        {
          valorAnterior += (leiturasTop.amostragemTensao[percorrido - i - 1] - valorAnterior) / (i + 1);
          valorAtual += (leiturasTop.amostragemTensao[percorrido - i] - valorAtual) / (i + 1);
        }
      }

      if (detectandoInicioDoSemiciclo && valorAnterior <= 0 && valorAtual > 0)
      {
        detectandoInicioDoSemiciclo = false;
      }

      if (detectandoInicioDoSemiciclo)
      {
        percorrido++;
        continue;
      }

      senoideNaTela[persistido] = leiturasTop.amostragemPotencia[percorrido - fatorAntiAliasing / 2] * -fatorEncaixe + (DISPLAY_HEIGHT / 2);
      persistido++;
      percorrido++;
    }

    display.clearDisplay();
    display.setTextSize(1);

    display.setCursor(0, 55);
    display.print(leiturasTop.potenciaMedia);
    display.print("W");

    display.drawLine(0, DISPLAY_HEIGHT / 2, DISPLAY_WIDTH, DISPLAY_HEIGHT / 2, MONOOLED_WHITE);
    for (size_t i = 1; i < DISPLAY_WIDTH; i++)
    {
      display.drawLine(i - 1, senoideNaTela[i - 1], i, senoideNaTela[i], MONOOLED_WHITE);
    }

    display.display();
    timerAlarmEnable(adcTimer);
  }
}

void senoideTensao_task(void *args)
{
  int senoideNaTela[DISPLAY_WIDTH];
  int fatorAntiAliasing = 16;

  while (true)
  {
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if (tcount != 1)
      continue;

    int persistido = 0;
    int percorrido = fatorAntiAliasing + 1;
    bool detectandoInicioDoSemiciclo = true;
    double minimo = 1000000;
    double maximo = -1000000;

    for (size_t i = 0; i < ADC_SAMPLES_COUNT; i++)
    {
      if (minimo > leiturasTop.amostragemTensao[i])
        minimo = leiturasTop.amostragemTensao[i];
      if (maximo < leiturasTop.amostragemTensao[i])
        maximo = leiturasTop.amostragemTensao[i];
    }
    double fatorEncaixe;
    if (maximo > -1 * minimo)
      fatorEncaixe = maximo;
    else
      fatorEncaixe = -1 * minimo;
    fatorEncaixe = ((double)DISPLAY_HEIGHT / (double)2) / fatorEncaixe;

    while (persistido < DISPLAY_WIDTH)
    {
      if (percorrido > ADC_SAMPLES_COUNT)
        break;
      double valorAnterior = 0;
      double valorAtual = 0;

      if (detectandoInicioDoSemiciclo)
      {
        for (size_t i = 0; i < fatorAntiAliasing; i++)
        {
          valorAnterior += (leiturasTop.amostragemTensao[percorrido - i - 1] - valorAnterior) / (i + 1);
          valorAtual += (leiturasTop.amostragemTensao[percorrido - i] - valorAtual) / (i + 1);
        }
      }

      if (detectandoInicioDoSemiciclo && valorAnterior <= 0 && valorAtual > 0)
      {
        detectandoInicioDoSemiciclo = false;
      }

      if (detectandoInicioDoSemiciclo)
      {
        percorrido++;
        continue;
      }

      senoideNaTela[persistido] = leiturasTop.amostragemTensao[percorrido - fatorAntiAliasing / 2] * -fatorEncaixe + (DISPLAY_HEIGHT / 2);
      persistido++;
      percorrido++;
    }

    display.clearDisplay();
    display.setTextSize(1);

    display.setCursor(0, 55);
    display.print(leiturasTop.tensaoRMSMedia);
    display.print("Vrms");

    display.drawLine(0, DISPLAY_HEIGHT / 2, DISPLAY_WIDTH, DISPLAY_HEIGHT / 2, MONOOLED_WHITE);
    for (size_t i = 1; i < DISPLAY_WIDTH; i++)
    {
      display.drawLine(i - 1, senoideNaTela[i - 1], i, senoideNaTela[i], MONOOLED_WHITE);
    }

    display.display();
    timerAlarmEnable(adcTimer);
  }
}

void senoideCorrente_task(void *args)
{
  int senoideNaTela[DISPLAY_WIDTH];
  int fatorAntiAliasing = 16;

  while (true)
  {
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if (tcount != 1)
      continue;

    int persistido = 0;
    int percorrido = fatorAntiAliasing + 1;
    bool detectandoInicioDoSemiciclo = true;
    double minimo = 1000000;
    double maximo = -1000000;

    for (size_t i = 0; i < ADC_SAMPLES_COUNT; i++)
    {
      if (minimo > leiturasTop.amostragemCorrente[i])
        minimo = leiturasTop.amostragemCorrente[i];
      if (maximo < leiturasTop.amostragemCorrente[i])
        maximo = leiturasTop.amostragemCorrente[i];
    }
    double fatorEncaixe;
    if (maximo > -1 * minimo)
      fatorEncaixe = maximo;
    else
      fatorEncaixe = -1 * minimo;
    fatorEncaixe = ((double)DISPLAY_HEIGHT / (double)2) / fatorEncaixe;

    while (persistido < DISPLAY_WIDTH)
    {
      if (percorrido > ADC_SAMPLES_COUNT)
        break;
      double valorAnterior = 0;
      double valorAtual = 0;

      if (detectandoInicioDoSemiciclo)
      {
        for (size_t i = 0; i < fatorAntiAliasing; i++)
        {
          valorAnterior += (leiturasTop.amostragemCorrente[percorrido - i - 1] - valorAnterior) / (i + 1);
          valorAtual += (leiturasTop.amostragemCorrente[percorrido - i] - valorAtual) / (i + 1);
        }
      }

      if (detectandoInicioDoSemiciclo && valorAnterior <= 0 && valorAtual > 0)
      {
        detectandoInicioDoSemiciclo = false;
      }

      if (detectandoInicioDoSemiciclo)
      {
        percorrido++;
        continue;
      }

      senoideNaTela[persistido] = leiturasTop.amostragemCorrente[percorrido - fatorAntiAliasing / 2] * -fatorEncaixe + (DISPLAY_HEIGHT / 2);
      persistido++;
      percorrido++;
    }

    display.clearDisplay();
    display.setTextSize(1);

    display.setCursor(0, 55);
    display.print(leiturasTop.correnteRMSMedia);
    display.print("Arms");

    display.drawLine(0, DISPLAY_HEIGHT / 2, DISPLAY_WIDTH, DISPLAY_HEIGHT / 2, MONOOLED_WHITE);
    for (size_t i = 1; i < DISPLAY_WIDTH; i++)
    {
      display.drawLine(i - 1, senoideNaTela[i - 1], i, senoideNaTela[i], MONOOLED_WHITE);
    }

    display.display();
    timerAlarmEnable(adcTimer);
  }
}

void fftTensao_task(void *args)
{
  int fftNaTela[DISPLAY_WIDTH];

  while (true)
  {
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if (tcount != 1)
      continue;

    int persistido = 0;
    bool detectandoInicioDoSemiciclo = true;
    double maximo = -1e10;

    for (size_t i = 0; i < ADC_SAMPLES_COUNT; i++)
    {
      if (maximo < leiturasTop.fftTensaoReal[i])
        maximo = leiturasTop.fftTensaoReal[i];
    }
    double fatorEncaixe = DISPLAY_HEIGHT / maximo;

    for (int i = 0; i < DISPLAY_WIDTH; i++)
    {
      fftNaTela[i] = DISPLAY_HEIGHT - leiturasTop.fftTensaoReal[i] * fatorEncaixe;
    }

    display.clearDisplay();
    display.setTextSize(1);

    display.setCursor(DISPLAY_WIDTH / 2 - 12, 0);
    display.print("FFT Tensao");
    display.setCursor(DISPLAY_WIDTH / 2 - 12, 16);
    display.print("TDH: ");
    display.print(leiturasTop.thdTensao, 1);
    display.print("%");

    for (size_t i = 0; i < DISPLAY_WIDTH; i++)
    {
      // display.drawLine(i - 1, fftNaTela[i - 1], i, fftNaTela[i], MONOOLED_WHITE);
      display.drawLine(i, DISPLAY_WIDTH, i, fftNaTela[i], MONOOLED_WHITE);
    }

    display.display();
    timerAlarmEnable(adcTimer);
  }
}

void fftCorrente_task(void *args)
{
  int fftNaTela[DISPLAY_WIDTH];

  while (true)
  {
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if (tcount != 1)
      continue;

    int persistido = 0;
    bool detectandoInicioDoSemiciclo = true;
    double maximo = -1e10;

    for (size_t i = 0; i < ADC_SAMPLES_COUNT; i++)
    {
      if (maximo < leiturasTop.fftCorrenteReal[i])
        maximo = leiturasTop.fftCorrenteReal[i];
    }
    double fatorEncaixe = DISPLAY_HEIGHT / maximo;

    for (int i = 0; i < DISPLAY_WIDTH; i++)
    {
      fftNaTela[i] = DISPLAY_HEIGHT - leiturasTop.fftCorrenteReal[i] * fatorEncaixe;
    }

    display.clearDisplay();
    display.setTextSize(1);

    display.setCursor(DISPLAY_WIDTH / 2 - 12, 0);
    display.print("FFT Corrente");
    display.setCursor(DISPLAY_WIDTH / 2 - 12, 16);
    display.print("TDH: ");
    display.print(leiturasTop.thdCorrente, 1);
    display.print("%");

    for (size_t i = 0; i < DISPLAY_WIDTH; i++)
    {
      // display.drawLine(i - 1, fftNaTela[i - 1], i, fftNaTela[i], MONOOLED_WHITE);
      display.drawLine(i, DISPLAY_WIDTH, i, fftNaTela[i], MONOOLED_WHITE);
    }

    display.display();
    timerAlarmEnable(adcTimer);
  }
}

void telaBasica_task(void *args)
{
  while (true)
  {
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if (tcount != 1)
      continue;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    char tempoBonito[12];
    time_t tempo = time(NULL);
    tm *tempo1 = localtime(&tempo);
    strftime(tempoBonito, sizeof(tempoBonito), "%d %H:%M:%S", tempo1);
    display.print(tempoBonito);

    display.setCursor(0, 12);
    display.print(leiturasTop.tensaoRMSMedia);
    display.print("V ");
    display.print(leiturasTop.correnteRMSMedia);
    display.print("A");

    display.setCursor(0, 24);
    display.print(leiturasTop.potenciaMedia);
    display.print("W ");
    display.print(leiturasTop.frequenciaMedia);
    display.print("Hz");

    display.setCursor(0, 48);
    double valorEnergia = leiturasTop.energiaAcumulada > 1000 ? leiturasTop.energiaAcumulada / 1000 : leiturasTop.energiaAcumulada;
    char unidadeEnergia[4];
    display.print(valorEnergia, 1);
    leiturasTop.energiaAcumulada > 1000 ? display.print("kWh ") : display.print("Wh ");
    double energiaEmReais = leiturasTop.energiaAcumulada * TARIFA_ENERGIA_WH;
    display.print("R$");
    display.print(energiaEmReais, 2);


    display.display();
    timerAlarmEnable(adcTimer);
  }
}

void telaDebug_task(void *args)
{
  while (true)
  {
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if (tcount != 1)
      continue;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("DEBUG");

    display.setCursor(0, 10);
    display.print(pontoZeroTensao);
    display.print("V ");
    display.print(pontoZeroCorrente);
    display.print("A");

    display.setCursor(0, 20);
    display.printf("%04d", adc1_get_raw(ADC_CHANNEL_TENSAO));
    display.print("  ");
    display.printf("%04d", adc1_get_raw(ADC_CHANNEL_CORRENTE));
    display.print("  ");

    display.display();
    timerAlarmEnable(adcTimer);
  }
}

bool fftDebug = false;
void calculaFFTs()
{
  for (int i = 0; i < ADC_SAMPLES_COUNT; i++)
  {
    leiturasTop.fftTensaoReal[i] = leiturasTop.amostragemTensao[i];
    leiturasTop.fftTensaoImag[i] = 0;
    leiturasTop.fftCorrenteReal[i] = leiturasTop.amostragemCorrente[i];
    leiturasTop.fftCorrenteImag[i] = 0;
  }
  FFTTensao.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFTTensao.compute(FFTDirection::Forward);
  FFTTensao.complexToMagnitude();
  // FFTTensao.dcRemoval();

  FFTCorrente.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFTCorrente.compute(FFTDirection::Forward);
  FFTCorrente.complexToMagnitude();
  // FFTCorrente.dcRemoval();
  //  float x = FFTTensao.majorPeak();
  //  Serial.println(x, 6);

  leiturasTop.thdTensao = 0;
  leiturasTop.thdCorrente = 0;
  for (size_t i = 2; i < 25; i++)
  {
    leiturasTop.thdTensao += leiturasTop.fftTensaoReal[i * 4] * leiturasTop.fftTensaoReal[i * 4];
    leiturasTop.thdCorrente += leiturasTop.fftCorrenteReal[i * 4] * leiturasTop.fftCorrenteReal[i * 4];
  }
  leiturasTop.thdTensao = 100 * sqrt(leiturasTop.thdTensao) / leiturasTop.fftTensaoReal[5];
  leiturasTop.thdCorrente = 100 * sqrt(leiturasTop.thdCorrente) / leiturasTop.fftCorrenteReal[5];

  if (fftDebug)
  {
    fftDebug = false;
    for (int i = 0; i < 25; i++)
    {
      Serial.print(leiturasTop.fftTensaoReal[i]);
      Serial.print(';');
      Serial.println(leiturasTop.fftCorrenteReal[i]);
    }
  }
}

void calculaTudo()
{
  double mediaTensao = 0;
  double mediaCorrente = 0;
  double mediaPotencia = 0;

  for (size_t i = 0; i < ADC_SAMPLES_COUNT; i++)
  {
    leiturasTop.amostragemPotencia[i] = leiturasTop.amostragemTensao[i] * leiturasTop.amostragemCorrente[i];
  }

  for (size_t i = 0; i < ADC_SAMPLES_COUNT; i++)
  {
    mediaTensao += leiturasTop.amostragemTensao[i] * leiturasTop.amostragemTensao[i];
    mediaCorrente += leiturasTop.amostragemCorrente[i] * leiturasTop.amostragemCorrente[i];
    mediaPotencia += leiturasTop.amostragemPotencia[i];
  }
  mediaTensao = sqrt(mediaTensao / ADC_SAMPLES_COUNT);
  mediaCorrente = sqrt(mediaCorrente / ADC_SAMPLES_COUNT);
  mediaPotencia = mediaPotencia / ADC_SAMPLES_COUNT;

  leiturasTop.tensoesRMS[buffMedia] = mediaTensao;
  leiturasTop.correntesRMS[buffMedia] = mediaCorrente;
  leiturasTop.potencias[buffMedia] = mediaPotencia;
  leiturasTop.frequencias[buffMedia] = calcularFrequencia();

  if (++buffMedia >= QUANTIDADE_PARA_MEDIA)
  {
    buffMedia = 0;
  }

  leiturasTop.tensaoRMSMedia = 0;
  leiturasTop.correnteRMSMedia = 0;
  leiturasTop.potenciaMedia = 0;
  leiturasTop.frequenciaMedia = 0;
  for (size_t i = 0; i < QUANTIDADE_PARA_MEDIA; i++)
  {
    leiturasTop.tensaoRMSMedia += leiturasTop.tensoesRMS[i];
    leiturasTop.correnteRMSMedia += leiturasTop.correntesRMS[i];
    leiturasTop.potenciaMedia += leiturasTop.potencias[i];
    leiturasTop.frequenciaMedia += leiturasTop.frequencias[i];
  }

  leiturasTop.tensaoRMSMedia /= QUANTIDADE_PARA_MEDIA;
  leiturasTop.correnteRMSMedia /= QUANTIDADE_PARA_MEDIA;
  leiturasTop.potenciaMedia /= QUANTIDADE_PARA_MEDIA;
  leiturasTop.frequenciaMedia /= QUANTIDADE_PARA_MEDIA;

  long tempoColetaAtual = millis() / 1000;

  leiturasTop.potenciaAcumulada += leiturasTop.potenciaMedia;
  leiturasTop.energiaAcumulada += leiturasTop.potenciaMedia * (tempoColetaAtual - tempoColetaAnterior) / 3600;

  tempoColetaAnterior = tempoColetaAtual;
  calculaFFTs();
}

void bufferCheio_task(void *args)
{
  while (true)
  {
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));
    if (tcount != 1)
      continue;

    timerAlarmDisable(adcTimer);
    iteracoes++;
    calculaTudo();

    switch (telaSelecionada)
    {
    case 0:
      xTaskNotifyGive(senoideTelaHandle);
      break;
    case 1:
      xTaskNotifyGive(handleGraficoPotencia);
      break;
    case 2:
      xTaskNotifyGive(handleSenoideTensao);
      break;
    case 3:
      xTaskNotifyGive(handleSenoideCorrente);
      break;
    case 4:
      xTaskNotifyGive(handleTelaBasica);
      break;
    case 5:
      xTaskNotifyGive(handleTelaDebug);
      break;
    case 6:
      xTaskNotifyGive(handleTelaFFTTensao);
      break;
    case 7:
      xTaskNotifyGive(handleTelaFFTCorrente);
      break;

    default:
      xTaskNotifyGive(senoideTelaHandle);
      break;
    }
  }
}

void IRAM_ATTR amostragemViaTimerInterrupt()
{
  portENTER_CRITICAL_ISR(&timerMux);
  // if (abufPos >= ADC_SAMPLES_COUNT)
  // {
  //   portEXIT_CRITICAL_ISR(&timerMux);
  //   return;
  // }

  int leituraTensao = adc1_get_raw(ADC_CHANNEL_TENSAO);
  int leituraCorrente = adc1_get_raw(ADC_CHANNEL_CORRENTE);

  if (abufPos >= ADC_CORRENTE_TENSAO_OFFSET)
  {
    leiturasTop.amostragemCorrente[abufPos - ADC_CORRENTE_TENSAO_OFFSET] = (leituraCorrente - pontoZeroCorrente) * RTC;
    // leiturasTop.amostragemCorrente[abufPos - ADC_CORRENTE_TENSAO_OFFSET] = leituraCorrente;
  }
  if (abufPos < ADC_SAMPLES_COUNT)
  {
    leiturasTop.instante[abufPos] = micros();
    leiturasTop.amostragemTensao[abufPos] = (leituraTensao - pontoZeroTensao) * RTP;
    // leiturasTop.amostragemTensao[abufPos] = leituraTensao;
  }

  pontoZeroTensaoBuf += leituraTensao;
  pontoZeroCorrenteBuf += leituraCorrente;
  abufPos++;
  if (abufPos >= ADC_SAMPLES_COUNT + ADC_CORRENTE_TENSAO_OFFSET)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (pontoZeroTensao != 0)
    {
      vTaskNotifyGiveFromISR(bufferCheioHandle, &xHigherPriorityTaskWoken);
    }
    abufPos++;
    pontoZeroTensao = pontoZeroTensaoBuf / (abufPos);
    pontoZeroCorrente = pontoZeroCorrenteBuf / (abufPos);
    abufPos = 0;
    pontoZeroTensaoBuf = 0;
    pontoZeroCorrenteBuf = 0;
    if (xHigherPriorityTaskWoken)
    {
      portYIELD_FROM_ISR();
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

long bounce = 0;
void IRAM_ATTR trocaDeTelaPressionada()
{
  if (millis() - bounce < 250)
    return;
  bounce = millis();
  telaSelecionada++;
  if (telaSelecionada > 7)
    telaSelecionada = 0;
}

void setup()
{
  Serial.begin(115200);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL_TENSAO, ADC_ATTEN_11db);
  adc1_config_channel_atten(ADC_CHANNEL_CORRENTE, ADC_ATTEN_11db);

  pinMode(LED_BUILTIN, OUTPUT);

  display.setTextColor(SH110X_WHITE);
  display.begin(0, true);
  display.display();
  delay(100);
  display.clearDisplay();

  touchAttachInterrupt(TOUCH_1, trocaDeTelaPressionada, 30);

  xTaskCreate(&blink_task, "blink_task", 1024, NULL, 5, &blinkHandle);
  xTaskCreate(&bufferCheio_task, "bufferCheio_task", 8192, NULL, 1, &bufferCheioHandle);
  xTaskCreate(&senoideTela_task, "senoideTela_task", 8192, NULL, 5, &senoideTelaHandle);

  xTaskCreate(&telaBasica_task, "telaBasica_task", 8192, NULL, 5, &handleTelaBasica);
  xTaskCreate(&telaDebug_task, "telaDebug_task", 8192, NULL, 5, &handleTelaDebug);
  xTaskCreate(&graficoPotencia_task, "graficoPotencia_task", 8192, NULL, 5, &handleGraficoPotencia);
  xTaskCreate(&senoideTensao_task, "senoideTensao_task", 8192, NULL, 5, &handleSenoideTensao);
  xTaskCreate(&senoideCorrente_task, "senoideCorrente_task", 8192, NULL, 5, &handleSenoideCorrente);
  xTaskCreate(&fftTensao_task, "fftTensao_task", 8192, NULL, 5, &handleTelaFFTTensao);
  xTaskCreate(&fftCorrente_task, "fftCorrente_task", 8192, NULL, 5, &handleTelaFFTCorrente);

  adcTimer = timerBegin(3, 80, true);
  timerAttachInterrupt(adcTimer, &amostragemViaTimerInterrupt, true);
  timerAlarmWrite(adcTimer, PERIODO_DE_AMOSTRAGEM_EM_MICRO_SEGUNDOS, true);
  timerAlarmEnable(adcTimer);
}

void loop()
{
  //
}
