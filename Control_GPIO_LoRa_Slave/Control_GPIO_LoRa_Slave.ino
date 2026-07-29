/*
  Slave.ino  -  FOGUETE (LoRa_E220 + logging de status de envio/recepcao)
  Sensor de pressão: BMP280
  v3.4 — Adaptado exclusivamente para BMP280 (Modo Contínuo)
*/

#include "Arduino.h"
#include "LoRa_E220.h"
#include "LoRaProtocol.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <SD.h>
#include <TinyGPSPlus.h>

// --- Pinos LoRa E220 ---
#define PIN_RX   16
#define PIN_TX   17
#define PIN_M0   4
#define PIN_M1   13
#define PIN_AUX  35

// --- Pinos Sensores ---
#define SD_CS        5
#define SD_MOSI      23
#define SD_MISO      19
#define SD_SCK       18
#define GPS_RX_PIN   36
#define GPS_TX_PIN   14

// --- Pinos Atuadores ---
#define SQUIB_PIN    33
#define LED_R        2
#define LED_G        27
#define LED_B        25
#define BUZZER_PIN   32
#define BMP_ADDR     0x76

#define BUZZER_RES   8

// --------------------------------------------------------------------------
// LED E BUZZER
// --------------------------------------------------------------------------
void ledSet(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}
void ledOff()      { ledSet(false, false, false); }
void ledVermelho() { ledSet(true,  false, false); }
void ledAmarelo()  { ledSet(true,  true,  false); }
void ledVerde()    { ledSet(false, true,  false); }
void ledAzul()     { ledSet(false, false, true);  }
void ledBranco()   { ledSet(true,  true,  true);  }

void buzzerTom(uint16_t freq, uint16_t durMs) {
  if (freq == 0) {
    ledcWrite(BUZZER_PIN, 0);
  } else {
    ledcWriteTone(BUZZER_PIN, freq);
    ledcWrite(BUZZER_PIN, 128);
  }
  delay(durMs);
  ledcWrite(BUZZER_PIN, 0);
}

void buzzerSilencio(uint16_t durMs) {
  ledcWrite(BUZZER_PIN, 0);
  delay(durMs);
}

void sinalBoot() {
  for (int i = 0; i < 3; i++) { ledBranco(); buzzerTom(1000, 80); ledOff(); delay(80); }
}
void sinalSensorOK(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) { ledVerde(); buzzerTom(1800, 60); ledOff(); delay(120); }
}
void sinalSensorFalha() {
  for (int i = 0; i < 3; i++) { ledVermelho(); buzzerTom(200, 100); ledOff(); delay(80); }
}
void sinalLoRaOK() {
  for (int i = 0; i < 4; i++) { ledAzul(); buzzerTom(1200, 60); ledOff(); delay(120); }
}
void sinalPronto() {
  ledAmarelo();
  buzzerTom(800,  80); buzzerSilencio(40);
  buzzerTom(1000, 80); buzzerSilencio(40);
  buzzerTom(1200, 80); buzzerSilencio(40);
  buzzerTom(1600, 120);
}
void sinalLancamento() {
  ledVerde();
  for (int f = 800; f <= 2400; f += 200) { ledcWriteTone(BUZZER_PIN, f); ledcWrite(BUZZER_PIN, 128); delay(40); }
  ledcWrite(BUZZER_PIN, 0);
}
void sinalApogeu() {
  for (int i = 0; i < 6; i++) { ledVermelho(); buzzerTom(2000, 60); ledAmarelo(); buzzerTom(1400, 60); }
  ledOff(); buzzerSilencio(50);
}
void sinalSquibDesligado() {
  ledAmarelo(); buzzerTom(1000, 150); buzzerSilencio(100); buzzerTom(1000, 150); ledOff();
}
void sinalPouso() {
  for (int i = 0; i < 3; i++) { ledVerde(); buzzerTom(1600, 150); ledOff(); delay(150); }
  buzzerSilencio(100);
  for (int f = 1600; f >= 400; f -= 200) { ledcWriteTone(BUZZER_PIN, f); ledcWrite(BUZZER_PIN, 128); delay(60); }
  ledcWrite(BUZZER_PIN, 0); ledVermelho();
}
void sinalGPSEnviado() {
  for (int i = 0; i < 2; i++) { ledAzul(); buzzerTom(1400, 80); ledOff(); delay(80); }
  ledVermelho();
}
void sinalBeacon() {
  Serial.println(F("[BEACON] Sinal de localizacao ativo (5s)"));
  uint32_t inicio = millis();
  while (millis() - inicio < 5000) { ledAmarelo(); buzzerTom(2200, 80); ledOff(); delay(170); }
  ledVermelho();
}
void sinalErroCritico() {
  while (1) { ledVermelho(); buzzerTom(300, 400); ledOff(); buzzerSilencio(200); delay(1000); }
}

// --------------------------------------------------------------------------
// Instâncias
// --------------------------------------------------------------------------
LoRa_E220 Transceiver(&Serial2, PIN_AUX, PIN_M0, PIN_M1);
HardwareSerial SerialGPS(1);
TinyGPSPlus gps;
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
SPIClass spiSD(HSPI);

bool statusBMP   = false, statusMPU = false, statusSD = false, statusLoRa = false, statusSquib = false;

float altitudeAtual = 0.0, velocidadeAtual = 0.0, altitudeMaxima = -9999.0, pressaoBase = 0.0;
char nomeArquivoVoo[20] = "/voo_001.csv";

enum EstadoVoo { AGUARDANDO, SUBIDA, DESCIDA, POUSADO };
EstadoVoo estadoAtual = AGUARDANDO;

float    accTotal = 0.0;
uint16_t _numLeituras = 0;
uint32_t tempoUltimoLog = 0, tempoUltimoLoRa = 0, tempoUltimoBMP = 0;
uint32_t tempoAcionamentoSquib = 0, tempoPouso = 0, tempoUltimoBeacon = 0;
bool     gpsPousoEnviado = false, beaconAtivo = false;

#define SD_BUFFER_MAX 40
struct LogLine {
  uint32_t tempo; uint8_t estado;
  float altitude, velocidade, accX, accY, accZ, lat, lon;
};
LogLine sdBuffer[SD_BUFFER_MAX];
uint8_t sdBufferIdx = 0;

void FlushBufferSD() {
  if (!statusSD || sdBufferIdx == 0) return;
  File file = SD.open(nomeArquivoVoo, FILE_APPEND);
  if (file) {
    for (uint8_t i = 0; i < sdBufferIdx; i++) {
      file.print(sdBuffer[i].tempo);       file.print(",");
      file.print(sdBuffer[i].estado);      file.print(",");
      file.print(sdBuffer[i].altitude, 2); file.print(",");
      file.print(sdBuffer[i].velocidade, 2); file.print(",");
      file.print(sdBuffer[i].accX, 3);     file.print(",");
      file.print(sdBuffer[i].accY, 3);     file.print(",");
      file.print(sdBuffer[i].accZ, 3);     file.print(",");
      if (sdBuffer[i].lat != 0.0 || sdBuffer[i].lon != 0.0) {
        file.print(sdBuffer[i].lat, 6); file.print(","); file.println(sdBuffer[i].lon, 6);
      } else {
        file.println("0,0");
      }
    }
    file.close();
  }
  sdBufferIdx = 0;
}

void SalvaDadosSD(sensors_event_t& a, sensors_event_t& g) {
  if (!statusSD) return;
  if (sdBufferIdx < SD_BUFFER_MAX) {
    sdBuffer[sdBufferIdx].tempo      = millis();
    sdBuffer[sdBufferIdx].estado     = estadoAtual;
    sdBuffer[sdBufferIdx].altitude   = altitudeAtual;
    sdBuffer[sdBufferIdx].velocidade = velocidadeAtual;
    sdBuffer[sdBufferIdx].accX       = a.acceleration.x;
    sdBuffer[sdBufferIdx].accY       = a.acceleration.y;
    sdBuffer[sdBufferIdx].accZ       = a.acceleration.z;
    sdBuffer[sdBufferIdx].lat        = gps.location.isValid() ? gps.location.lat() : 0.0;
    sdBuffer[sdBufferIdx].lon        = gps.location.isValid() ? gps.location.lng() : 0.0;
    sdBufferIdx++;
  }
  if (sdBufferIdx >= SD_BUFFER_MAX) FlushBufferSD();
}

void criarArquivoVoo() {
  uint8_t num = 1;
  while (num < 255) {
    sprintf(nomeArquivoVoo, "/voo_%03d.csv", num);
    if (!SD.exists(nomeArquivoVoo)) break;
    num++;
  }
  File csv = SD.open(nomeArquivoVoo, FILE_WRITE);
  if (csv) {
    csv.println("Tempo,Estado,Alt,Vel,AccX,AccY,AccZ,Lat,Lon");
    csv.close();
    Serial.print(F("[SD] Arquivo criado: ")); Serial.println(nomeArquivoVoo);
  } else {
    Serial.println(F("[SD] ERRO ao criar arquivo!"));
  }
}

float setBaseline(int amostras) {
  Serial.print(F("  Calculando baseline (")); Serial.print(amostras); Serial.println(F(" amostras)..."));
  double soma = 0; int validas = 0;
  for (int i = 0; i < amostras; i++) {
    float press = bmp.readPressure() / 100.0F; // hPa
    if (press > 300.0 && press < 1100.0) { soma += press; validas++; }
    delay(50);
  }
  if (validas == 0) return 0.0;
  float baseline = soma / validas;
  Serial.print(F("  Baseline: ")); Serial.print(baseline, 4); Serial.println(F(" hPa"));
  return baseline;
}

void VerificacaoSistema() {
  Serial.println(F("========================================"));
  Serial.println(F("  VERIFICACAO DO SISTEMA (SLAVE)"));
  Serial.println(F("========================================"));

  // O BMP280 pode usar 0x76 ou 0x77. Usaremos 0x76 conforme definido.
  statusBMP = bmp.begin(BMP_ADDR); 
  if (!statusBMP) {
    // Tenta com o chip ID genérico caso seja uma variante chinesa comum (0x58)
    statusBMP = bmp.begin(BMP_ADDR, 0x58);
  }

  if (statusBMP) {
    // Configura o BMP280 para rodar de forma contínua em background (MODE_NORMAL)
    // Otimizado para leitura rápida de altitude em foguetes
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
    Serial.println(F("  [OK] BMP280")); sinalSensorOK(2); 
  } else { 
    Serial.println(F("  [FALHA] BMP280")); sinalSensorFalha(); 
  }
  delay(300);

  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    statusMPU = mpu.begin();
    if (statusMPU) {
      mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.println(F("  [OK] MPU6050")); sinalSensorOK(3);
    } else { Serial.println(F("  [FALHA] MPU6050")); sinalSensorFalha(); }
  } else { Serial.println(F("  [FALHA] MPU6050 | Erro I2C")); sinalSensorFalha(); }
  delay(300);

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  statusSD = SD.begin(SD_CS, spiSD);
  if (statusSD) { Serial.println(F("  [OK] SD Card")); }
  else { Serial.println(F("  [FALHA] SD Card")); }
  delay(300);

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(1000);
  bool gpsOk = false; uint32_t t0 = millis();
  while (millis() - t0 < 1000) { if (SerialGPS.available()) { gpsOk = true; break; } }
  if (gpsOk) { Serial.println(F("  [OK] GPS")); sinalSensorOK(5); }
  else { Serial.println(F("  [FALHA] GPS")); sinalSensorFalha(); }
  delay(300);

  Serial2.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
  Transceiver.begin();
  ResponseStructContainer c = Transceiver.getConfiguration();
  statusLoRa = (c.status.code == 1);
  if (statusLoRa) { Serial.println(F("  [OK] LoRa E220")); sinalLoRaOK(); }
  else { Serial.println(F("  [FALHA] LoRa E220")); sinalSensorFalha(); }
  c.close();
  delay(300);

  pinMode(SQUIB_PIN, OUTPUT); digitalWrite(SQUIB_PIN, LOW); statusSquib = true;
  Serial.println(F("  [OK] Squib | LOW (seguro)"));
  Serial.println(F("----------------------------------------"));

  if (!statusBMP || !statusMPU) {
    Serial.println(F("  ERRO CRITICO: travado por seguranca.")); sinalErroCritico();
  }
}

void setup() {
  Serial.begin(115200); delay(500);
  pinMode(SQUIB_PIN, OUTPUT); digitalWrite(SQUIB_PIN, LOW);
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT); ledOff();
  ledcAttach(BUZZER_PIN, 2000, BUZZER_RES); ledcWrite(BUZZER_PIN, 0);

  ledVermelho(); sinalBoot();
  Wire.begin(21, 22); Wire.setClock(100000);

  VerificacaoSistema();

  for (int i = 0; i < 6; i++) { ledVermelho(); delay(250); ledOff(); delay(250); }
  sensors_event_t a, g, t;
  for (int i = 0; i < 50; i++) { mpu.getEvent(&a, &g, &t); delay(20); }

  pressaoBase = setBaseline(50);
  if (pressaoBase == 0.0) sinalErroCritico();
  if (statusSD) criarArquivoVoo();
  altitudeMaxima = -9999.0;

  sinalPronto();
  Serial.println(F("===== SLAVE (FOGUETE) PRONTO ====="));
}

void loop() {
  while (SerialGPS.available() > 0) gps.encode(SerialGPS.read());

  if (Transceiver.available() > 0) {
    ResponseStructContainer rsc = Transceiver.receiveMessage(sizeof(Command));
    if (rsc.status.code == 1) {
      Command c; memcpy(&c, rsc.data, sizeof(Command));
      atenderComando(c);
    }
    rsc.close();
  }

  uint32_t now = millis();
  
  // Leitura direta do BMP280 - Não precisa de estado assíncrono manual
  if (now - tempoUltimoBMP >= 25) { // 40Hz
    float novaAlt = bmp.readAltitude(pressaoBase);
    
    if (tempoUltimoBMP > 0) {
      float dt = (now - tempoUltimoBMP) / 1000.0;
      if (dt > 0.0) velocidadeAtual = (novaAlt - altitudeAtual) / dt;
    }
    
    altitudeAtual = novaAlt; 
    tempoUltimoBMP = now;
    if (sdBufferIdx >= SD_BUFFER_MAX) FlushBufferSD();
  }

  ControleDeVoo();
}

void ControleDeVoo() {
  uint32_t now = millis();
  sensors_event_t a, g, temp; mpu.getEvent(&a, &g, &temp);
  accTotal = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);

  if (estadoAtual == AGUARDANDO) {
    static uint8_t contadorLancamento = 0;
    if (accTotal > 35.0) {
      contadorLancamento++;
      if (contadorLancamento >= 5) { estadoAtual = SUBIDA; contadorLancamento = 0; sinalLancamento(); Serial.println(F("[VOO] LANCAMENTO DETECTADO!")); }
    } else { contadorLancamento = 0; }
  }
  else if (estadoAtual == SUBIDA) {
    if (altitudeAtual > altitudeMaxima) altitudeMaxima = altitudeAtual;
    static uint8_t contadorApogeu = 0;
    if ((altitudeMaxima - altitudeAtual) > 1.5) {
      contadorApogeu++;
      if (contadorApogeu >= 3) {
        estadoAtual = DESCIDA; digitalWrite(SQUIB_PIN, HIGH); tempoAcionamentoSquib = now; contadorApogeu = 0;
        sinalApogeu(); Serial.println(F("[VOO] APOGEU CONFIRMADO! Paraquedas acionado."));
      }
    } else { contadorApogeu = 0; }
  }
  else if (estadoAtual == DESCIDA) {
    ledVerde();
    static uint8_t contadorPouso = 0;
    if (accTotal > 8.5 && accTotal < 11.0 && altitudeAtual < 10.0) {
      contadorPouso++;
      if (contadorPouso >= 10) {
        estadoAtual = POUSADO; tempoPouso = now; gpsPousoEnviado = false; beaconAtivo = true; tempoUltimoBeacon = now; contadorPouso = 0;
        FlushBufferSD(); sinalPouso(); Serial.println(F("[VOO] POUSO DETECTADO!"));
      }
    } else { contadorPouso = 0; }
  }
  else if (estadoAtual == POUSADO) {
    if (!gpsPousoEnviado && (now - tempoPouso >= 1000)) {
      Response r; memset(&r, 0, sizeof(Response));
      r.timestamp = millis(); r.contador = _numLeituras++; r.resp = RESP_GPS;
      r.lat = gps.location.isValid() ? gps.location.lat() : 0.0; r.lon = gps.location.isValid() ? gps.location.lng() : 0.0;
      Transceiver.sendMessage(&r, sizeof(r)); gpsPousoEnviado = true; sinalGPSEnviado();
    }
    if (beaconAtivo && (now - tempoUltimoBeacon >= 90000)) { tempoUltimoBeacon = now; sinalBeacon(); }
  }

  if (tempoAcionamentoSquib > 0 && (now - tempoAcionamentoSquib > 2000)) {
    digitalWrite(SQUIB_PIN, LOW); tempoAcionamentoSquib = 0; FlushBufferSD(); sinalSquibDesligado();
  }

  if (now - tempoUltimoLog >= 50) { SalvaDadosSD(a, g); tempoUltimoLog = now; }
  if (estadoAtual == SUBIDA || estadoAtual == DESCIDA) {
    if (now - tempoUltimoLoRa >= 750) { Command c; c.cmd = CMD_READ; atenderComando(c); tempoUltimoLoRa = now; }
  }

  static uint32_t tempoUltimoFlush = 0;
  if (estadoAtual == AGUARDANDO && (now - tempoUltimoFlush >= 10000)) {
    if (sdBufferIdx > 0) FlushBufferSD(); tempoUltimoFlush = now;
  }
}

void enviarResposta(const Response& r) {
  Serial.println("resposta enviada");
  Transceiver.sendMessage((void*)&r, sizeof(r));
}

void atenderComando(const Command& c) {
  Response r; memset(&r, 0, sizeof(Response));
  r.timestamp = millis(); r.contador = estadoAtual;

    Serial.print("Comando Recebido - ");
    Serial.println(c.cmd);

  switch (c.cmd) {
    case CMD_READ:
      r.resp  = RESP_DATA;
      r.alt   = altitudeAtual;
      r.speed = abs(velocidadeAtual);
      r.lat   = gps.location.isValid() ? gps.location.lat() : 0.0;
      r.lon   = gps.location.isValid() ? gps.location.lng() : 0.0;
      enviarResposta(r);
      break;
    case CMD_GET_GPS:
      r.resp = RESP_GPS;
      r.lat  = gps.location.isValid() ? gps.location.lat() : 0.0;
      r.lon  = gps.location.isValid() ? gps.location.lng() : 0.0;
      enviarResposta(r);
      break;
    case CMD_PING:
      r.resp = RESP_PONG;
      enviarResposta(r);
      break;
    case CMD_ARMAR:
      estadoAtual = SUBIDA; Serial.println(F("[CMD] Armado manualmente."));
      break;
    case CMD_RESET_BASE:
      pressaoBase = setBaseline(50); altitudeMaxima = -9999.0;
      Serial.println(F("[CMD] Baseline resetada."));
      break;
    case CMD_DOWNLOAD_SD:
      if (estadoAtual == AGUARDANDO || estadoAtual == POUSADO) { FlushBufferSD(); EnviaDadosSDLoRa(); }
      break;
  }
}

void EnviaDadosSDLoRa() {
  if (!statusSD) return;
  Serial.println(F("[SD] Iniciando envio via LoRa..."));
  File file = SD.open(nomeArquivoVoo, FILE_READ);
  if (file) {
    uint8_t buffer[60];
    while (file.available()) {
      int idx = 0;
      while (file.available() && idx < 60) buffer[idx++] = file.read();
      Transceiver.sendMessage(buffer, idx);
      delay(200);
    }
    file.close();
  }
  Response r; memset(&r, 0, sizeof(Response)); r.resp = RESP_SD_FIM; enviarResposta(r);
}
