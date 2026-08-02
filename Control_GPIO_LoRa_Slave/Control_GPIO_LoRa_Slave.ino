/*
  Slave.ino  -  FOGUETE (LoRa_E220 + logging de status de envio/recepcao)
  Sensor de pressao: BMP280
  v4.0 — Protocolo unificado (Response de tamanho fixo), dump do SD em
         binario (mais rapido e ~40% menor que texto), checksum CRC8 e
         reenvio pontual de registros (CMD_RESEND_SD).
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
// Instancias
// --------------------------------------------------------------------------
LoRa_E220 Transceiver(&Serial2, PIN_AUX, PIN_M0, PIN_M1);
HardwareSerial SerialGPS(1);
TinyGPSPlus gps;
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
SPIClass spiSD(HSPI);

bool statusBMP   = false, statusMPU = false, statusSD = false, statusLoRa = false, statusSquib = false;

float altitudeAtual = 0.0, velocidadeAtual = 0.0, altitudeMaxima = -9999.0, pressaoBase = 0.0;
char nomeArquivoVoo[20]    = "/voo_001.csv"; // log legivel (para leitura direta do cartao)
char nomeArquivoVooBin[20] = "/voo_001.bin"; // log binario (usado no dump via LoRa)

enum EstadoVoo { AGUARDANDO, SUBIDA, DESCIDA, POUSADO };
EstadoVoo estadoAtual = AGUARDANDO;

float    accTotal = 0.0;
uint16_t _numLeituras = 0;
uint32_t tempoUltimoLog = 0, tempoUltimoLoRa = 0, tempoUltimoBMP = 0;
uint32_t tempoAcionamentoSquib = 0, tempoPouso = 0, tempoUltimoBeacon = 0;
bool     gpsPousoEnviado = false, beaconAtivo = false;

#define SD_BUFFER_MAX 40
LogLine sdBuffer[SD_BUFFER_MAX]; // LogLine agora vem de LoRaProtocol.h (struct compartilhada)
uint8_t sdBufferIdx = 0;

// --------------------------------------------------------------------------
// Helpers de protocolo (Response com checksum)
// --------------------------------------------------------------------------
Response montarResposta(uint8_t tipo) {
  Response r;
  memset(&r, 0, sizeof(Response)); // zera tudo, inclusive bytes nao usados do union (deterministico p/ checksum)
  r.resp      = tipo;
  r.timestamp = millis();
  return r;
}

void enviarResposta(Response& r) {
  r.checksum = calcChecksum(r);
  Transceiver.sendMessage((void*)&r, sizeof(r));
}

// --------------------------------------------------------------------------
// SD - gravacao (CSV legivel + binario para dump rapido via LoRa)
// --------------------------------------------------------------------------
void FlushBufferSD() {
  if (!statusSD || sdBufferIdx == 0) return;

  // 1) Log legivel (para leitura direta do cartao, fora do voo)
  File fcsv = SD.open(nomeArquivoVoo, FILE_APPEND);
  if (fcsv) {
    for (uint8_t i = 0; i < sdBufferIdx; i++) {
      fcsv.print(sdBuffer[i].tempo);        fcsv.print(",");
      fcsv.print(sdBuffer[i].estado);       fcsv.print(",");
      fcsv.print(sdBuffer[i].altitude, 2);  fcsv.print(",");
      fcsv.print(sdBuffer[i].velocidade, 2);fcsv.print(",");
      fcsv.print(sdBuffer[i].accX, 3);      fcsv.print(",");
      fcsv.print(sdBuffer[i].accY, 3);      fcsv.print(",");
      fcsv.print(sdBuffer[i].accZ, 3);      fcsv.print(",");
      if (sdBuffer[i].lat != 0.0 || sdBuffer[i].lon != 0.0) {
        fcsv.print(sdBuffer[i].lat, 6); fcsv.print(","); fcsv.println(sdBuffer[i].lon, 6);
      } else {
        fcsv.println("0,0");
      }
    }
    fcsv.close();
  }

  // 2) Log binario (fonte usada no dump via LoRa: sem parsing, leitura direta por offset)
  File fbin = SD.open(nomeArquivoVooBin, FILE_APPEND);
  if (fbin) {
    fbin.write((uint8_t*)sdBuffer, sizeof(LogLine) * sdBufferIdx);
    fbin.close();
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
    sprintf(nomeArquivoVoo,    "/voo_%03d.csv", num);
    sprintf(nomeArquivoVooBin, "/voo_%03d.bin", num);
    if (!SD.exists(nomeArquivoVoo) && !SD.exists(nomeArquivoVooBin)) break;
    num++;
  }
  File csv = SD.open(nomeArquivoVoo, FILE_WRITE);
  if (csv) {
    csv.println("Tempo,Estado,Alt,Vel,AccX,AccY,AccZ,Lat,Lon");
    csv.close();
    Serial.print(F("[SD] Arquivo criado: ")); Serial.println(nomeArquivoVoo);
  } else {
    Serial.println(F("[SD] ERRO ao criar arquivo CSV!"));
  }
  File bin = SD.open(nomeArquivoVooBin, FILE_WRITE);
  if (bin) {
    bin.close();
    Serial.print(F("[SD] Arquivo binario criado: ")); Serial.println(nomeArquivoVooBin);
  } else {
    Serial.println(F("[SD] ERRO ao criar arquivo binario!"));
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
    // Tenta com o chip ID generico caso seja uma variante chinesa comum (0x58)
    statusBMP = bmp.begin(BMP_ADDR, 0x58);
  }

  if (statusBMP) {
    // Configura o BMP280 para rodar de forma continua em background (MODE_NORMAL)
    // Otimizado para leitura rapida de altitude em foguetes
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
  Serial.print(F("Tamanho do pacote (Response): ")); Serial.print(sizeof(Response)); Serial.println(F(" bytes"));
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

  // Leitura direta do BMP280 - Nao precisa de estado assincrono manual
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
      Response r = montarResposta(RESP_GPS);
      r.dados.gps.lat = gps.location.isValid() ? gps.location.lat() : 0.0;
      r.dados.gps.lon = gps.location.isValid() ? gps.location.lng() : 0.0;
      enviarResposta(r); gpsPousoEnviado = true; sinalGPSEnviado();
    }
    if (beaconAtivo && (now - tempoUltimoBeacon >= 90000)) { tempoUltimoBeacon = now; sinalBeacon(); }
  }

  if (tempoAcionamentoSquib > 0 && (now - tempoAcionamentoSquib > 2000)) {
    digitalWrite(SQUIB_PIN, LOW); tempoAcionamentoSquib = 0; FlushBufferSD(); sinalSquibDesligado();
  }

  if (now - tempoUltimoLog >= 50) { SalvaDadosSD(a, g); tempoUltimoLog = now; }
  if (estadoAtual == SUBIDA || estadoAtual == DESCIDA) {
    if (now - tempoUltimoLoRa >= 750) { Command c; c.cmd = CMD_READ; c.param = 0; atenderComando(c); tempoUltimoLoRa = now; }
  }

  static uint32_t tempoUltimoFlush = 0;
  if (estadoAtual == AGUARDANDO && (now - tempoUltimoFlush >= 10000)) {
    if (sdBufferIdx > 0) FlushBufferSD(); tempoUltimoFlush = now;
  }
}

void atenderComando(const Command& c) {
  Serial.print("Comando Recebido - ");
  Serial.println(c.cmd);

  switch (c.cmd) {
    case CMD_READ: {
      Response r = montarResposta(RESP_DATA);
      r.contador          = estadoAtual;
      r.dados.telemetria.alt   = altitudeAtual;
      r.dados.telemetria.speed = abs(velocidadeAtual);
      r.dados.telemetria.lat   = gps.location.isValid() ? gps.location.lat() : 0.0;
      r.dados.telemetria.lon   = gps.location.isValid() ? gps.location.lng() : 0.0;
      enviarResposta(r);
      break;
    }
    case CMD_GET_GPS: {
      Response r = montarResposta(RESP_GPS);
      r.dados.gps.lat = gps.location.isValid() ? gps.location.lat() : 0.0;
      r.dados.gps.lon = gps.location.isValid() ? gps.location.lng() : 0.0;
      enviarResposta(r);
      break;
    }
    case CMD_PING: {
      Response r = montarResposta(RESP_PONG);
      enviarResposta(r);
      break;
    }
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
    case CMD_RESEND_SD:
      enviarRegistroSD(c.param);
      break;
  }
}

// --------------------------------------------------------------------------
// Dump do SD via LoRa - binario, com seq + checksum, muito mais leve que o
// envio de bytes crus de texto CSV da v3.4 (~40% menos bytes por registro
// e framing correto, ja que todo pacote usa a mesma struct Response).
// --------------------------------------------------------------------------
void EnviaDadosSDLoRa() {
  if (!statusSD) return;

  File file = SD.open(nomeArquivoVooBin, FILE_READ);
  if (!file) { Serial.println(F("[SD] ERRO ao abrir arquivo binario!")); return; }

  uint32_t totalRegistros = file.size() / sizeof(LogLine);
  Serial.print(F("[SD] Iniciando envio via LoRa. Total de registros: "));
  Serial.println(totalRegistros);

  // 1) Anuncia quantos registros virao, para o Master detectar buracos depois
  Response inicio = montarResposta(RESP_SD_INICIO);
  inicio.dados.sdInicio.totalRegistros = (uint16_t)totalRegistros;
  enviarResposta(inicio);
  delay(150);

  // 2) Envia cada registro
  LogLine reg;
  uint16_t seq = 0;
  while (file.read((uint8_t*)&reg, sizeof(LogLine)) == sizeof(LogLine)) {
    Response r = montarResposta(RESP_SD_RECORD);
    r.dados.sdRegistro.seq        = seq;
    r.dados.sdRegistro.tempo      = reg.tempo;
    r.dados.sdRegistro.estado     = reg.estado;
    r.dados.sdRegistro.altitude   = reg.altitude;
    r.dados.sdRegistro.velocidade = reg.velocidade;
    r.dados.sdRegistro.accX       = reg.accX;
    r.dados.sdRegistro.accY       = reg.accY;
    r.dados.sdRegistro.accZ       = reg.accZ;
    r.dados.sdRegistro.lat        = reg.lat;
    r.dados.sdRegistro.lon        = reg.lon;
    enviarResposta(r);

    Serial.print(F("[SD] Enviado registro ")); Serial.print(seq + 1);
    Serial.print("/"); Serial.println(totalRegistros);

    seq++;
    delay(120); // airtime menor que a v3.4 (pacote fixo de 43 bytes)
  }
  file.close();

  Response fim = montarResposta(RESP_SD_FIM);
  enviarResposta(fim);
  Serial.println(F("[SD] Envio concluido."));
}

// Reenvia UM registro especifico, por seq, sem precisar refazer o dump inteiro.
// Usado pelo Master quando detecta um buraco (pacote perdido) ao final do download.
void enviarRegistroSD(uint16_t seq) {
  if (!statusSD) return;
  File file = SD.open(nomeArquivoVooBin, FILE_READ);
  if (!file) return;

  uint32_t offset = (uint32_t)seq * sizeof(LogLine);
  if (offset >= file.size()) { file.close(); return; }

  file.seek(offset);
  LogLine reg;
  if (file.read((uint8_t*)&reg, sizeof(LogLine)) == sizeof(LogLine)) {
    Response r = montarResposta(RESP_SD_RECORD);
    r.dados.sdRegistro.seq        = seq;
    r.dados.sdRegistro.tempo      = reg.tempo;
    r.dados.sdRegistro.estado     = reg.estado;
    r.dados.sdRegistro.altitude   = reg.altitude;
    r.dados.sdRegistro.velocidade = reg.velocidade;
    r.dados.sdRegistro.accX       = reg.accX;
    r.dados.sdRegistro.accY       = reg.accY;
    r.dados.sdRegistro.accZ       = reg.accZ;
    r.dados.sdRegistro.lat        = reg.lat;
    r.dados.sdRegistro.lon        = reg.lon;
    enviarResposta(r);
    Serial.print(F("[SD] Reenviado registro seq=")); Serial.println(seq);
  }
  file.close();
}
