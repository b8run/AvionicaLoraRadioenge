#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include "LoRaMESH.h"
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>

// --- Configurações de Hardware ---
#define BMP_ADDR 0x76                   
#define SD_CS 5
#define GPS_RX_PIN 4
#define GPS_TX_PIN 2
#define SQUIB_PIN 26                     // Pino do Acionador do Paraquedas (Relé/Mosfet)

// --- Comandos de Protocolo LoRa ---
#define CMD_COLETAR_TELEMETRIA 0x22     // Coleta manual (fallback)
#define CMD_BAIXAR_SD 0x23              // Inicia download do arquivo de log
#define CMD_FIM_ARQUIVO 0x24            // Sinaliza fim do download
#define CMD_OBTER_GPS 0x25              // Solicita apenas Lat/Lon
#define CMD_ARMAR_FOGUETE 0x26          // Força estado para SUBIDA via rádio
#define CMD_TELEMETRIA_AUTO 0x27        // Identificador de pacote automático de voo
#define CMD_RESETAR_BASE 0x28           // Zera a altitude de referência (baseline)

#define MAX_LORA_PAYLOAD 100 

// --- Máquina de Estados ---
enum EstadoVoo { AGUARDANDO, SUBIDA, DESCIDA, POUSADO };
EstadoVoo estadoAtual = AGUARDANDO;

// --- Instâncias Globais ---
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
HardwareSerial SerialLoRa(2);
HardwareSerial SerialGPS(1);
LoRaMESH lora(&SerialLoRa);
TinyGPSPlus gps;

// --- Variáveis de Navegação e Timers ---
float altitudeMaxima = -9999.0;
float altitudePadrao = 1013.25;         // QNH local (ajuste no dia do lançamento)
float altitudeAtual = 0.0;
float altitudeBase = 0.0;              // Altitude capturada no zero (reset de baseline)

uint32_t tempoUltimoLog = 0;
uint32_t tempoUltimoLoRa = 0;
uint32_t tempoAcionamentoSquib = 0;
uint32_t tempoUltimaAltitude = 0;       // Para verificar pouso

uint16_t senderId;                      
uint8_t commandIn;                      
uint8_t rxBuffer[MAX_PAYLOAD_SIZE];     
uint8_t rxSize;                         

// --- Declaração de Funções ---
void InicializaSensores();              
void InicializaLoRa();                  
void InicializaSD();                    
void EscutaComandos();                  
void ControleDeVoo();                   
void AcionaParaquedas();                
void SalvaDadosSD(sensors_event_t& a, sensors_event_t& g); 
void TransmiteTelemetriaAutomatica();   
void EnviaDadosSDLoRa();                
void EnviaGPSLoRa();                    
void VerificaPouso();
void ResetarAltitudeBase();             // Zera a altitude relativa ao ponto de lançamento

//========================================================================
// SETUP
//========================================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(SQUIB_PIN, OUTPUT);
  digitalWrite(SQUIB_PIN, LOW);         // Garante squib desligado por segurança
  
  InicializaLoRa();
  InicializaSensores();
  //InicializaSD();
  ResetarAltitudeBase();                // Captura altitude atual como ponto zero
  
  Serial.println("[FOGUETE] Sistema de Bordo Pronto. Estado: AGUARDANDO.");
}

//========================================================================
// LOOP PRINCIPAL
//========================================================================
void loop() {
  // 1. Alimenta o decodificador GPS continuamente
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  
  // 2. Escuta comandos do Master (timeout rápido para não travar o voo)
  EscutaComandos();

  // 3. Executa a lógica de navegação, detecção de apogeu e log
  ControleDeVoo();
}

//========================================================================
// INICIALIZAÇÕES
//========================================================================
void InicializaLoRa() {
  SerialLoRa.begin(9600, SERIAL_8N1, 16, 17);
  lora.begin(false);
}

void InicializaSensores() {
  Wire.begin(21, 22);
Wire.setClock(400000); // I2C mais rápido (400kHz)
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (!bmp.begin(BMP_ADDR)) {
    Serial.println("[ERRO] Sensor BMP280 não encontrado!");
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                    Adafruit_BMP280::SAMPLING_X2,     
                    Adafruit_BMP280::SAMPLING_X16,    
                    Adafruit_BMP280::FILTER_X16,      
                    Adafruit_BMP280::STANDBY_MS_1); 
  }

  if (!mpu.begin()) {
    Serial.println("[ERRO] Sensor MPU6050 não encontrado!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G); // Importante: Suporta até 16G no lançamento
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

void InicializaSD() {
  Serial.print("[SD] Inicializando cartão...");
  if (!SD.begin(SD_CS)) {
    Serial.println(" Falhou!");
    return;
  }
  Serial.println(" OK!");
  
  File file = SD.open("/flight_data.csv", FILE_APPEND);
  if (file) {
    if (file.size() == 0) {
      file.println("Tempo(ms),Estado,Altitude(m),AccX,AccY,AccZ,GyrX,GyrY,GyrZ,Lat,Lon,hora");
    }
    file.close();
  }
}

//========================================================================
// CONTROLE DE VOO E NAVEGAÇÃO
//========================================================================
void ControleDeVoo() {
  uint32_t now = millis();
  
  // Atualiza leituras inerciais e barométricas (altitude RELATIVA ao ponto de lançamento)
  altitudeAtual = bmp.readAltitude(altitudePadrao) - altitudeBase;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- LÓGICA DE ESTADOS ---
  if (estadoAtual == AGUARDANDO) {
    // Detecta lançamento (ex: Z ou X > 3G dependendo de como a placa está montada no foguete)
    // 1G = ~9.8 m/s^2. 3G = ~29.4 m/s^2.
    float accTotal = sqrt(
      a.acceleration.x * a.acceleration.x +
      a.acceleration.y * a.acceleration.y +
      a.acceleration.z * a.acceleration.z
    );
    if (accTotal > 35.0) { // ~3.5G
      estadoAtual = SUBIDA;
      Serial.println("[VOO] LANÇAMENTO DETECTADO! Estado: SUBIDA");
    }
  } 
  else if (estadoAtual == SUBIDA) {
    if (altitudeAtual > altitudeMaxima) {
      altitudeMaxima = altitudeAtual;
    }
    
    // Detecta Apogeu: Caiu 3 metros do pico máximo (filtro simples contra ruído)
    if ((altitudeMaxima - altitudeAtual) > 1.0) {
      estadoAtual = DESCIDA;
      Serial.println("[VOO] APOGEU DETECTADO! Acionando Paraquedas...");
      AcionaParaquedas();
    }
  }
  else if (estadoAtual == DESCIDA) {
    VerificaPouso();
  }

  // --- CONTROLE DO SQUIB ---
  // Desliga o relé 2 segundos após acionado para não queimar e economizar bateria
  if ((now - tempoAcionamentoSquib > 2000) && tempoAcionamentoSquib > 0) {
    digitalWrite(SQUIB_PIN, LOW);
    tempoAcionamentoSquib = 0; 
  }

  // --- DATALOGGER (Gravando no SD a 20Hz / cada 50ms) ---
  if (now - tempoUltimoLog >= 50) {
    SalvaDadosSD(a, g);
    tempoUltimoLog = now;
  }

  // --- TELEMETRIA EM VOO (Enviando dados essenciais a 1Hz) ---
  if (estadoAtual == SUBIDA || estadoAtual == DESCIDA) {
    if (now - tempoUltimoLoRa >= 1000) {
      TransmiteTelemetriaAutomatica();
      tempoUltimoLoRa = now;
    }
  }
}

void VerificaPouso() {
  // Se a altitude não mudou mais que 1 metro nos últimos 5 segundos, considera pousado.
  static float altitudeAnterior = 0;
  if (millis() - tempoUltimaAltitude > 5000) {
    if (abs(altitudeAtual - altitudeAnterior) < 1.0) {
      estadoAtual = POUSADO;
      Serial.println("[VOO] POUSO DETECTADO. Aguardando resgate.");
    }
    altitudeAnterior = altitudeAtual;
    tempoUltimaAltitude = millis();
  }
}

void AcionaParaquedas() {
  digitalWrite(SQUIB_PIN, HIGH);
  tempoAcionamentoSquib = millis();
}

//========================================================================
// GRAVAÇÃO DE DADOS (SD CARD)
//========================================================================
void SalvaDadosSD(sensors_event_t& a, sensors_event_t& g) {
  File file = SD.open("/flight_data.csv", FILE_APPEND);
  if (file) {
    file.print(millis()); file.print(",");
    file.print(estadoAtual); file.print(",");
    file.print(altitudeAtual); file.print(",");
    file.print(a.acceleration.x); file.print(",");
    file.print(a.acceleration.y); file.print(",");
    file.print(a.acceleration.z); file.print(",");
    file.print(g.gyro.x); file.print(",");
    file.print(g.gyro.y); file.print(",");
    file.print(g.gyro.z); file.print(",");
    
    if (gps.location.isValid()) {
      file.print(gps.location.lat(), 6); file.print(",");
      file.print(gps.location.lng(), 6); file.print(",");
      file.println(gps.location.age(), 6);
    } else {
      file.println("0.0,0.0,0.0");
    }
    file.close(); 
  }
}

//========================================================================
// COMUNICAÇÃO LORA (RECEBIMENTO E ENVIO)
//========================================================================
void EscutaComandos() {
  // Timeout de 10ms garante que a função saia rápido se não houver pacote
  if (lora.ReceivePacketCommand(&senderId, &commandIn, rxBuffer, &rxSize, 10)) {
    if (commandIn == 0x28) {
          Serial.println("recebi :"+rxBuffer[0]);
      if (rxBuffer[0] == CMD_ARMAR_FOGUETE) {
        estadoAtual = SUBIDA;
        Serial.println("[COMANDO] Foguete armado via RF.");
      } 
      else if (rxBuffer[0] == CMD_RESETAR_BASE) {
        ResetarAltitudeBase();
        Serial.println("[COMANDO] Baseline de altitude resetada via RF.");
      }
      else if (rxBuffer[0] == CMD_OBTER_GPS) {
        EnviaGPSLoRa();
      }
      else if (rxBuffer[0] == CMD_BAIXAR_SD) {
        // Por segurança, só permite baixar o SD se não estiver voando
        if (estadoAtual == AGUARDANDO || estadoAtual == POUSADO) {
          EnviaDadosSDLoRa();
        } else {
          Serial.println("[AVISO] Comando de download ignorado. Foguete em voo!");
        }
      }
    }
  }
}

void TransmiteTelemetriaAutomatica() {
  uint8_t txBuffer[10];
  txBuffer[0] = CMD_TELEMETRIA_AUTO; 
  txBuffer[1] = estadoAtual;
  
  // Compacta a altitude atual para caber em 2 bytes
  int16_t alt_int = (int16_t)altitudeAtual;
  txBuffer[2] = (alt_int >> 8) & 0xFF;
  txBuffer[3] = alt_int & 0xFF;

  if (lora.PrepareFrameCommand(senderId, 0x28, txBuffer, 4)) {
    lora.SendPacket();
  }
}

void EnviaGPSLoRa() {
  uint8_t txBuffer[9];
  txBuffer[0] = CMD_OBTER_GPS; 
  
  int32_t lat32 = 0;
  int32_t lon32 = 0;
  
  if (gps.location.isValid()) {
    lat32 = (int32_t)(gps.location.lat() * 1000000);
    lon32 = (int32_t)(gps.location.lng() * 1000000);
  }
  
  // Empacotamento Big-Endian
  txBuffer[1] = (lat32 >> 24) & 0xFF; txBuffer[2] = (lat32 >> 16) & 0xFF;
  txBuffer[3] = (lat32 >> 8) & 0xFF;  txBuffer[4] = lat32 & 0xFF;
  
  txBuffer[5] = (lon32 >> 24) & 0xFF; txBuffer[6] = (lon32 >> 16) & 0xFF;
  txBuffer[7] = (lon32 >> 8) & 0xFF;  txBuffer[8] = lon32 & 0xFF;

  if (lora.PrepareFrameCommand(senderId, 0x28, txBuffer, 9)) {
    lora.SendPacket();
    Serial.println("[FOGUETE] Coordenadas de resgate transmitidas.");
  }
}

void ResetarAltitudeBase() {
  // Faz média de 20 leituras para uma baseline estável (evita ruído pontual)
  float soma = 0.0;
  const int N = 20;
  for (int i = 0; i < N; i++) {
    soma += bmp.readAltitude(altitudePadrao);
    delay(10);
  }
  altitudeBase = soma / N;
  altitudeMaxima = -9999.0;            // Reseta também o pico registrado
  Serial.print("[BMP] Baseline de altitude definida em: ");
  Serial.print(altitudeBase);
  Serial.println(" m (absoluto). Altitude relativa = 0 m.");
}

void EnviaDadosSDLoRa() {
  Serial.println("[FOGUETE] Iniciando transmissão do Log...");
  
  File file = SD.open("/flight_data.csv", FILE_READ);
  if (!file) {
    Serial.println("[ERRO] Arquivo não encontrado.");
    return;
  }

  uint8_t txBuffer[MAX_LORA_PAYLOAD];
  txBuffer[0] = CMD_BAIXAR_SD; 
  
  while (file.available()) {
    int index = 1; 
    while (file.available() && index < MAX_LORA_PAYLOAD) {
      txBuffer[index++] = file.read();
    }
    
    if (lora.PrepareFrameCommand(senderId, 0x28, txBuffer, index)) {
      lora.SendPacket();
      delay(500); // Essencial para não afogar o módulo LoRa do receptor
    }
  }
  file.close();

  // Envia pacote final indicando conclusão
  txBuffer[0] = CMD_FIM_ARQUIVO;
  if (lora.PrepareFrameCommand(senderId, 0x28, txBuffer, 1)) {
    lora.SendPacket();
    Serial.println("[FOGUETE] Download via RF concluído.");
  }
}