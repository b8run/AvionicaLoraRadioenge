#include <Arduino.h>
#include "LoRaMESH.h"

// Pinos da Serial do LoRa no ESP32 Master (Ajuste conforme sua montagem)
#define LORA_RX_PIN 16
#define LORA_TX_PIN 17

// --- Comandos de Protocolo LoRa (Idênticos ao Slave) ---
#define CMD_COLETAR_TELEMETRIA 0x22
#define CMD_BAIXAR_SD 0x23
#define CMD_FIM_ARQUIVO 0x24
#define CMD_OBTER_GPS 0x25
#define CMD_ARMAR_FOGUETE 0x26
#define CMD_TELEMETRIA_AUTO 0x27
#define CMD_RESETAR_BASE 0x28           // Zera a altitude de referência (baseline)


// ID de Destino (0xFFFF funciona como Broadcast em muitas bibliotecas Mesh, 
// ou coloque o ID específico do rádio do foguete se você o configurou)
#define ID_FOGUETE 1

// Instâncias
HardwareSerial SerialLoRa(2);           
LoRaMESH lora(&SerialLoRa);             

uint16_t senderId;                      
uint8_t commandIn;                      
uint8_t rxBuffer[MAX_PAYLOAD_SIZE];     
uint8_t rxSize;                         

// Declaração das Funções
void ProcessaPacoteRecebido(uint8_t* buffer, uint8_t size);
void EnviaComandoLoRa(uint8_t comando);

//========================================================================
// SETUP
//========================================================================
void setup() {
  Serial.begin(115200);
  
  // Inicializa a comunicação com o módulo LoRa
  SerialLoRa.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  lora.begin(false);
  
  Serial.println("=================================================");
  Serial.println("[BASE] Estação Master de Solo Iniciada.");
  Serial.println("[BASE] Digite os comandos no monitor serial:");
  Serial.println("  'ARMAR'  -> Força o estado do foguete para SUBIDA");
  Serial.println("  'GPS'    -> Solicita as coordenadas atuais (Lat/Lon)");
  Serial.println("  'BAIXAR' -> Inicia o download do CSV do SD Card");
  Serial.println("  'RESET_BASE_LINE' -> O Master pode mandar esse comando antes do lançamento para refazer o zero sem precisar reiniciar o foguete.");
  Serial.println("=================================================");
}

//========================================================================
// LOOP PRINCIPAL
//========================================================================
void loop() {
  // 1. ESCUTA O FOGUETE (RF -> PC)
  if (lora.ReceivePacketCommand(&senderId, &commandIn, rxBuffer, &rxSize, 10)) {
    if (commandIn == 0x28) {
      ProcessaPacoteRecebido(rxBuffer, rxSize);
    }
  }

  // 2. ESCUTA O COMPUTADOR (PC -> RF)
  if (Serial.available() > 0) {
    String comandoPC = Serial.readStringUntil('\n');
    comandoPC.trim(); // Remove espaços e quebras de linha
    comandoPC.toUpperCase();

    if (comandoPC == "ARMAR") {
      Serial.println("[BASE] Enviando comando: ARMAR FOGUETE...");
      EnviaComandoLoRa(CMD_ARMAR_FOGUETE);
    } 
    else if (comandoPC == "GPS") {
      Serial.println("[BASE] Enviando comando: OBTER GPS...");
      EnviaComandoLoRa(CMD_OBTER_GPS);
    } 
    else if (comandoPC == "BAIXAR") {
      Serial.println("[BASE] Enviando comando: BAIXAR DADOS DO SD...");
      EnviaComandoLoRa(CMD_BAIXAR_SD);
    } 
    else if (comandoPC == "RESET_BASE_LINE") {
      Serial.println("[BASE] Enviando comando: RESETAR BASE...");
      EnviaComandoLoRa(CMD_RESETAR_BASE);
    } 
    else if (comandoPC.length() > 0) {
      Serial.println("[BASE] Comando desconhecido.");
    }
    
  }
}

//========================================================================
// PROCESSAMENTO DE PACOTES RECEBIDOS DO LORA
//========================================================================
void ProcessaPacoteRecebido(uint8_t* buffer, uint8_t size) {
  uint8_t tipoPacote = buffer[0];

  switch (tipoPacote) {
    
    // --- TELEMETRIA DE VOO ---
    case CMD_TELEMETRIA_AUTO: {
      uint8_t estadoVoo = buffer[1];
      int16_t altitudeMax = (buffer[2] << 8) | buffer[3];
      
      String nomeEstado;
      if (estadoVoo == 0) nomeEstado = "AGUARDANDO";
      else if (estadoVoo == 1) nomeEstado = "SUBIDA";
      else if (estadoVoo == 2) nomeEstado = "DESCIDA";
      else if (estadoVoo == 3) nomeEstado = "POUSADO";
      else nomeEstado = "DESCONHECIDO";

      Serial.print("[TELEMETRIA] Estado: ");
      Serial.print(nomeEstado);
      Serial.print(" | Altura Atual/Max: ");
      Serial.print(altitudeMax);
      Serial.println(" m");
      break;
    }

    // --- PACOTE DE COORDENADAS GPS ---
    case CMD_OBTER_GPS: {
      if (size < 9) {
        Serial.println("[ERRO] Pacote GPS incompleto.");
        break;
      }
      
      // Reconstrói os inteiros de 32 bits (Big-Endian)
      int32_t lat32 = (buffer[1] << 24) | (buffer[2] << 16) | (buffer[3] << 8) | buffer[4];
      int32_t lon32 = (buffer[5] << 24) | (buffer[6] << 16) | (buffer[7] << 8) | buffer[8];

      // Converte de volta para float/double (dividindo por 1 milhão)
      float lat = lat32 / 1000000.0;
      float lon = lon32 / 1000000.0;

      Serial.print("[GPS] Latitude: ");
      Serial.print(lat, 6);
      Serial.print(" | Longitude: ");
      Serial.println(lon, 6);
      break;
    }

    // --- FATIAS DO ARQUIVO DO CARTÃO SD ---
    case CMD_BAIXAR_SD: {
      // Como o CSV é texto puro, imprimimos direto na serial com o prefixo
      // para o script Python capturar
      Serial.print("[SD_DATA]");
      for (int i = 1; i < size; i++) {
        Serial.print((char)buffer[i]);
      }
      break;
    }

    // --- FIM DO DOWNLOAD DO SD ---
    case CMD_FIM_ARQUIVO: {
      Serial.println("\n[BASE] ==================================");
      Serial.println("[BASE] Download do SD concluído.");
      Serial.println("[BASE] ==================================");
      break;
    }

    default:
      Serial.println("[BASE] Pacote de telemetria não reconhecido.");
      break;
  }
}

//========================================================================
// ENVIO DE COMANDOS
//========================================================================
void EnviaComandoLoRa(uint8_t comando) {
  uint8_t txBuffer[1];
  txBuffer[0] = comando;
  
  // Prepara e envia o frame usando o comando 0x28 como invólucro padrão da aplicação
  if (lora.PrepareFrameCommand(ID_FOGUETE, 0x28, txBuffer, 1)) {
    lora.SendPacket();
    Serial.println("[BASE] Pacote enviado.");
  } else {
    Serial.println("[BASE] Falha ao preparar o pacote.");
  }
}