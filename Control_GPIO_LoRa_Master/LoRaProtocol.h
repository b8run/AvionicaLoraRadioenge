#ifndef LORA_PROTOCOL_H
#define LORA_PROTOCOL_H

#include <Arduino.h>

// ============================================================================
// PROTOCOLO LoRa - v4.0
// Mudanca principal em relacao a v3.4: toda mensagem SLAVE -> MASTER usa a
// MESMA struct (Response), de tamanho FIXO. Isso elimina o bug em que o
// download do SD (que mandava bytes crus de texto CSV, tamanho variavel)
// nao batia com o Transceiver.receiveMessage(sizeof(Response)) do Master,
// fazendo os pacotes do dump serem descartados silenciosamente.
//
// Confiabilidade adicionada:
//  - checksum (CRC8) em toda Response -> detecta corrupcao por ruido de RF
//  - seq (numero sequencial) em cada registro do SD -> detecta perda de pacote
//  - RESP_SD_INICIO informa quantos registros virao -> Master sabe o total
//    esperado e consegue identificar buracos (gaps) ao final do download
//  - CMD_RESEND_SD permite ao Master pedir o reenvio pontual de um registro
//    especifico (por seq), sem precisar reiniciar o dump inteiro
// ============================================================================

// ---- Comandos (MASTER -> SLAVE) ----
enum CmdType : uint8_t {
  CMD_READ        = 1, // Pede telemetria
  CMD_PING        = 2,
  CMD_ARMAR       = 3, // Forca estado SUBIDA
  CMD_RESET_BASE  = 4, // Zera baseline barometrica
  CMD_GET_GPS     = 5, // Solicita Coordenadas
  CMD_DOWNLOAD_SD = 6, // Inicia Dump do Cartao SD (binario)
  CMD_RESEND_SD   = 7  // Solicita reenvio de UM registro especifico (usa 'param')
};

// ---- Respostas (SLAVE -> MASTER) ----
enum RespType : uint8_t {
  RESP_DATA       = 1, // Telemetria (Alt, Vel, Estado)
  RESP_PONG       = 2,
  RESP_GPS        = 3, // GPS (Lat, Lon)
  RESP_SD_INICIO  = 4, // Anuncia quantos registros virao no dump
  RESP_SD_RECORD  = 5, // Um registro binario do voo (equivale a 1 linha do CSV)
  RESP_SD_FIM     = 6  // Sinaliza fim do stream do SD
};

// ---- Struct MASTER -> SLAVE ----
struct __attribute__((packed)) Command {
  uint8_t  cmd;
  uint16_t param;   // usado por CMD_RESEND_SD (numero do registro / seq). Ignorado nos demais.
};

// ---- Payload variavel, interpretado conforme o campo 'resp' ----
union __attribute__((packed)) RespPayload {
  struct __attribute__((packed)) {
    float alt, speed, lat, lon;
  } telemetria;                         // RESP_DATA

  struct __attribute__((packed)) {
    float lat, lon;
  } gps;                                // RESP_GPS

  struct __attribute__((packed)) {
    uint16_t totalRegistros;
  } sdInicio;                           // RESP_SD_INICIO

  struct __attribute__((packed)) {
    uint16_t seq;
    uint32_t tempo;
    uint8_t  estado;
    float    altitude, velocidade, accX, accY, accZ, lat, lon;
  } sdRegistro;                         // RESP_SD_RECORD
};

// ---- Struct SLAVE -> MASTER (tamanho FIXO para QUALQUER tipo de mensagem) ----
struct __attribute__((packed)) Response {
  uint8_t     resp;
  uint32_t    timestamp;
  uint16_t    contador;     // estado do voo (telemetria) ou nao usado nos outros tipos
  RespPayload dados;
  uint8_t     checksum;     // CRC8 sobre todos os bytes anteriores da struct
};
// Tamanho total (packed): 1 + 4 + 2 + 35 + 1 = 43 bytes.
// Verifique o payload maximo do seu modulo E220 (tipicamente 58~200 bytes)
// e ajuste se necessario -- 43 bytes cabe folgado na maioria dos modelos.

// ---- Registro binario gravado no cartao SD (usado para montar o dump) ----
struct __attribute__((packed)) LogLine {
  uint32_t tempo;
  uint8_t  estado;
  float    altitude, velocidade, accX, accY, accZ, lat, lon;
};

// ============================================================================
// CRC8 (polinomio 0x07) - identico nos dois lados para validar integridade
// ============================================================================
inline uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

inline uint8_t calcChecksum(const Response& r) {
  return crc8((const uint8_t*)&r, sizeof(Response) - sizeof(r.checksum));
}

#endif
