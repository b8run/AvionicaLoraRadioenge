#ifndef LORA_PROTOCOL_H
#define LORA_PROTOCOL_H

#include <Arduino.h>

// ---- Comandos (MASTER -> SLAVE) ----
enum CmdType : uint8_t {
  CMD_READ        = 1, // Pede telemetria
  CMD_PING        = 2, 
  CMD_ARMAR       = 3, // Força estado SUBIDA
  CMD_RESET_BASE  = 4, // Zera baseline barométrica
  CMD_GET_GPS     = 5, // Solicita Coordenadas
  CMD_DOWNLOAD_SD = 6  // Inicia Dump do Cartão SD
};

// ---- Respostas (SLAVE -> MASTER) ----
enum RespType : uint8_t {
  RESP_DATA       = 1, // Retorna Telemetria (Alt, Vel, Estado)
  RESP_PONG       = 2,
  RESP_GPS        = 3, // Retorna GPS (Lat, Lon)
  RESP_SD_FIM     = 4  // Sinaliza fim do download do SD
};

// ---- Struct MASTER -> SLAVE ----
struct __attribute__((packed)) Command {
  uint8_t cmd;
};

// ---- Struct SLAVE -> MASTER ----
struct __attribute__((packed)) Response {
  uint8_t  resp;       
  uint32_t timestamp;  
  float    alt;        // Substituiu valor1 original
  float    speed;      // Substituiu valor2 original
  float    lat;        // Adicionado para frontend
  float    lon;        // Adicionado para frontend
  uint16_t contador;   // Usado para enviar o Estado do Voo
};

#endif
