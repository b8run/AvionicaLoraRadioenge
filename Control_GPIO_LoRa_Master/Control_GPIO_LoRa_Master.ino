/*
  Master.ino  -  ESTAÇÃO BASE
  Biblioteca: LoRa_E220 (Alteriom fork) | Protocolo: LoRaProtocol.h
  v3.4 — Logs Iniciais de Depuração + Saída JSON
*/

#include "Arduino.h"
#include "LoRa_E220.h"
#include "LoRaProtocol.h"

#define PIN_RX   16
#define PIN_TX   17
#define PIN_M0   21
#define PIN_M1   19
#define PIN_AUX  18

LoRa_E220 Transceiver(&Serial2, PIN_AUX, PIN_M0, PIN_M1);

bool recebendoSD = false;
uint32_t ultimoComandoEnviado = 0;

void handleCommand(String cmd);
void enviarComando(uint8_t tipoCmd);
void processarResposta(const Response& r);
void printHelp();

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial2.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);

  // --- Logs Visíveis para Depuração Humana (Boot) ---
  Serial.println(F("========================================"));
  Serial.println(F("       ===== MASTER (BASE) ====="));
  Serial.println(F("========================================"));
  Serial.print(F("Tamanho do Pacote (Command): ")); Serial.print(sizeof(Command)); Serial.println(F(" bytes"));
  Serial.print(F("Tamanho do Pacote (Response): ")); Serial.print(sizeof(Response)); Serial.println(F(" bytes"));
  
  bool ok = Transceiver.begin();
  Serial.print(F("Inicializacao LoRa E220 -> "));
  Serial.println(ok ? F("OK") : F("FALHA"));
  
  printHelp();
  Serial.println(F("----------------------------------------"));
  Serial.println(F("Aguardando pacotes ou comandos..."));
  // ---------------------------------------------------
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    handleCommand(input);
  }

  if (Transceiver.available() > 0) {
    ResponseStructContainer rsc = Transceiver.receiveMessage(sizeof(Response));

    if (rsc.status.code == E220_SUCCESS) {
      Response r;
      memcpy(&r, rsc.data, sizeof(Response));
      if (r.resp >= RESP_DATA && r.resp <= RESP_SD_FIM) {
        processarResposta(r);
      }
    }
    rsc.close();
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if      (cmd == "READ")  enviarComando(CMD_READ);
  else if (cmd == "PING")  enviarComando(CMD_PING);
  else if (cmd == "ARMAR") enviarComando(CMD_ARMAR);
  else if (cmd == "RESET") enviarComando(CMD_RESET_BASE);
  else if (cmd == "GPS")   enviarComando(CMD_GET_GPS);
  else if (cmd == "SD")    { recebendoSD = true; enviarComando(CMD_DOWNLOAD_SD); }
  else if (cmd == "HELP")  printHelp();
}

void enviarComando(uint8_t tipoCmd) {
  Command c; c.cmd = tipoCmd;
  Transceiver.sendMessage(&c, sizeof(c));
  ultimoComandoEnviado = millis();
}

void processarResposta(const Response& r) {
  switch (r.resp) {
    case RESP_DATA: {
      String estadoStr = "AGUARDANDO";
      if (r.contador == 1) estadoStr = "SUBIDA";
      else if (r.contador == 2) estadoStr = "DESCIDA";
      else if (r.contador == 3) estadoStr = "POUSADO";

      // Formata o pacote JSON direto para o frontend
      Serial.print("{\"lat\":"); Serial.print(r.lat, 6);
      Serial.print(",\"lon\":"); Serial.print(r.lon, 6);
      Serial.print(",\"alt\":"); Serial.print(r.alt, 2);
      Serial.print(",\"speed\":"); Serial.print(r.speed, 2);
      Serial.print(",\"state\":\""); Serial.print(estadoStr);
      Serial.println("\"}");
      break;
    }

    case RESP_GPS:
      Serial.print("{\"lat\":"); Serial.print(r.lat, 6);
      Serial.print(",\"lon\":"); Serial.print(r.lon, 6);
      Serial.println("}");
      break;

    case RESP_PONG:
      Serial.println("{\"msg\": \"PONG recebido\"}");
      break;

    case RESP_SD_FIM:
      Serial.println("{\"msg\": \"Download do SD concluido\"}");
      recebendoSD = false;
      break;
  }
}

void printHelp() {
  Serial.println(F("\n--- Comandos via Monitor Serial ---"));
  Serial.println(F(" READ  -> Pede Telemetria | GPS   -> Pede Coordenadas"));
  Serial.println(F(" ARMAR -> Forca Voo       | RESET -> Zera Altimetro"));
  Serial.println(F(" SD    -> Baixa Log       | PING  -> Testa Conexao"));
  Serial.println(F(" HELP  -> Menu de Comandos"));
}
