/*
  Master.ino  -  ESTACAO BASE
  Biblioteca: LoRa_E220 (Alteriom fork) | Protocolo: LoRaProtocol.h
  v4.0 — Protocolo unificado (Response de tamanho fixo p/ qualquer mensagem),
         validacao de checksum (CRC8), dump binario do SD com deteccao de
         registros faltantes e reenvio automatico via CMD_RESEND_SD.
*/

#include "Arduino.h"
#include "LoRa_E220.h"
#include "LoRaProtocol.h"

#define PIN_RX   16
#define PIN_TX   17
#define PIN_M0   21
#define PIN_M1   19
#define PIN_AUX  18

#define MAX_REGISTROS_SD   5000  // limite de seguranca contra totalRegistros corrompido
#define MAX_TENTATIVAS_SD  3     // rodadas de reenvio automatico ao final do dump
#define TIMEOUT_REENVIO_MS 800   // tempo de espera por cada registro reenviado

LoRa_E220 Transceiver(&Serial2, PIN_AUX, PIN_M0, PIN_M1);

bool     recebendoSD      = false;
uint16_t sdTotalRegistros = 0;
bool*    sdRecebidos      = nullptr;
uint32_t ultimoComandoEnviado = 0;

void handleCommand(String cmd);
void enviarComando(uint8_t tipoCmd, uint16_t param = 0);
void processarResposta(const Response& r);
void resolverRegistrosFaltantes();
bool aguardarRegistro(uint16_t seqEsperado, uint32_t timeoutMs);
void liberarBufferSD();
void printHelp();

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial2.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);

  // --- Logs Visiveis para Depuracao Humana (Boot) ---
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
    // Toda mensagem SLAVE -> MASTER agora tem o MESMO tamanho fixo (sizeof(Response)),
    // nao importa se e telemetria, GPS ou um registro do dump do SD.
    ResponseStructContainer rsc = Transceiver.receiveMessage(sizeof(Response));

    if (rsc.status.code == E220_SUCCESS) {
      Response r; memcpy(&r, rsc.data, sizeof(Response));
      if (r.checksum == calcChecksum(r)) {
        processarResposta(r);
      } else {
        Serial.println(F("[LORA] Pacote com checksum invalido, descartado (possivel ruido de RF)"));
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
  else if (cmd == "SD")    { recebendoSD = true; liberarBufferSD(); enviarComando(CMD_DOWNLOAD_SD); }
  else if (cmd == "HELP")  printHelp();
}

void enviarComando(uint8_t tipoCmd, uint16_t param) {
  Command c; c.cmd = tipoCmd; c.param = param;
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
      Serial.print("{\"lat\":"); Serial.print(r.dados.telemetria.lat, 6);
      Serial.print(",\"lon\":"); Serial.print(r.dados.telemetria.lon, 6);
      Serial.print(",\"alt\":"); Serial.print(r.dados.telemetria.alt, 2);
      Serial.print(",\"speed\":"); Serial.print(r.dados.telemetria.speed, 2);
      Serial.print(",\"state\":\""); Serial.print(estadoStr);
      Serial.println("\"}");
      break;
    }

    case RESP_GPS:
      Serial.print("{\"lat\":"); Serial.print(r.dados.gps.lat, 6);
      Serial.print(",\"lon\":"); Serial.print(r.dados.gps.lon, 6);
      Serial.println("}");
      break;

    case RESP_PONG:
      Serial.println("{\"msg\": \"PONG recebido\"}");
      break;

    case RESP_SD_INICIO: {
      uint16_t total = r.dados.sdInicio.totalRegistros;
      if (total == 0 || total > MAX_REGISTROS_SD) {
        Serial.println(F("[SD] total de registros invalido/corrompido, abortando download"));
        recebendoSD = false;
        break;
      }
      liberarBufferSD();
      sdTotalRegistros = total;
      sdRecebidos = new bool[sdTotalRegistros](); // zero-initialized
      Serial.print(F("[SD] Iniciando download: ")); Serial.print(sdTotalRegistros);
      Serial.println(F(" registros esperados"));
      break;
    }

    case RESP_SD_RECORD: {
      uint16_t seq = r.dados.sdRegistro.seq;
      if (sdRecebidos != nullptr && seq < sdTotalRegistros) {
        if (!sdRecebidos[seq]) {
          sdRecebidos[seq] = true;
          Serial.print(F("[SD] Registro ")); Serial.print(seq + 1);
          Serial.print("/"); Serial.print(sdTotalRegistros); Serial.println(F(" recebido"));
        }
      } else {
        Serial.print(F("[SD] Registro com seq fora do esperado (")); Serial.print(seq); Serial.println(F("), ignorado"));
      }
      break;
    }

    case RESP_SD_FIM:
      Serial.println(F("[SD] Fim do stream recebido. Verificando registros faltantes..."));
      resolverRegistrosFaltantes();
      recebendoSD = false;
      liberarBufferSD();
      Serial.println("{\"msg\": \"Download do SD concluido\"}");
      break;
  }
}

// --------------------------------------------------------------------------
// Confiabilidade: detecta buracos (registros nao recebidos) e pede reenvio
// pontual via CMD_RESEND_SD, em ate MAX_TENTATIVAS_SD rodadas.
// --------------------------------------------------------------------------
void resolverRegistrosFaltantes() {
  if (sdRecebidos == nullptr || sdTotalRegistros == 0) return;

  for (uint8_t tentativa = 1; tentativa <= MAX_TENTATIVAS_SD; tentativa++) {
    uint16_t faltando = 0;

    for (uint16_t seq = 0; seq < sdTotalRegistros; seq++) {
      if (sdRecebidos[seq]) continue;
      faltando++;

      Serial.print(F("[SD] Tentativa ")); Serial.print(tentativa);
      Serial.print(F(" - solicitando reenvio do registro ")); Serial.println(seq);

      enviarComando(CMD_RESEND_SD, seq);
      aguardarRegistro(seq, TIMEOUT_REENVIO_MS); // se chegar, marca sdRecebidos[seq] via processarResposta
    }

    if (faltando == 0) {
      Serial.println(F("[SD] Todos os registros recebidos, nenhum reenvio necessario."));
      break;
    }
    Serial.print(F("[SD] Fim da tentativa ")); Serial.print(tentativa);
    Serial.print(F(": ainda faltam ")); Serial.print(faltando); Serial.println(F(" registro(s)"));
  }

  uint16_t recebidosFinal = 0;
  for (uint16_t i = 0; i < sdTotalRegistros; i++) if (sdRecebidos[i]) recebidosFinal++;

  Serial.print(F("[SD] Resultado final: ")); Serial.print(recebidosFinal);
  Serial.print("/"); Serial.print(sdTotalRegistros);
  Serial.print(F(" registros ("));
  Serial.print((recebidosFinal * 100.0) / sdTotalRegistros, 1);
  Serial.println(F("%)"));

  if (recebidosFinal < sdTotalRegistros) {
    Serial.println(F("[SD] AVISO: alguns registros nao puderam ser recuperados apos todas as tentativas."));
  }
}

// Espera bloqueada por um registro especifico (usado so durante o reenvio pontual,
// que ja ocorre fora da rota critica de voo). Processa qualquer pacote que chegar
// nesse meio tempo, entao telemetria fora de ordem tambem e tratada normalmente.
bool aguardarRegistro(uint16_t seqEsperado, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (Transceiver.available() > 0) {
      ResponseStructContainer rsc = Transceiver.receiveMessage(sizeof(Response));
      if (rsc.status.code == E220_SUCCESS) {
        Response r; memcpy(&r, rsc.data, sizeof(Response));
        rsc.close();
        if (r.checksum != calcChecksum(r)) continue;
        processarResposta(r);
        if (r.resp == RESP_SD_RECORD && r.dados.sdRegistro.seq == seqEsperado) {
          return true;
        }
      } else {
        rsc.close();
      }
    }
  }
  return false;
}

void liberarBufferSD() {
  if (sdRecebidos != nullptr) { delete[] sdRecebidos; sdRecebidos = nullptr; }
  sdTotalRegistros = 0;
}

void printHelp() {
  Serial.println(F("\n--- Comandos via Monitor Serial ---"));
  Serial.println(F(" READ  -> Pede Telemetria | GPS   -> Pede Coordenadas"));
  Serial.println(F(" ARMAR -> Forca Voo       | RESET -> Zera Altimetro"));
  Serial.println(F(" SD    -> Baixa Log       | PING  -> Testa Conexao"));
  Serial.println(F(" HELP  -> Menu de Comandos"));
}
