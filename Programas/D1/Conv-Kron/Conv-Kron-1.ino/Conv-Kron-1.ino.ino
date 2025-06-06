//************************************************************************************************************
//                                                                                                           *
//               Conversor MODBUS RTU Kron para UDP em Rede WiFi usando o Controlador ESP8266                *
//                                                                                                           *
//************************************************************************************************************
//
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

// Configurações de Comunicação IP e WiFi
const char *rede1 = "CasaVermelha";
const char *senha1 = "Vlg270672.";
const char *rede2 = "CasaVerde";
const char *senha2 = "luccasofia";
IPAddress local_IP(192,168,0,175);
IPAddress gateway(192,168,0,6);
IPAddress subnet(255,255,0,0);
IPAddress primaryDNS(192,168,1,1);
IPAddress secondaryDNS(192,168,0,6);

// Configurações e Variáveis de Comunicação MQTT para Comunicação com o Broker
WiFiClient espClient;
PubSubClient client(espClient);
const char *mqtt_broker = "200.98.140.180";
const char *topic = "MULTKPLUS";
const char *mqtt_username = "usuario1";
const char *mqtt_password = "senha1";
const int  mqtt_port = 1883;
bool mqttStatus = false;
bool connectMQTT();                                              // Protótipo de Função
void callback(char *topic, byte *payload, unsigned int length);  // Protótipo de Função
byte MsgBytes[128];                                              // Buffer com a Mensagem a ser enviada para o Broker

// Configurações de Comunicação UDP para comunicação em protocolo CoAP com o Gateway
WiFiUDP Udp;
unsigned int localPort = 5683;
byte TxBufEst[256];
byte RxBuffer[64];
unsigned int MsgCoAPId;
bool Modo_UTR_UDP = true;

// Configurações de Comunicação da Interface Serial Assíncrona de Comunicação com o Multimedidor Kron Mult-K Plus
EspSoftwareSerial::UART myPort;
byte MsgTxS1[16];               // Buffer para Envio da Mensagem de Requisicao
byte MsgRxS1[256];              // Buffer para Recebimento da Mensagem de Dados
unsigned int NumBytesMsgSerRx;  // Número de Bytes da Mensagem Modbus Serial Recebida
unsigned int NumBytesMsgRecEsperados;

// Definicao das constantes de conversao dos valores analogicos
#define KMed00        1.0  // Constante da Medida00
#define OffMed00      0.0  // Offset da Medida00

// Definicao dos numeros dos pinos para leitura das Entradas Analogicas (EA)
#define PinoEA0         0  // EA0

// Pinos da Interface Serial para Comunicação com o Multimedidor Kron
#define MYPORT_TX      12  // Tx (Conectar o pino D12/MISO/D6 da Placa ao Pino 4 (D) do CI Driver 75176)
#define MYPORT_RX      13  // Rx (Conectar o pino D11/MOSI/D7 da Placa ao Pino 1 (R) do CI Driver 75176)
#define HabTx485        0  // Controle Driver RS485 (0 = Rec / 1 = Trans - Conectar o Pino D8 da Placa aos Pinos DE e /RE do 75176)

// Definição dos números dos pinos de controle dos LEDs da placa
#define LED1            2  // Led indicador de comunicação com o servidor em protocolo UDP
#define LED2           14  // Led indicador de conexão à rede WiFi

#define BaseTempo  999673  // 1000000

int EA0 = 0;
byte ED0 = 0;
byte ED1 = 0;
byte ED2 = 0;
byte ED3 = 0;

// Relogio
byte Hora;
byte Minuto;
byte Segundo;
byte Dia;
byte Mes;
int Ano;

byte HoraRec;
byte MinutoRec;
byte SegundoRec;
byte DiaRec;
byte MesRec;
int AnoRec;

// Flag que indica Acerto de Relogio pelo Host
byte AcertaRelogio;



//************************************************************************************************************
//                                                                                                           *
//                                      Procedimentos de Inicialização                                       *
//                                                                                                           *
//************************************************************************************************************
//
void setup() {

  // Definir a Funcao dos Pinos de Saida Digital
  pinMode(HabTx485, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED1, OUTPUT);
  
  // Inicia a saÍda digital do controle do Driver RS485 em nível 0 (Recepção)
  digitalWrite(HabTx485, LOW);
  
  // Inicia as saídas de controle dos LEDs da placa
  digitalWrite(LED2, LOW);
  digitalWrite(LED1, HIGH);
  
  AtualizaEstadosMedidas();
 
  Serial.begin(115200);

  myPort.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  if (!myPort) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }

  // Configuração do Endereço IP Estático, Gateway e DNS
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Falha na Configuração do Endereço IP Estático");
  }

  ConectaWiFi();          // Tenta conectar em uma Rede WiFi

  if (Modo_UTR_UDP) {     // Se o modo de operação da UTR é UDP, espera mensagem de requisição
    Udp.begin(localPort);
    Serial.println();
    Serial.printf("Aguardando Mensagens UDP no endereço IP %s, na Porta UDP %d\n", WiFi.localIP().toString().c_str(), localPort);
  }
  else {                  // Se o modo de operação da UTR é MQTT, tenta conectar ao Broker
    mqttStatus = connectMQTT();
  }

}


//************************************************************************************************************
//                                                                                                           *
//                                          Programa Principal                                               *
//                                                                                                           *
//************************************************************************************************************
//
void loop() {

  if (WiFi.status() == WL_CONNECTED) {  // Se a UTR está conectada na Rede WiFi,
    digitalWrite(LED2, HIGH);           // acende o LED indicando conexão WiFi OK,
    AtualizaEstadosMedidas();           // atualiza os estads e medidas físicas,
    if (Modo_UTR_UDP) {                 // e se o modo de operação da UTR é UDP,
      ComunicacaoEthernet();            // verifica se foi recebida mensagem de requisição.
      delay(50);                        // Espera 50ms
      digitalWrite(LED1, HIGH);         // Apaga o LED indicador de comunicação OK com o Multimedidor Kron
    }
    else {                              // Se o modo de operação da UTR é MQTT
      byte endereco = 1;                // Efetua a comunicação com o Multimedidor Kron,
      byte funcao = 4;
      byte RegInic = 16;
      byte NumReg = 44;
      NumBytesMsgRecEsperados = (NumReg * 2) + 5;
      ComunicacaoSerial(endereco, funcao, RegInic, NumReg);

      if (NumBytesMsgSerRx == NumBytesMsgRecEsperados) {   // Se o Multimedidor Respondeu Corretamente,
        digitalWrite(LED1, LOW);                           // acende o LED indicando,
        client.publish(topic, MsgRxS1, NumBytesMsgSerRx);  // e envia a mensagem recebida do Multimedidor para o Broker
      }
      else {                                               // Se o Multimedidor não respondeu,
        byte MsgErr[4];
        MsgErr[0] = 0;
        MsgErr[1] = 0x55;
        MsgErr[2] = 0xAA;
        MsgErr[3] = 0xFF;                                  // monta uma mensagem de erro,
        client.publish(topic, MsgErr, 4);                  // e envia para o Broker.
      }
      delay(200);
      digitalWrite(LED1, HIGH);                            // Apaga o LED indicador de comunicação OK com o Multimedidor Kron
      delay(2800);
    }
  }
  else {
    digitalWrite(LED2, LOW);
    delay(50);
  }
}


//*******************************************************************************************************
// Nome da Rotina: AtualizaEstadosMedidas()                                                             *
//                                                                                                      *
// Funcao: carrega os estados das entradas digitais e carrega e calcula a medida                        *
//                                                                                                      *
// Entrada: nenhum                                                                                      *
//                                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void AtualizaEstadosMedidas() {

  //ED0 = digitalRead(PinoED0);
  //ED1 = digitalRead(PinoED1);
  EA0 = (KMed00 * analogRead(PinoEA0)) + OffMed00;
    
}


//*******************************************************************************************************
// Nome da Rotina: ComunicacaoEthernet                                                                  *
//                                                                                                      *
// Funcao: verifica se tem requisicao pela interface ethernet (UDP), executa o comando recebido         *
//         e envia a mensagem de resposta.                                                              *
//                                                                                                      *
// Entrada: nenhum                                                                                      *
//                                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void ComunicacaoEthernet() {

  byte Comando = 0; 
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {                     // Verifica se chegou mensagem UDP do Dispositivo Mestre,
    IPAddress remote = Udp.remoteIP();      // Se chegou mensagem, obter o endereco IP do Dispositivo Mestre,
    Udp.read(RxBuffer, 255);                // e ler o pacote de dados recebido.

    if ((RxBuffer[0] == 0x40) && (RxBuffer[1] == 0x01)) {      // Se recebeu um cabeçalho CoAP válido, lê os dados do Payload
      MsgCoAPId = DoisBytesParaInt(RxBuffer[3], RxBuffer[2]);  // Le o Identificador da Mensagem
      Comando = RxBuffer[7];                                   // Le o Byte de Comando
      Hora = RxBuffer[8];                                      // Le a Hora
      Minuto = RxBuffer[9];                                    // Le o Minuto
      Segundo = RxBuffer[10];                                  // Le o Segundo
      Dia = RxBuffer[11];                                      // Le o Dia
      Mes = RxBuffer[12];                                      // Le o Mes
      Ano = 256*RxBuffer[13] + RxBuffer[14];                   // Le o Ano
      
      Serial.println("");
      Serial.println("Recebida Mensagem do Dispositivo Mestre");
      ExecutaComando(Comando);
      Comando = 0;
      
      byte endereco = 1;
      byte funcao = 4;
      byte RegInic = 16;
      byte NumReg = 44;
      int resultado = ComunicacaoSerial(endereco, funcao, RegInic, NumReg);
      if (resultado == 0) {
        MontaMsgEstados((NumReg * 2) + 5);
      }
      else{
        MontaMsgEstados(0);
      }
      
    }
    
  } // if (packetSize)
    
} // Fim da Rotina ComunicacaoEthernet


//***************************************************************************************************
// Nome da Rotina: ExecutaComando                                                                   *
//                                                                                                  *
// Funcao: executa um comando recebido em uma mensagem CoAP UDP                                     *
//                                                                                                  *
// Entrada: codigo de identificacao do comando                                                      *
//                                                                                                  *
// Saida: nenhum                                                                                    *
//***************************************************************************************************
//
void ExecutaComando(byte codigo) {

  switch (codigo) {
   
    case 1: // Resposta a Solicitacao de Leitura de Estados
      
    break;

    case  2:
      //digitalWrite(PinoSD0,LOW);
      //digitalWrite(LED1,LOW);
    break;

    case  3:
      //digitalWrite(PinoSD0,HIGH);
      //digitalWrite(LED1,HIGH);
    break;

    case  4:
      //digitalWrite(PinoSD1,LOW);
    break;

    case  5:
      //digitalWrite(PinoSD1,HIGH);
    break;

  }
  
} // Fim da rotina ExecutaComando


//*******************************************************************************************************
// Nome da Rotina: MontaMsgEstados                                                                      *
//                                                                                                      *
// Funcao: monta e transmite a mensagem de resposta com os estados da UTR                               *
//                                                                                                      *
// Entrada: codigo do comando que esta mensagem foi usada como resposta                                 *
//                                                                                                      *
// Saida: preenche o array TxBufEst e transmite pela ethernet em protocolo UDP                          *
//*******************************************************************************************************
//
void MontaMsgEstados(byte NumBytesRec) {

  // Cabecalho da Mensagem CoAP
  TxBufEst[0] = 0x60;
  TxBufEst[1] = 0x45;
  TxBufEst[2] = ByteHigh(MsgCoAPId);
  TxBufEst[3] = ByteLow(MsgCoAPId);
  TxBufEst[4] = 0x0c1;
  TxBufEst[5] = 0x2a;
  TxBufEst[6] = 0x0ff;
  TxBufEst[7] = NumBytesRec;

  // Payload da Mensagem CoAP
  TxBufEst[8] = Hora;            // Relógio - Hora
  TxBufEst[9] = Minuto;          // Relógio - Minuto
  TxBufEst[10] = Segundo;        // Relógio - Segundo
  TxBufEst[11] = Dia;            // Relógio - Dia
  TxBufEst[12] = Mes;            // Relógio - Mês
  TxBufEst[13] = ByteHigh(Ano);  // Relógio - Ano
  TxBufEst[14] = ByteLow(Ano);   // Relógio - Ano
  TxBufEst[15] = 0;              // Reserva

  if (NumBytesRec > 8) {
    int NumReg = 44;
    int NumBytes = (NumReg * 2) + 5;
    for (int i = 0; i < NumBytes; i++) {
      TxBufEst[i + 16] = MsgRxS1[i];
    }
  }
  else {
    int NumReg = 44;
    int NumBytes = (NumReg * 2) + 5;
    for (int i = 0; i < NumBytes; i++) {
      TxBufEst[i + 16] = 0;
    }
  }
  
  TxBufEst[126] = 127;
  TxBufEst[127] = 255;

  // Transmite a mensagem para o cliente
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(TxBufEst, 128);
  Udp.endPacket();

  Serial.println("Enviada Mensagem de Resposta CoAP UDP para o Mestre");

} // Fim da Rotina MontaMsgEstados()


//*************************************************************************************************************************
// Nome da Rotina: ComunicacaoSerial                                                                                      *
//                                                                                                                        *
// Funcao: envia a requisicao pela interface Serial RS485 em protocolo moDBUS RTU e recebe a mensagem de resposta do      *
//         Multimedidor.                                                                                                  *
//                                                                                                                        *
// O formato da mensagem de requisição para as funções 3 e 4 para leitura de uma medida é mostrado a seguir:              *
//                                                                                                                        *
//         | Byte 00: endereço ( 1 )                                                                                      *
//         | Byte 01: funçao ( 3 ou 4 )                                                                                   *
//         | Byte 02: Registro Inicial - MSB ( 0 )                                                                        *
//         | Byte 03: Registro Inicial - LSB - ver tabela de medidas e parâmetros                                         *
//         | Byte 04: Número de Registros - MSB ( 0 )                                                                     *
//         | Byte 05: Número de Registros - LSB ( 2 ) - cada medida ou parâmetro de 32 bits ocupa 2 registros             *
//         | Byte 06: CRC16                                                                                               *
//         | Byte 07: CRC16                                                                                               *
//                                                                                                                        *
// O formato da mensagem de resposta para as funções 3 e 4 para leitura de uma medida é mostrado a seguir:                *
//                                                                                                                        *
//         | Byte 00: Endereço ( 1 )                                                                                      *
//         | Byte 01: Funçao ( 3 ou 4 )                                                                                   *
//         | Byte 02: Número de Bytes de Dados ( 4 )                                                                      *
//         | Byte 03: Primeiro Registro - MSB ( campo1 = F2 )                                                             *
//         | Byte 04: Primeiro Registro - LSB ( campo2 = F1 )                                                             *
//         | Byte 05: Segundo Registro - MSB  ( campo3 = F0 )                                                             *
//         | Byte 06: Segundo Registro - LSB  ( campo4 = EXP )                                                            *
//         | Byte 07: CRC                                                                                                 *
//         | Byte 08: CRC                                                                                                 *
//                                                                                                                        *
// O formato da mensagem de requisição para a função 16 para escrita de um parâmetro é mostrado a seguir:                 *
//                                                                                                                        *
//         | Byte 00: endereço ( 1 )                                                                                      *
//         | Byte 01: funçao ( 16 )                                                                                       *
//         | Byte 02: Registro Inicial - MSB    ( campo1 = 0 )                                                            *
//         | Byte 03: Registro Inicial - LSB    ( campo2 = 0 (RTP), campo2 = 2 (RTC) )                                    *
//         | Byte 04: Número de Registros - MSB ( campo3 = 0 )                                                            *
//         | Byte 05: Número de Registros - LSB ( campo4 = 2 )   - cada medida ou parâmetro de 32 bits ocupa 2 registros  *
//         | Byte 06: Número de Bytes de Dados  ( campo5 = 4 )   - cada medida ou parâmetro de 32 bits ocupa 4 bytes      *
//         | Byte 07: Primeiro Registro - MSB   ( campo6 = F2 )  - valor IEEE754 a programar                              *
//         | Byte 08: Primeiro Registro - LSB   ( campo7 = F1 )  - valor IEEE754 a programar                              *
//         | Byte 09: Segundo Registro - MSB    ( campo8 = F0 )  - valor IEEE754 a programar                              *
//         | Byte 10: Segundo Registro - LSB    ( campo9 = EXP ) - valor IEEE754 a programar                              *
//         | Byte 11: CRC                                                                                                 *
//         | Byte 12: CRC                                                                                                 *
//                                                                                                                        *
// O formato da mensagem de resposta para a função 16 para escrita de um parâmetro é mostrado a seguir:                   *
//                                                                                                                        *
//         | Byte 00: Endereço ( 1 )                                                                                      *
//         | Byte 01: Funçao ( 16 )                                                                                       *
//         | Byte 02: Primeiro Registro a Programar - MSB                                                                 *
//         | Byte 03: Primeiro Registro a Programar - LSB ( 0 => RTP, 2 => RTC )                                          *
//         | Byte 04: Número de Registros a Programar - MSB ( 0 )                                                         *
//         | Byte 05: Número de Registros a Programar - LSB ( 2 )                                                         *
//         | Byte 07: CRC                                                                                                 *
//         | Byte 08: CRC                                                                                                 *
//                                                                                                                        *
// Entrada: parametros de requisicao do protocolo MODBUS RTU                                                              *
//                                                                                                                        *
// Saida: 0 = Mensagem Recebida Ok / 1 = Mensagem Recebida com Erro de CRC / 2 = O Multmedidor não respondeu              *
//*************************************************************************************************************************
//
int ComunicacaoSerial(byte endereco, byte funcao, byte RegInic, byte NumReg) {

  byte NBMsgT = 8;      // Numero de Bytes da Mensagem de Requisicao
  byte TmoIED;
  byte ch;
  boolean CRC_OK = false;
  byte resultado = 2;
  
  unsigned int CRC = 0xffff;
  byte lsb;

  // Carrega a Mensagem de Requisicao no Buffer de Transmissao
  MsgTxS1[0] = endereco;       // Endereco do Multimedidor
  MsgTxS1[1] = funcao;         // Funcao
  MsgTxS1[2] = 0;
  MsgTxS1[3] = RegInic;
  MsgTxS1[4] = 0;
  MsgTxS1[5] = NumReg;

  unsigned int NumBytesMsgRx = (NumReg * 2) + 5;  // Número de Bytes que a Mensagem de Resposta do Multimedidor deve ter

  // Calcula o CRC16 e carrega nos ultimos dois bytes da mensagem
  for (int j = 0; j < (NBMsgT - 2); j++) {
    CRC = CRC ^ MsgTxS1[j];
    for (int i = 0; i < 8; i++) {
      lsb = CRC & 0x0001;
      if (lsb == 0){
        CRC = CRC / 2;
      }
      if (lsb == 1) {
        CRC = CRC / 2;
        CRC = CRC^0xA001;
      }
    }
  }

  MsgTxS1[NBMsgT - 2] = ByteLow(CRC); // CRC Low
  MsgTxS1[NBMsgT - 1] = ByteHigh(CRC); // CRC High

  myPort.flush();
  Serial.println("");
  Serial.print("Chama o Multimedidor - ");

  digitalWrite(HabTx485, HIGH);  // Habilita a transmissão da Serial RS485
  delay(10);                     // Espera 50ms para transmitir os bytes

  // Transmite os bytes da  Mensagem de Requisicao para o Multimedidor pela Serial RS485
  for (int i = 0; i < NBMsgT ; i++) {
    myPort.write(MsgTxS1[i]);
    delay(1);
  }

  delay(4);                      // Espera 50ms para desabilitar a transmissão RS485
  digitalWrite(HabTx485, LOW);   // Desabilita a transmissão da Serial RS485
  myPort.flush();
  
  // Le os bytes recebidos
  long cont = 0;
  unsigned int cntb = 0;
  while ((cntb < NumBytesMsgRx) && (cont < 50000)) {
    if (myPort.available() > 0) {
      MsgRxS1[cntb] = myPort.read();
      cntb = cntb + 1;
      cont = 0;
    }
    cont = cont + 1;
  }
  
  Serial.print("Recebida Mensagem do Multimedidor com = ");
  Serial.print(cntb);
  Serial.println(" Bytes");
  Serial.print("Mensagem do Multimedidor: ");
  for (int i = 0; i < cntb; i++) {
    Serial.print(MsgRxS1[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");

  if (cntb == NumBytesMsgRx) {

    // Calcula o CRC da mensagem recebida
    CRC = 0xffff;
    for (int j = 0; j < (cntb - 2); j++) {
      CRC = CRC ^ MsgRxS1[j];
      for (int i = 0; i < 8; i++) {
        lsb = CRC & 0x0001;
        if (lsb == 0){
          CRC = CRC / 2;
        }
        if (lsb == 1) {
          CRC = CRC / 2;
          CRC = CRC^0xA001;
        }
      }
    }
        
    // Verifica se a mensagem de resposta do Multimedidor chegou corretamente
    if ((MsgRxS1[cntb - 2] == ByteLow(CRC)) && (MsgRxS1[cntb - 1] == ByteHigh(CRC))) {
      Serial.println("CRC da Mensagem Recebida OK");
      digitalWrite(LED1, LOW);
      resultado = 0;
      myPort.flush();
    }
    else {
      Serial.println("Erro de CRC da Mensagem Recebida");
      resultado = 1;
      myPort.flush();
    }
  }
  
  if (cntb == 0) {
    Serial.println("O Multimedidor não Respondeu");
    resultado = 2;
    myPort.flush();
  }
  
  return(resultado);
  
} // Fim da Rotina ComunicacaoSerial()


//*******************************************************************************************************
// Nome da Rotina: ConectaWiFi                                                                          *
//                                                                                                      *
// Funcao: tenta fazer a conexão em duas redes WiFi                                                     *
//                                                                                                      *
// Entrada: não tem                                                                                     *
//                                                                                                      *
// Saida: não tem                                                                                       *
//*******************************************************************************************************
//
void ConectaWiFi() {
  int cont = 0;
  Serial.printf("Tentando conectar na rede WiFi %s ", rede1);
  Serial.println();
  WiFi.begin(rede1, senha1);
  while ((WiFi.status() != WL_CONNECTED) && (cont < 80)) {
    delay(50);
    cont++;
  }
  if (cont < 60) {
    Serial.printf(" Conectado à rede WiFi %s ", rede1);
    Serial.println();
    digitalWrite(LED2, HIGH);
  }
  else {
    cont = 0;
    Serial.printf("Tentando conectar na rede WiFi %s ", rede2);
    Serial.println();
    WiFi.begin(rede2, senha2);
    while ((WiFi.status() != WL_CONNECTED) && (cont < 80)) {
      delay(50);
      cont++;
    }
    if (cont < 60) {
      Serial.printf(" Conectado à rede WiFi %s ", rede2);
      Serial.println();
      digitalWrite(LED2, HIGH);
    }
  }
}


//*******************************************************************************************************
// Nome da Rotina: connectMQTT()                                                                        *
//                                                                                                      *
// Funcao: conecta a UTR8266 ao Broker                                                                  *
//                                                                                                      *
// Entrada: nenhum                                                                                      *
//                                                                                                      *
// Saida: true => a UTR está conectada ao Broker / false => falha ao tentar conectar ao Broker          *
//*******************************************************************************************************
//
bool connectMQTT() {

  byte tentativa = 0;
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  do {
    String client_id = "UTR01";
    
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.printf("Cliente %s conectado ao Broker\n", client_id.c_str());
      digitalWrite(LED2, HIGH);
    }
    else {
      Serial.print("Falha ao conectar: ");
      Serial.println(IdentificaErro(client.state()));
      Serial.println();
      Serial.print("Tentativa: ");
      Serial.println(tentativa);
      delay(2000);
    }
    tentativa++;
    
  } while(!client.connected() && tentativa < 5);

  return (client.connected());
  
}


//*******************************************************************************************************
// Nome da Rotina: callback                                                                             *
//                                                                                                      *
// Funcao: espera o recebimento de mensagem MQTT como Subscriber                                        *
//                                                                                                      *
// Entrada: tópico, ponteiro do payload com a mensagem recebida, número de bytes da mensagem recebida   *
//                                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Recebida mensagem no tópico ");
  Serial.print(topic);
  Serial.print(" : ");
  for (int i = 0; i < length; i++) {
    Serial.print(payload[i]);
    Serial.print(" ");
  }
  Serial.println();
  delay(100);
    
}


//*******************************************************************************************************
// Nome da Rotina: ByteHigh                                                                             *
// Funcao: obtem o byte mais significativo de um inteiro de dois bytes                                  *
// Entrada: valor inteiro                                                                               *
// Saida: byte mais significativo                                                                       *
//*******************************************************************************************************
//
unsigned int ByteHigh(unsigned int valor) {
  unsigned int BH = valor / 256;
  return(BH);
}

//*******************************************************************************************************
// Nome da Rotina: ByteLow                                                                              *
// Funcao: obtem o byte menos significativo de um inteiro de dois bytes                                 *
// Entrada: valor inteiro                                                                               *
// Saida: byte menos significativo                                                                      *
//*******************************************************************************************************
//
unsigned int ByteLow(unsigned int valor) {
  unsigned int BH = valor / 256;
  unsigned int BL = valor - (256 * BH);
  return(BL);
}

//*******************************************************************************************************
// Nome da Rotina: DoubleByteLow                                                                        *
// Funcao: obtem o byte menos significativo de um double na faixa de 0 a 16.777.216 (24 bits)           *
// Entrada: double                                                                                      *
// Saida: byte menos significativo                                                                      *
//*******************************************************************************************************
//
byte DoubleByteLow(double valor) {
  byte BH = valor / 65536;
  byte BM = (valor - (65536 * BH)) / 256;
  byte BL = valor - (65536 * BH) - (256 * BM);
  return(BL);
}

//*******************************************************************************************************
// Nome da Rotina: DoubleByteMid                                                                        *
// Funcao: obtem o byte intermediário de um double na faixa de 0 a 16.777.216 (24 bits)                 *
// Entrada: double                                                                                      *
// Saida: byte menos significativo                                                                      *
//*******************************************************************************************************
//
byte DoubleByteMid(double valor) {
  byte BH = valor / 65536;
  byte BM = (valor - (65536 * BH)) / 256;
  return(BM);
}

//*******************************************************************************************************
// Nome da Rotina: DoubleByteHigh                                                                       *
// Funcao: obtem o byte nais significativo de um double na faixa de 0 a 16.777.216 (24 bits)            *
// Entrada: double                                                                                      *
// Saida: byte menos significativo                                                                      *
//*******************************************************************************************************
//
byte DoubleByteHigh(double valor) {
  byte BH = valor / 65536;
  return(BH);
}

//*******************************************************************************************************
// Nome da Rotina: DoisBytesParaInt                                                                     *
// Funcao: converte dois bytes para um inteiro                                                          *
// Entrada: byte menos significativo, byte mais significativo                                           *
// Saida: valor inteiro                                                                                 *
//*******************************************************************************************************
//
int DoisBytesParaInt(byte BL, byte BH) {
  int ByteL = BL;
  int ByteH = BH;
  if (BL < 0) { ByteL = 256 + BL; }
  if (BH < 0) { ByteH = 256 + BH; }
  return (ByteL + 256*ByteH);
}


String IdentificaErro(int erro) {

  String NomeErro = "Não Indentificado";
  if (erro == -4) NomeErro = "MQTT_CONNECTION_TIMEOUT";     // the server didn't respond within the keepalive time
  if (erro == -3) NomeErro = "MQTT_CONNECTION_LOST";        // the network connection was broken
  if (erro == -2) NomeErro = "MQTT_CONNECT_FAILED";         // the network connection failed
  if (erro == -1) NomeErro = "MQTT_DISCONNECTED";           // the client is disconnected cleanly
  if (erro == 0) NomeErro = "MQTT_CONNECTED";               // the client is connected
  if (erro == 1) NomeErro = "MQTT_CONNECT_BAD_PROTOCOL";    // the server doesn't support the requested version of MQTT
  if (erro == 2) NomeErro = "MQTT_CONNECT_BAD_CLIENT_ID";   // the server rejected the client identifier
  if (erro == 3) NomeErro = "MQTT_CONNECT_UNAVAILABLE";     // the server was unable to accept the connection
  if (erro == 4) NomeErro = "MQTT_CONNECT_BAD_CREDENTIALS"; // the username/password were rejected
  if (erro == 5) NomeErro = "MQTT_CONNECT_UNAUTHORIZED";    // the client was not authorized to connect
  return(NomeErro);
  
}
