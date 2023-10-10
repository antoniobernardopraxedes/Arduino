//*************************************************************************************************************************
//                                                                                                                        *
//                                       Programa do Concentrador Arduino UNO                                             *
//                                                                                                                        *
// Autor: Antonio Bernardo de Vasconcellos Praxedes                                                                       *
//                                                                                                                        *
// Data: 18/07/2023                                                                                                       *
//                                                                                                                        *
// Funções: efetuar a comunicação ethernet com o servidor HTTP e o atualizador, efetuar a comunicação serial em protocolo *
//          MODBUS RTU com o multimedidor Kron Mult K 05, efetuar a leitura e conversão das medidas das entradas          *
//          analógicas. Ao receber uma mensagem do servidor HTTP ou do atualizador pela ethernet, o programa encaminha a  *
//          informação ao multimedidor, espera a resposta e responde ao servidor HTTP ou o atualizador pela ethernet.     *
//          O programa também envia na mensagem ethernet de resposta os valores das medidas lidas nas entrada analógicas. *
//                                                                                                                        *
//*************************************************************************************************************************

// Inclusao das bibliotecas usadas no programa
#include <TimerOne.h>
#include <TimeLib.h>
#include <SPI.h>                 // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <SoftwareSerial.h>
#include <Wire.h>

#define HabTx485             4   // 0 =  Recepção Serial RS485 / 1 = Transmissão Serial RS485

#define SDLED               13

// Definicao dos numeros dos pinos para leitura das Entradas Analogicas (EA)
#define EA0                  0   // EA0
#define EA1                  1   // EA1
#define EA2                  2   // EA2
#define EA3                  3   // EA3
#define EA4                  4   // EA4
#define EA5                  5   // EA5

#define BaseTempo         4167
#define NumAmostras        10


//**************************************************************************************************************************
//                                                                                                                         *
//                                                 Declaracao das Variaveis                                                *
//                                                                                                                         *
//**************************************************************************************************************************
//

// Array com os estados das Entradas Digitais (ED)
byte ED[8];

int Medida00 = 0;
int Medida01 = 0;
int Medida02 = 0;
int Medida03 = 0;
int Medida04 = 0;
int Medida05 = 0;

// Relogio
byte HoraRec;
byte MinutoRec;
byte SegundoRec;
byte DiaRec;
byte MesRec;
int AnoRec;

// Relogio recebido do Computador
byte HoraRsp;
byte MinutoRsp;
byte SegundoRsp;
byte DiaRsp;
byte MesRsp;
byte AnoRsp;

// Definir Endereco MAC e Endereco IP do Medidor
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF };
IPAddress ip(192,168,0,152);
unsigned int localPort = 5683;

// Variaveis da comunicacao ethernet
const unsigned int TamBufTx = 256;
unsigned int Indice = 0;
unsigned int Contador = 0;
byte BufSVCheio = 0;
byte HabAquisicao = 0;
byte MsgTr = 0;

byte TxBufEst[128];
byte Comando = 0;
int MsgCoAPId;

// buffers para enviar e receber dados pela interafce ethernet
char RxBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

tmElements_t tm;

// Define Interface Serial Virtual para comunicação com o multimedidor KRON
SoftwareSerial mySerial(2, 3); // Pino 2 = RX, Pino 3 = TX

// Buffers de Mensagens da Interface Serial1 de Comunicacao com o Multimedidor KRON
byte MsgTxS1[32];   // Buffer para Envio da Mensagem de Requisicao
byte MsgRxS1[64];   // Buffer para Recebimento da Mensagem de Dados

 
//****************************************************************************************************************************
//                                                                                                                           *
//                                               Procedimentos de Inicializacao                                              *
//                                                                                                                           *
//****************************************************************************************************************************
//
void setup() {

  // Definir a Funcao dos Pinos de Entrada Digital
  pinMode(HabTx485, OUTPUT);
      
  pinMode(SDLED, OUTPUT);
  
  digitalWrite(SDLED, LOW);     // Inicia o LED da placa apagado
  digitalWrite(HabTx485, LOW);  // Inicia a Serial RS485 em modo Recepção
  
  HoraRec = 0;
  MinutoRec = 0;
  SegundoRec = 0;
  DiaRec = 1;
  MesRec = 1;
  AnoRec = 2021;

  // Inicia as Interfaces Seriais Assíncronas
  Serial.begin(9600);
  mySerial.begin(9600);

  // Inicia a Ethernet e o UDP:
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

} // Fim dos Procedimentos de Inicializacao


//**************************************************************************************************************************
//                                                                                                                         *
//                                              Inicio do Programa Principal                                               *
//                                                                                                                         *
//**************************************************************************************************************************
//
void loop() {
  
  ComunicacaoEthernet();

  Aquisicao();

  //delay(50);
            
} // Final do Loop Principal do Programa


//**************************************************************************************************************************
// Nome da Rotina: Aquisicao                                                                                               *
// Funcao: aquisicao de medidas                                                                                            *
// Entradas: nenhuma                                                                                                       *
// Saidas: nenhuma                                                                                                         *
//**************************************************************************************************************************
//
void Aquisicao() {

  Medida00 = analogRead(EA0);
  Medida01 = analogRead(EA1);
  Medida02 = analogRead(EA2);
  Medida03 = analogRead(EA3);
  Medida04 = analogRead(EA4);
  Medida05 = analogRead(EA5);
      
} // fim da rotina Aquisicao


//*********************************************************************************************************************
//                                                                                                                    *
// Nome da Rotina: ComunicacaoEthernet                                                                                *
//                                                                                                                    *
// Funcao: verifica se tem requisicao pela interface ethernet (UDP), recebe a mensagem, encaminha para o              *
//         multimedidor em protocolo MODBUS RTU, espera a resposta do Multimedidor e envia os bytes da resposta       *
//         pela ethernet.                                                                                             *
//                                                                                                                    *
// Entrada: nenhum                                                                                                    *
//                                                                                                                    *
// Saida: nenhum                                                                                                      *
//                                                                                                                    *
//*********************************************************************************************************************
//
void ComunicacaoEthernet() {

  byte Comando = 0; 
  int packetSize = Udp.parsePacket();
  if (packetSize) {                             // Verifica se chegou mensagem UDP pela ethernet
    IPAddress remote = Udp.remoteIP();          // Se chegou mensagem, obter o endereco IP do cliente
    Udp.read(RxBuffer, UDP_TX_PACKET_MAX_SIZE); // e ler o pacote de dados recebido do cliente

    Serial.println("");
    Serial.println("Recebida Mensagem pela Ethernet");
    
    byte endereco = RxBuffer[8];
    byte funcao = RxBuffer[9];
    
    byte campo1 = RxBuffer[10];
    byte campo2 = RxBuffer[11];
    byte campo3 = RxBuffer[12];
    byte campo4 = RxBuffer[13];

    byte campo5 = RxBuffer[14];  // Os campos 5 a 9 somente são usados na função 16 para parametrizar as constantes do TP e do TC
    byte campo6 = RxBuffer[15];
    byte campo7 = RxBuffer[16];
    byte campo8 = RxBuffer[17];
    byte campo9 = RxBuffer[18];

    int Cntb = 16;

    Cntb = ComunicacaoSerial(endereco, funcao, campo1, campo2, campo3, campo4, campo5, campo6, campo7, campo8, campo9);
    
    MontaMsgEstados(Cntb);
    
  } // if (packetSize)
  
    
} // Fim da Rotina ComunicacaoEthernet

 
//*************************************************************************************************************************
//                                                                                                                        *
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
// Saida: número de bytes recebidos do Multimedidor. Se retornar zero, o Multimedidor não respondeu                       *
//                                                                                                                        *
//*************************************************************************************************************************
//
int ComunicacaoSerial(byte endereco, byte funcao, byte campo1, byte campo2, byte campo3, byte campo4, byte campo5, byte campo6, byte campo7, byte campo8, byte campo9) {

  byte NBMsgT = 8;      // Numero de Bytes da Mensagem de Requisicao
  byte NBMsgR = 100;    // Numero máximo de Bytes da Mensagem de Resposta Recebida
  byte TmoIED;
  byte Res = 0;
  byte ch;

  unsigned int CRC = 0xffff;
  byte lsb;

  // Carrega a Mensagem de Requisicao no Buffer de Transmissao
  MsgTxS1[0] = endereco;       // Endereco do Multimedidor
  MsgTxS1[1] = funcao;         // Funcao
  MsgTxS1[2] = campo1;
  MsgTxS1[3] = campo2;
  MsgTxS1[4] = campo3;
  MsgTxS1[5] = campo4;

  if (funcao == 16) {
    NBMsgT = 13;             // Numero de Bytes da Mensagem de Requisicao para a função 16
    MsgTxS1[6] = campo5;
    MsgTxS1[7] = campo6;
    MsgTxS1[8] = campo7;
    MsgTxS1[9] = campo8;
    MsgTxS1[10] = campo9;
    
  }
        
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

  Serial.println("");
  Serial.println("Chama o Multimedidor");

  delay(50);
  digitalWrite(HabTx485, HIGH);  // Habilita a transmissão da Serial RS485
  delay(10);                     // Espera 50ms para transmitir os bytes

  // Transmite os bytes da  Mensagem de Requisicao para o Multimedidor pela Serial RS485
  for (int i = 0; i < NBMsgT ; i++) {
    mySerial.write(MsgTxS1[i]);
    delay(1);
  }

  delay(4);                      // Espera 50ms para desabilitar a transmissão RS485
  digitalWrite(HabTx485, LOW);   // Desabilita a transmissão da Serial RS485

  // Le os bytes recebidos do Concentrador Arduino Uno
  int cont = 0;
  int cntb = 0;
  while ((cntb < NBMsgR) && (cont < 1000)) {
    if (mySerial.available() > 0) {
      MsgRxS1[cntb] = mySerial.read();
      cntb = cntb + 1;
      delay(2);
    }
    cont = cont + 1;
  }
  Serial.print("Numero de Bytes Recebidos do Multimedidor = ");
  Serial.println(cntb);

  if (cont < 1000) { // Se nao ocorreu TimeOut, 
    
    // Calcula o CRC da mensagem recebida
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
    byte CRC_OK = 0;
    if ((MsgRxS1[cntb - 2] == ByteLow(CRC)) && (MsgRxS1[cntb - 1] == ByteHigh(CRC))) {
      CRC_OK = 1;
    }

    if (CRC_OK == 1) {  // Se a mensagem recebida esta OK,
      Serial.println("Mensagem OK");
      Res = 1;                       // Sinaliza recebimento de mensagem do Multimedidor OK
    }
  }
  
  return(cntb);
  
} // Fim da Rotina ComunicacaoSerial()


//*******************************************************************************************************
//                                                                                                      *
// Nome da Rotina: MontaMsgEstados                                                                      *
//                                                                                                      *
// Funcao: monta e transmite a mensagem de resposta com a mensagem de resposta do Multimedidor          *
//                                                                                                      *
// Entrada: numero de bytes da mensagem recebida do Multimedidor                                        *
//                                                                                                      *
// Saida: preenche o array TxBufEst e transmite pela ethernet em protocolo UDP                          *
//                                                                                                      *
//*******************************************************************************************************
//
void MontaMsgEstados(int Cntb) {

  // Cabecalho da Mensagem: bytes 000 a 015
  TxBufEst[0] = 3;                 // Endereco do Concentrador Arduino
  TxBufEst[1] = 1;                 // Modelo da UTR (1 = Arduino UNO)
  TxBufEst[2] = 0;                 // Modo (AcertaRelogio=0 ou AcertaRelogio=1)
  TxBufEst[3] = ByteHigh(Cntb);
  TxBufEst[4] = ByteLow(Cntb);

  int i = 0;
  if (Cntb > 0) {
    for (i = 0; i < Cntb; i++) {
      TxBufEst[i + 8] = MsgRxS1[i];
    }
  }

  // Medida 00
  TxBufEst[32] = ByteLow(Medida00);
  TxBufEst[33] = ByteHigh(Medida00);
  
  // Medida 01
  TxBufEst[34] = ByteLow(Medida01);
  TxBufEst[35] = ByteHigh(Medida01);
  
  // Medida 02
  TxBufEst[36] = ByteLow(Medida02);
  TxBufEst[37] = ByteHigh(Medida02);
  
  // Medida 03
  TxBufEst[38] = ByteLow(Medida03);
  TxBufEst[39] = ByteHigh(Medida03);
  
  // Medida 04
  TxBufEst[40] = ByteLow(Medida04);
  TxBufEst[41] = ByteHigh(Medida04);
  
  // Medida 05
  TxBufEst[42] = ByteLow(Medida05);
  TxBufEst[43] = ByteHigh(Medida05);
  
  //int TamMsgRsp = Cntb + 8;
  int TamMsgRsp = 48;
  
  // Transmite a mensagem para o cliente
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(TxBufEst, TamMsgRsp);
  Udp.endPacket();
  
} // Fim da Rotina MontaMsgEstados()

//*******************************************************************************************************
// Nome da Rotina: ByteHigh                                                                             *
// Funcao: obtem o byte mais significativo de um inteiro de dois bytes                                  *
// Entrada: valor inteiro                                                                               *
// Saida: byte mais significativo                                                                       *
//*******************************************************************************************************
//
byte ByteHigh(unsigned int valor) {
  byte BH = valor / 256;
  return(BH);
}

//*******************************************************************************************************
// Nome da Rotina: ByteLow                                                                              *
// Funcao: obtem o byte menos significativo de um inteiro de dois bytes                                 *
// Entrada: valor inteiro                                                                               *
// Saida: byte menos significativo                                                                      *
//*******************************************************************************************************
//
byte ByteLow(unsigned int valor) {
  byte BH = valor / 256;
  byte BL = valor - (256 * BH);
  return(BL);
}

//*******************************************************************************************************
// Nome da Rotina: Centena                                                                              *
// Funcao: obtem a centena mais significativa de um inteiro na faixa de 0000 a 9999                     *
// Entrada: valor inteiro                                                                               *
// Saida: centena mais significativa                                                                    *
//*******************************************************************************************************
//
byte Centena(unsigned int valor) {
  byte CH = valor / 100;
  return(CH);
}

//*******************************************************************************************************
// Nome da Rotina: Dezena                                                                           *
// Funcao: obtem a centena menos significativa de um inteiro na faixa de 0000 a 9999                    *
// Entrada: valor inteiro                                                                               *
// Saida: centena menos significativa                                                                   *
//*******************************************************************************************************
//
byte Dezena(unsigned int valor) {
  byte CH = valor / 100;
  byte CL = valor - (100 * CH);
  return(CL);
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
