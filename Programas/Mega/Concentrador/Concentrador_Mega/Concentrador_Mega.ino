//*********************************************************************************************************************
//                                                                                                                    *
//        Programa do Concentrador da Usina de Geracao Solar com Arduíno Mega                                         *
//                                                                                                                    *
// Autor: Antonio Bernardo de Vasconcellos Praxedes                                                                   *
//                                                                                                                    *
// Data: 11/10/2023                                                                                                   *
//                                                                                                                    *
//*********************************************************************************************************************

// Inclusao das bibliotecas usadas no programa
#include <EEPROM.h>
#include <SPI.h>             // needed for Arduino versions later than 0018
#include <EthernetUdp.h>     // UDP library from: bjoern@cs.stanford.edu 12/30/2008
//#include <Rtc_Pcf8563.h>
#include <Wire.h>

#define DE1              3   // Saida de Controle para o Driver de Transmissao RS485 do CC1 (Ativo em 1)
#define RE1              4   // Saida de Habilitacao da Recepcao RS485 do CC1 (Ativo em 0)
#define DE2              5   // Saida de Controle para o Driver de Transmissao RS485 do CC2 (Ativo em 1)
#define RE2              6   // Saida de Habilitacao da Recepcao RS485 do CC2 (Ativo em 0)
#define ABaud       115200   // Baud Rate da Interface Serial 2
#define TETR           100   // Tempo em ms de Espera para Transmitir a Resposta pela Ethernet
#define TmpCPEMax    14400
#define KMed11     1.22727   // Constante da Medida 11
#define BTPP            50   // Base de Tempo do Ciclo de Execucao do Programa Principal (ms)
#define BaseTempo   999673   // 1000000
#define TamMsgTxCoAP   320   // Numero de Bytes da Mensagem de Resposta em Protocolo CoAP

//#define KIsIv1         0.7;  // Constante de Correcao do Valor da Corrente de Saida do Inversor 1


//***********************************************************************************************************************
//                                                                                                                      *
//                                            Declaracao das Variaveis                                                  *
//                                                                                                                      *
//***********************************************************************************************************************
//
// Constantes 
unsigned int EsperaByte = 11000000/ABaud;

// Medidas
double EA[16];        // Array com os Valores da Media das Entradas Analogicas
double EAT[16];       // Array com os acumuladores de calculo de media das entradas analogicas
double EAME0;         // Variavel com a Media estendida da Entrada Analogica 0 = Tensao 24Vcc Geral
double EAMET0;        // Variavel com o acumulador para calculo da media estendida da Tensao 24Vcc

// Medidas Gerais
double Icarga3;       // Corrente Carga 3 (Geladeira)
double VRede;         // Tensão da Rede
double VBat;          // Tensão do Banco de Baterias
double VMBat;         // Tensão Média Estendida do Banco de Baterias
double ICircCC;       // Corrente Total dos Circuitos CC
double WCircCC;       // Potência Total dos Circuitos CC
double ITotCg;        // Corrente Total Consumida pelas Cargas
double WTotCg;        // Potência Total Consumida pelas Cargas
double IFonteCgBat;   // Corrente de Saída da Fonte CC
double WFonteCC;      // Potência de Saída da Fonte CC
double IBat;          // Corrente de Carga / Descarga do Banco de Baterias
double WBat;          // Potência de Carga / Descarga do Banco de Baterias
double TBat;          // Temperatura do Banco de Baterias
double IFontesCC12;   // Corrente fornecida pelas fontes CC1 e CC2

// Medidas da UTR2 - Comunicação com os Controladores de Carga
double VP12;          // Medida 00: 0x3100 - PV array voltage 1
double IS12;          // Medida 01: 0x3101 - PV array current 1
double WS12;          // Medida 02: 0x3102 - PV array power 1
double VBat1;         // Medida 03: 0x3104 - Battery voltage 1
double ISCC1;         // Medida 04: 0x3105 - Battery charging current 1
double WSCC1;         // Medida 05: 0x3106 - Battery charging power 1
double TBat1;         // Temperatura do Banco de Baterias lida pelo Controlador de Carga 1
double VP34;          // Medida 08: 0x3100 - PV array voltage 2
double IS34;          // Medida 09: 0x3101 - PV array current 2
double WS34;          // Medida 10: 0x3102 - PV array power 2
double VBat2;         // Medida 11: 0x3104 - Battery voltage 2
double ISCC2;         // Medida 12: 0x3105 - Battery charging current 2
double WSCC2;         // Medida 13: 0x3106 - Battery charging power 2
double TBat2;         // Temperatura do Banco de Baterias lida pelo Controlador de Carga 2

// Medidas da Geração
double ITotGer;       // Corrente Total Gerada
double WTotGer;       // Potência Total Gerada

// Medidas do Inversor 2
double VSIv2;         // Tensão de Saída do Inversor 2
double IEIv2;         // Corrente de Entrada do Inversor 2
double WEIv2;         // Potência de Entrada do Inversor 2
double ISInv2;        // Corrente de Saída do Inversor 2
double WSInv2;        // Potência de Saída do Inversor 2
double TDInv2;        // Temperatura do Driver do Inversor 2
double TTInv2;        // Temperatura do Transformador do Inversor 2

// Medidas do Inversor 1
double VSIv1;         // Tensão de Saída do Inversor 1
double IEIv1;         // Corrente de Entrada do Inversor 1
double WEIv1;         // Potência de Entrada do Inversor 1
double ISInv1;        // Corrente de Saída do Inversor 1
double WSInv1;        // Potência de Saída do Inversor 1
double TDInv1;        // Temperatura do Driver do Inversor 1
double TTInv1;        // Temperatura do Transformador do Inversor 1

// Medidas Agua
double TmpBombaLig;   // Contador de Segundos da Bomba Ligada
double TmpCxAzNvBx;   // Contador de Segundos da Caixa Azul em Nivel Baixo

// Entradas Digitais Lidas da UTR Arduino Mega
byte DJEINV1;         // I (29) Disjuntor de Entrada 24Vcc do Inversor 1: Desligado = 0 / Ligado = 1
byte DJEINV2;         // I (30) Disjuntor de Entrada 24Vcc do Inversor 2: Desligado = 0 / Ligado = 1
byte ChLR;            // I (41) Chave Local = 0 / Remoto = 1 => (primeira chave de baixo para cima)
byte ChEN;            // I (42) Chave Economia = 0 / Normal = 1 => (segunda chave de baixo para cima)
byte ChMA;            // I (43) Chave Manual = 0 / Automatico = 1 => (terceira chave de baixo para cima)
byte ChPrB;           // I (44) Chave de Habilitacao de Cargas Posicao Baixo = 1
byte ChPrC;           // I (45) Chave de Habilitacao de Cargas Posicao Cima = 1
byte CircuitoBoia;    // I (35) Circuito de Alimentacao da Boia da Cx Azul: Desligado = 0 / Ligado = 1
byte BoiaCxAzul;      // I (37) Retorno da Boia da Cx Azul: Cheia = 0 / Precisa Encher = 1
byte CircuitoBomba;   // I (39) Circuito de Alimentacao da Bomba: Desligado = 0 / Ligado = 1
byte AlimRedeBomba;   // I (40) Alimentacao CA Rede para a Bomba: Desligado = 0 / Ligado = 1
byte EdCxAzCheia;     // I (6)  Contato que sinaliza que a Caixa Azul esta cheia
byte FonteCC1Lig;     // I (23) Fonte de Alimentacao 24Vcc do Inversor 2 e Carregador de Baterias
byte FonteCC2Lig;     // I (7)  Fonte de Alimentacao 24Vcc dos Circuitos CC Ligada

// Saidas Digitais Lidas da UTR Arduino Mega
byte SD00;            // SD00 - 0 => CT1 = Rede / 1 => CT1 = Inv (Energia Portao e Iluminacao Externa)
byte SD01;            // SD01 - 0 => Desliga Inv1 e CT4 Rede / 1 => Liga Inv 1 e CT4 Inv 1 (Bomba)
byte SD02;            // SD02 - 0 => CT3 = Rede / 1 => CT3 = Inversor (Energia Geladeira)
byte SD03;            // SD03 - 0 => Inversor Conectado na CT2 / 1 => Inversor Desconectado da CT2
byte SD04;            // SD04 - Controla o LED 2 (Amarelo): sinaliza falha de inversor
byte SD05;            // SD05 - Controla o LED 3 (Verde): sinaliza tensao 220VCA para as Cargas
byte SD06;            // SD06 - 0 => Inversor Conectado na CT1 / 1 => Inversor Desconectado da CT1
byte SD07;            // SD07 - 0 => Inversor Conectado na CT3 / 1 => Inversor Desconectado da CT3
byte SD08;            // SD08 - Controla o LED 1 (Verde)
byte SD09;            // SD09 - Controla o LED 4 (Verde)
byte SD10;            // SD10 - 0 => Desliga Inversor 2 / 1 => Liga Inversor 2
byte SD11;            // SD11 - 0 => Inversor Conectado na CT4 / 1 => Inversor Desconectado da CT4
byte SD12;            // SD12 - Controla o LED Azul 1
byte SD13;            // SD13 - Controla o LED Azul 2
byte SD14;            // SD14 - Controla o LED Azul 3
byte SD15;            // SD15 - Controla o LED Azul 4
byte SD16;            // SD16 - 0 => Fonte Ligada / 1 => Fonte Desligada
byte SD17;            // SD17 - 0 => CT2 = Rede / 1 => CT2 = Inversor (Casa Verde e Oficina)
byte SD18;            // Reserva
byte SD19;            // Reserva

// Variaveis de Estado
byte EstadoInversor1;    // Inversor 1 - 1 = Ligado / 0 = Desligado
byte EstadoInversor2;    // Inversor 2 - 1 = Ligado / 0 = Desligado
byte EstadoRede;         // Estado da Tensao da Rede
byte EstadoCarga3;       // Estado da Carga 3: 0 = Desligada / 1 = Ligada
byte ModoComando;        // 0 = Controle Local / 1 = Controle Remoto
byte ModoControle;       // 0 = Controle Manual / 1 = Controle Automatico das Cargas 2, 3 e 4
byte ModoControle1;      // 0 = Controle Manual / 1 = Controle Automatico da Carga 1
byte ModoOperacao;       // 0 = Economia de Bateria / 1 = Normal
byte Carga1;             // 0 = Desabilita Carga 1 / 1 = Habilita Carga 1
byte Carga2;             // 0 = Desabilita Carga 2 / 1 = Habilita Carga 2
byte Carga3;             // 0 = Desabilita Carga 3 / 1 = Habilita Carga 3
byte Carga4;             // 0 = Desabilita Carga 4 / 1 = Habilita Carga 4
byte BombaLigada;        // 0 = Bomba Desligada / 1 = Bomba Ligada
byte CDBat;              // 0 = Banco de Baterias em Descarga / 1 = Banco de Baterias em Carga
byte EstadoCxAz;         // Estado da Caixa Azul

// Variaveis de Alarme
byte FalhaInversor1;
byte SubTensaoInv1;
byte SobreTensaoInv1;
byte SobreTempDrInv1;
byte SobreTempTrInv1;
byte SobreCorrenteInv1;
byte DisjAbertoIv1;
byte FalhaInversor2;
byte SubTensaoInv2;
byte SobreTensaoInv2;
byte SobreTempDrInv2;
byte SobreTempTrInv2;
byte SobreCorrenteInv2;
byte DisjAbertoIv2;
byte CxAzNivBaixo;

// Indicadores Auxiliares
boolean EvtFaltaCA;
boolean HabCom;

//boolean HabTmpBombaLig;    // Flag de Habilitacao do Temporizador de Bomba Ligada
unsigned int TmpCxPrEnch;    // Contador de Segundos do tempo que a caixa azul precisa encher

// Relogio recebido do Servidor Raspberry PI 3
byte HoraRsp;
byte MinutoRsp;
byte SegundoRsp;
byte DiaRsp;
byte MesRsp;
byte AnoRsp;

// Relogio recebido da UTR Arduino Mega
byte HoraRec;
byte MinutoRec;
byte SegundoRec;
byte DiaRec;
byte MesRec;
byte AnoRec;

// Medidas lidas dos IEDs atraves da Serial2
double Med1[20];
double Med2[20];

// Buffers de Mensagens da Interface Serial2
byte MsgTx[8];
byte MsgRx[32];
byte VerCRCRec[2];
byte TmoIED[2];
byte ContRet[2];
unsigned int ContC[2];

// Buffers de Mensagens da Interface Serial1 de Comunicacao com a UTR Arduino Mega
byte MsgTxS1[16];   // Buffer para Envio da Mensagem de Requisicao para a UTR Arduino Mega
byte MsgRxS1[64];   // Buffer para Recebimento da Mensagem de Dados da UTR Arduino Mega
byte EDbyte[8];     // Buffer para as Entradas Digitais em Modo Compacto
byte Comando = 0;
byte EstComIED1 = 0;

// Flag que indica Acerto de Relogio pelo Host
byte AcertaRelogio;

// Variaveis de gerenciamento dos Registros de Eventos na EEPROM
int PontReg; // Ponteiro para a proxima posicao a ser gravado registro na EEPROM
int NumRegEpr;  // Numero de Registros de Eventos disponiveis na EEPROM

// Definir Endereco MAC e Endereco IP do Controlador
byte mac[] = { 0x21, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,0,150);

// Variaveis da comunicacao ethernet
//const unsigned int TamBufTx = 512;
unsigned int MsgCoAPId;
unsigned int Indice = 0;
unsigned int localPort = 5683;
unsigned int Contador = 0;
byte BufSVCheio = 0;
byte HabAquisicao = 0;
byte MsgTr = 0;

byte TxBufEst[512];

// buffers para enviar e receber dados pela interafce ethernet
char RxBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

//init the real time clock
//Rtc_Pcf8563 rtc;

 
//***********************************************************************************************************************
//                                                                                                                      *
//                                            Procedimentos de Inicializacao                                            *
//                                                                                                                      *
//***********************************************************************************************************************
//
void setup() {

  // Programa o modo de operacao dos pinos de controle e inicia os estados iniciais
  pinMode(DE1, OUTPUT);    // Saida de Controle para o Driver de Transmissao RS485 do CC1 (Ativo em 1) 
  pinMode(RE1, OUTPUT);    // Saida de Habilitacao da Recepcao RS485 do CC1 (Ativo em 0)
  pinMode(DE2, OUTPUT);    // Saida de Controle para o Driver de Transmissao RS485 do CC2 (Ativo em 1)
  pinMode(RE2, OUTPUT);    // Saida de Habilitacao da Recepcao RS485 do CC2 (Ativo em 0)

  digitalWrite(DE1, LOW);  // Sinal Ativo em 1 - estado inicial = 0
  digitalWrite(RE1, HIGH); // Sinal Ativo em 0 - estado inicial = 1
  digitalWrite(DE2, LOW);  // Sinal Ativo em 1 - estado inicial = 0
  digitalWrite(RE2, HIGH); // Sinal Ativo em 0 - estado inicial = 1

  // Inicia os contadores de falhas
  ContC[0] = 0;
  ContC[1] = 0;
      
  // Inicia as variaveis das EAs com zero
  for (int i = 0; i < 16; i++) {
    EA[i] = 0;
    EAT[i] = 0;
  }
  
  // Zera os Alarmes
  FalhaInversor1 = 0;
  SubTensaoInv1 = 0;
  SobreTensaoInv1 = 0;
  SobreTempDrInv1 = 0;
  SobreTempTrInv1 = 0;
  SobreCorrenteInv1 = 0;
  DisjAbertoIv1 = 0;
  
  FalhaInversor2 = 0;
  SubTensaoInv2 = 0;
  SobreTensaoInv2 = 0;
  SobreTempDrInv2 = 0;
  SobreTempTrInv2 = 0;
  SobreCorrenteInv2 = 0;
  DisjAbertoIv2 = 0;
  CxAzNivBaixo = 0;
  EvtFaltaCA = false;
  HabCom = true;
    
  ModoComando = 1;         // Inicia o Modo de Comando Remoto
  ModoControle = 0;        // Inicia o Modo de Controle Manual
  ModoControle1 = 0;       // Inicia o Modo de Controle Manual da Carga 1
  ModoOperacao = 0;        // Inicia o Modo de Operacao em Economia de Bateria
  Carga1 = 0;              // Inicia a Carga 1 desabilitada
  Carga2 = 0;              // Inicia a Carga 2 desabilitada
  Carga3 = 0;              // Inicia a Carga 3 desabilitada
  Carga4 = 0;              // Inicia a Carga 4 desabilitada
  BombaLigada = 0;         // Inicia o indicador de Bomba Ligada
  //HabTmpBombaLig = false;
  TmpBombaLig = 0;
  TmpCxPrEnch = 0;
  
  AcertaRelogio = 0;

  // Inicia as Interfaces Seriais Assincronas
  Serial.begin(9600);    // Carga de Programa
  Serial1.begin(9600);   // Comunicacao com a UTR Arduino Mega
  Serial2.begin(115200); // Comunicacao com os Controladores de carga 
    
  // Inicia a Ethernet e o UDP:
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

} // Fim dos Procedimentos de Inicializacao


//*********************************************************************************************************************
//                                                                                                                    *
//                                   Inicio do Programa Principal - Concentrador                                      *
//                                                                                                                    *
//*********************************************************************************************************************
//
void loop() {

  // Efetua os procedimentos de comunicacao com o Nivel Superior pela interface Ethernet  
  ComunicacaoEthernet();
   
} // Final do Loop Principal do Programa


//*********************************************************************************************************************
// Nome da Rotina: ComunicacaoEthernet                                                                                *
//                                                                                                                    *
// Funcao: verifica se tem requisicao pela interface ethernet (UDP) e responde a solicitacao                          *
//                                                                                                                    *
// Entrada: nenhum                                                                                                    *
// Saida: nenhum                                                                                                      *
//                                                                                                                    *
//*********************************************************************************************************************
//
void ComunicacaoEthernet() {

  int packetSize = Udp.parsePacket();
  if (packetSize) {                             // Verifica se chegou mensagem UDP pela ethernet
    IPAddress remote = Udp.remoteIP();          // Se chegou mensagem, obter o endereco IP do cliente
    Udp.read(RxBuffer, UDP_TX_PACKET_MAX_SIZE); // e ler o pacote de dados recebido do cliente

    // Se recebeu mensagem simples
    if (RxBuffer[0] < 0x40) {
      Serial.println("");
      Serial.println("Recebeu Mensagem Simples");
      Comando = RxBuffer[0];                      // Le o codigo do comando
      HoraRsp = RxBuffer[1];                      // Le o relogio
      MinutoRsp = RxBuffer[2];
      SegundoRsp = RxBuffer[3];
      DiaRsp = RxBuffer[4];
      MesRsp = RxBuffer[5];
      AnoRsp = RxBuffer[6];
      
      EstComIED1 = ComunicacaoSerial();           // Efetua a comunicacao com a UTR Arduino Mega
      ComunicacaoIEDs();                          // Efetua a comunicacao com os Controladores de Carga
      EfetuaCalculos();                           // Calcula um conjunto de Variaveis
      MontaMsgEstados(Comando);                   // Transmite as informacoes pela Interface Ethernet
      Comando = 0;
    }

    // Se recebeu mensagem CoAP Confirmavel do tipo GET
    if ((RxBuffer[0] == 0x40) && (RxBuffer[1] == 0x01)) {
      Serial.println("");
      Serial.println("Recebeu Mensagem CoAP");
      
      MsgCoAPId = DoisBytesParaInt(RxBuffer[3], RxBuffer[2]);
      
      Comando = RxBuffer[15];                 // Le o Comando do Payload
      HoraRsp = RxBuffer[16];                 // Le a Hora do Payload
      MinutoRsp = RxBuffer[17];               // Le o Minuto do Payload
      SegundoRsp = RxBuffer[18];              // Le o Segundo do Payload
      DiaRsp = RxBuffer[19];                  // Le o Dia do Payload
      MesRsp = RxBuffer[20];                  // Le o Mes do Payload
      AnoRsp = RxBuffer[21];                  // Le o Ano do Payload

      EstComIED1 = ComunicacaoSerial();       // Efetua a comunicacao com a UTR Arduino Mega
      ComunicacaoIEDs();                      // Efetua a comunicacao com os Controladores de Carga
      EfetuaCalculos();                       // Calcula um conjunto de Variaveis
      MontaMsgRspCoAP(Comando);               // Transmite as informacoes pela Interface Ethernet
      Comando = 0;
    }
  } // if (packetSize)
} // Fim da Rotina ComunicacaoEthernet


//*********************************************************************************************************************
// Nome da Rotina: ComunicacaoSerial                                                                                  *
//                                                                                                                    *
// Funcao: envia a requisicao pela interface Serial1 e recebe a mensagem de resposta da UTR Arduino Mega              *
//                                                                                                                    *
// Entrada: nenhum                                                                                                    *
// Saida: byte: 1 => a UTR respondeu a requisicao / 0 => a UTR nao respondeu a requisicao                             *
//                                                                                                                    *
//*********************************************************************************************************************
//
boolean ComunicacaoSerial() {

  byte NBMsgT = 14;   // Numero de Bytes da Mensagem de Requisicao para a UTR Arduino Mega
  byte NBMsgR = 60;   // Numero de Bytes da Mensagem de Resposta Recebida da UTR Arduino Mega
  byte TmoIED;
  byte Res = 0;
  byte ch;

  unsigned int CRC = 0xffff;
  byte lsb;

  // Limpa a Interface Serial 1 antes de chamar a UTR
  byte tmp;
  while (Serial1.available() > 0) {
    tmp = Serial1.read();
  }

  // Carrega a Mensagem de Requisicao no Buffer de Transmissao
  MsgTxS1[0] = 1;           // Endereco da UTR
  MsgTxS1[1] = 3;           // Funcao
  MsgTxS1[2] = Comando;     // Codigo do Comando
  MsgTxS1[3] = 0;           // Reserva
  MsgTxS1[4] = HoraRsp;     // Hora
  MsgTxS1[5] = MinutoRsp;   // Minuto
  MsgTxS1[6] = SegundoRsp;  // Segundo
  MsgTxS1[7] = DiaRsp;      // Dia
  MsgTxS1[8] = MesRsp;      // Mes
  MsgTxS1[9] = AnoRsp;      // Ano
  MsgTxS1[10] = 0;          // Reserva
  MsgTxS1[11] = 0;          // Reserva
        
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

  MsgTxS1[NBMsgT - 2] = ByteL(CRC); // CRC Low
  MsgTxS1[NBMsgT - 1] = ByteH(CRC); // CRC High
      
  // Transmite a Mensagem de Requisicao para a UTR Arduino Mega pela Serial1
  for (int i = 0; i < NBMsgT ; i++) {
    while (Serial1.availableForWrite() < 1) {
    }
    Serial1.write(MsgTxS1[i]);
  }

  Serial.println("");
  Serial.println("Msg Tx para a UTR");
  
  delay(300);  // Espera 100ms pela mensagem de resposta
  
  // Le os bytes recebidos da UTR Arduino Mega pela Interface Serial1
  int cont = 0;
  int cntb = 0;
  while ((cntb < NBMsgR) && (cont < 1000)) {
    if (Serial1.available() > 0) {
      MsgRxS1[cntb] = Serial1.read();
      cntb = cntb + 1;
    }
    cont = cont + 1;
  }
  Serial.print("Num Bytes Rec da UTR = ");
  Serial.println(cntb);

  if (cont < 1000) { // Se nao ocorreu TimeOut, 
    
    // Calcula o CRC da mensagem recebida
    for (int j = 0; j < (NBMsgR - 2); j++) {
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

    // Verifica se a mensagem de resposta da UTR Arduino Mega chegou corretamente
    byte CRC_OK = 0;
    if ((MsgRxS1[NBMsgR - 2] == ByteL(CRC)) && (MsgRxS1[NBMsgR - 1] == ByteH(CRC))) {
      CRC_OK = 1;
    }

    // Verifica se o CRC e os bytes de identificacao conferem (endereco e funcao)
    if ((CRC_OK == 1) && (MsgRxS1[0] == 1) && (MsgRxS1[1] == 3)) {  // Se a mensagem recebida esta OK,
      Serial.println("Id Msg OK");
      Res = 1;                       // Sinaliza recebimento de mensagem OK da UTR Arduino Mega
      // carrega as informacoes recebidas da UTR Arduino Mega nas variaveis do Concentrador Arduino Mega                              
      Comando = MsgRxS1[2];          // Comando Recebido da UTR Arduino Mega 
                                     // Reserva
      HoraRec = MsgRxS1[4];          // Hora do RTC da UTR
      MinutoRec = MsgRxS1[5];        // Minuto do RTC da UTR
      SegundoRec = MsgRxS1[6];       // Segundo do RTC da UTR
      DiaRec = MsgRxS1[7];           // Dia do RTC da UTR
      MesRec = MsgRxS1[8];           // Mês do RTC da UTR
      AnoRec = MsgRxS1[9];           // Ano do RTC da UTR (00 a 99)
      EstadoCxAz = MsgRxS1[10];      // Estado da Caixa Azul
      
      // Carrega as EDs compactadas recebidas da UTR Arduino Mega no buffer EDByte[]
      EDbyte[0] = MsgRxS1[12];
      EDbyte[1] = MsgRxS1[13];
      EDbyte[2] = MsgRxS1[14];
      EDbyte[3] = MsgRxS1[15];
      EDbyte[4] = MsgRxS1[16];
      EDbyte[5] = MsgRxS1[17];
      EDbyte[6] = MsgRxS1[18];
      EDbyte[7] = MsgRxS1[19];

      DesmontaEDs();             // Desmonta o Buffer de EDs Compactadas EDbyte[] para as Variaveis

      // Carrega as EAs recebidas da UTR Arduino Mega nas variaveis
      VBat = MontaEA(MsgRxS1[20], MsgRxS1[21]);         // Tensao do Barramento Principal 24Vcc
      EA[1] = MontaEA(MsgRxS1[22], MsgRxS1[23]);        // Reserva
      TDInv2 = MontaEA(MsgRxS1[24], MsgRxS1[25]);       // Temperatura do Driver do Inversor 2 
      ICircCC = MontaEA(MsgRxS1[26], MsgRxS1[27]);      // Corrente CC: Circuitos de Corrente Continua
      VSIv1 = MontaEA(MsgRxS1[28], MsgRxS1[29]);        // Tensao CA: Saida do Inversor 1
      VRede = MontaEA(MsgRxS1[30], MsgRxS1[31]);        // Tensao CA: Rede
      VSIv2 = MontaEA(MsgRxS1[32], MsgRxS1[33]);        // Tensao CA: Saida do Inversor 1
      TTInv2 = MontaEA(MsgRxS1[34], MsgRxS1[35]);       // Temperatura do Transformador do Inversor 2
      TDInv1 = MontaEA(MsgRxS1[36], MsgRxS1[37]);       // Temperatura do Driver do Inversor 1
      TTInv1 = MontaEA(MsgRxS1[38], MsgRxS1[39]);       // Temperatura do Transformador do Inversor 1
      ISInv1 = MontaEA(MsgRxS1[40], MsgRxS1[41]);       // Corrente CA: Saida do Inversor 1
      IFonteCgBat = MontaEA(MsgRxS1[42], MsgRxS1[43]);  // Corrente CC: Saida da Fonte 24Vcc
      IEIv1 = MontaEA(MsgRxS1[44], MsgRxS1[45]);        // Corrente CC: Entrada do Inversor 1
      ISInv2 = MontaEA(MsgRxS1[46], MsgRxS1[47]);       // Corrente CA: Saida do Inversor 2
      Icarga3 = MontaEA(MsgRxS1[48], MsgRxS1[49]);      // Corrente CA:Carga 3
      IEIv2 = MontaEA(MsgRxS1[50], MsgRxS1[51]);        // Corrente CC: Entrada do Inversor 2
      EAME0 = MontaEA(MsgRxS1[52], MsgRxS1[53]);        // Tensao CC: media da tensao de 24Vcc
      TmpBombaLig = MontaEA(MsgRxS1[54], MsgRxS1[55]);  // Tempo da Bomba ligada
      TmpCxAzNvBx =  MontaEA(MsgRxS1[56], MsgRxS1[57]); // Tempo que a Caixa Azul esta com Nivel Baixo
  
    }  // Verifica o CRC e/ou a identificacao da mensagem
  }  // Verifica se ocorreu TimeOut
  
  return(Res);
  
} // Fim da Rotina ComunicacaoSerial()


//*********************************************************************************************************************
// Nome da Rotina: DesmontaEDs()                                                                                      *
//                                                                                                                    *
// Funcao: desmonta os bits do formato compacto para as variaveis de estado digital                                   *
//                                                                                                                    *
// Entrada: nenhum                                                                                                    *
// Saida: nenhum                                                                                                      *
//*********************************************************************************************************************
//
void DesmontaEDs() {
  
  DJEINV1 =        (EDbyte[0] & B00000001); 
  CircuitoBoia =  ((EDbyte[0] & B00000010) >> 1);
  BoiaCxAzul =    ((EDbyte[0] & B00000100) >> 2);
  CircuitoBomba = ((EDbyte[0] & B00001000) >> 3);
  AlimRedeBomba = ((EDbyte[0] & B00010000) >> 4);
  CxAzNivBaixo =  ((EDbyte[0] & B00100000) >> 5);
  EstadoRede =    ((EDbyte[0] & B01000000) >> 6);
  ModoOperacao =  ((EDbyte[0] & B10000000) >> 7);

  ModoComando =      (EDbyte[1] & B00000001);
  ModoControle =    ((EDbyte[1] & B00000010) >> 1);
  Carga1 =          ((EDbyte[1] & B00000100) >> 2);
  Carga2 =          ((EDbyte[1] & B00001000) >> 3);
  Carga3 =          ((EDbyte[1] & B00010000) >> 4);
  Carga4 =          ((EDbyte[1] & B00100000) >> 5);
  HabCom =          ((EDbyte[1] & B01000000) >> 6);
  EstadoInversor1 = ((EDbyte[1] & B10000000) >> 7);

  ModoComando =      (EDbyte[2] & B00000001);
  EstadoInversor2 = ((EDbyte[2] & B00000010) >> 1);
  EstadoCarga3 =    ((EDbyte[2] & B00000100) >> 2);
  BombaLigada =     ((EDbyte[2] & B00001000) >> 3);
  //FlgTmpFT =        ((EDbyte[2] & B00010000) >> 4);
  //CTRF =            ((EDbyte[2] & B00100000) >> 5);
  //FlgTmpFTR =       ((EDbyte[2] & B01000000) >> 6);
  ModoControle1 =   ((EDbyte[2] & B10000000) >> 7);

  FalhaInversor1 =   (EDbyte[3] & B00000001);
  SubTensaoInv1 =   ((EDbyte[3] & B00000010) >> 1);
  SobreTensaoInv1 = ((EDbyte[3] & B00000100) >> 2);
  SobreTempDrInv1 = ((EDbyte[3] & B00001000) >> 3);
  SobreTempTrInv1 = ((EDbyte[3] & B00010000) >> 4);
  DisjAbertoIv1 =   ((EDbyte[3] & B00100000) >> 5);
  FalhaInversor2 =  ((EDbyte[3] & B01000000) >> 6);
  SubTensaoInv2 =   ((EDbyte[3] & B10000000) >> 7);

  SobreTensaoInv2 =  (EDbyte[4] & B00000001);
  SobreTempDrInv2 = ((EDbyte[4] & B00000010) >> 1);
  SobreTempTrInv2 = ((EDbyte[4] & B00000100) >> 2);
  DisjAbertoIv2 =   ((EDbyte[4] & B00001000) >> 3);
  SD00 =            ((EDbyte[4] & B00010000) >> 4);
  SD01 =            ((EDbyte[4] & B00100000) >> 5);
  SD02 =            ((EDbyte[4] & B01000000) >> 6);
  SD03 =            ((EDbyte[4] & B10000000) >> 7);

  SD04 =  (EDbyte[5] & B00000001);
  SD05 = ((EDbyte[5] & B00000010) >> 1);
  SD06 = ((EDbyte[5] & B00000100) >> 2);
  SD07 = ((EDbyte[5] & B00001000) >> 3);
  SD08 = ((EDbyte[5] & B00010000) >> 4);
  SD09 = ((EDbyte[5] & B00100000) >> 5);
  SD10 = ((EDbyte[5] & B01000000) >> 6);
  SD11 = ((EDbyte[5] & B10000000) >> 7);

  SD12 =         (EDbyte[6] & B00000001);
  SD13 =        ((EDbyte[6] & B00000010) >> 1);
  SD14 =        ((EDbyte[6] & B00000100) >> 2);
  SD15 =        ((EDbyte[6] & B00001000) >> 3);
  SD16 =        ((EDbyte[6] & B00010000) >> 4);
  SD17 =        ((EDbyte[6] & B00100000) >> 5);
  EdCxAzCheia = ((EDbyte[6] & B01000000) >> 6);
  FonteCC2Lig = ((EDbyte[6] & B10000000) >> 7);

  FonteCC1Lig = (EDbyte[7] & B00000001);
  SobreCorrenteInv1 = ((EDbyte[7] & B00000010) >> 1);
  SobreCorrenteInv2 = ((EDbyte[7] & B00000100) >> 2);

} // Fim da Rotina DesmontaEDs


//*********************************************************************************************************************
// Nome da Rotina: MontaEA()                                                                                          *
//                                                                                                                    *
// Funcao: monta uma variavel de entrada analogica EA em formato double a partir de um inteiro de dois bytes          *
//                                                                                                                    *
// Entrada: (Byte Low, byte High)                                                                                     *
// Saida: nenhum                                                                                                      *
//*********************************************************************************************************************
//
double MontaEA(byte ByteL, byte ByteH) {

  double valor = (256 * ByteH) + ByteL;
  return (valor);
  
}


//*********************************************************************************************************************
// Nome da Rotina: MontaMsgEstados                                                                                    *
//                                                                                                                    *
// Funcao: monta e transmite a mensagem de resposta com os estados da UTR                                             *
//                                                                                                                    *
// Entrada: codigo do comando que esta mensagem foi usada como resposta                                               *
// Saida: preenche o array TxBufEst e transmite pela ethernet em protocolo UDP                                        *
//                                                                                                                    *
//*********************************************************************************************************************
//
void MontaMsgEstados(byte Codigo) {

  // Cabecalho da Mensagem: bytes 000 a 015
  TxBufEst[0] = 1;                 // Endereco da UTR
  TxBufEst[1] = 3;                 // Modelo da UTR (3 = Arduino Mega 2560)
  TxBufEst[2] = AcertaRelogio;     // Modo (AcertaRelogio=0 ou AcertaRelogio=1)
  TxBufEst[3] = 81;                // Numero de Estados Digitais Configurados (max 112)
  TxBufEst[4] = 48;                // Numero de Medidas Configuradas (max 48)
  TxBufEst[5] = HoraRec;           // Hora recebida da UTR Arduino Mega
  TxBufEst[6] = MinutoRec;         // Minuto recebido da UTR Arduino Mega
  TxBufEst[7] = SegundoRec;        // Segundo recebido da UTR Arduino Mega
  TxBufEst[8] = DiaRec;            // Dia recebido da UTR Arduino Mega
  TxBufEst[9] = MesRec;            // Mês recebido da UTR Arduino Mega
  TxBufEst[10] = AnoRec;           // Ano recebido da UTR Arduino Mega (00 a 99)
  TxBufEst[11] = EstComIED1;       // Estado de Comunicacao da UTR Arduino Mega
  TxBufEst[12] = VerCRCRec[0];     // Estado de Comunicacao do Controlador de Carga 1
  TxBufEst[13] = VerCRCRec[1];     // Estado de Comunicacao do Controlador de Carga 2
  TxBufEst[15] = Codigo;           // O comando foi recebido e executado (codigo do comando)
  
  // Estados Digitais: bytes 016 a 047 - 1 byte por estado
  TxBufEst[16] = ChLR;
  TxBufEst[17] = ChEN;
  TxBufEst[18] = ChMA;
  TxBufEst[19] = ChPrB;
  TxBufEst[20] = ChPrC;
  
  TxBufEst[21] = DJEINV1;
  TxBufEst[22] = CircuitoBoia;
  TxBufEst[23] = BoiaCxAzul;
  TxBufEst[24] = CircuitoBomba;
  TxBufEst[25] = AlimRedeBomba;
  //TxBufEst[26] = Estado24Vcc;
  TxBufEst[27] = EstadoRede;
  TxBufEst[28] = ModoOperacao;
  TxBufEst[29] = ModoComando;
  TxBufEst[30] = ModoControle;
  TxBufEst[31] = Carga1;
  TxBufEst[32] = Carga2;
  TxBufEst[33] = Carga3;
  TxBufEst[34] = Carga4;
  TxBufEst[35] = HabCom;
  TxBufEst[36] = EstadoInversor1;
  TxBufEst[37] = EstadoInversor2;
  TxBufEst[38] = EstadoCarga3;
  TxBufEst[39] = BombaLigada;
  
  TxBufEst[44] = ModoControle1;
  
  TxBufEst[45] = FalhaInversor1;
  TxBufEst[46] = SubTensaoInv1;
  TxBufEst[47] = SobreTensaoInv1;
  TxBufEst[48] = SobreTempDrInv1;
  TxBufEst[49] = SobreTempTrInv1;
  TxBufEst[50] = DisjAbertoIv1;
  TxBufEst[51] = FalhaInversor2;
  TxBufEst[52] = SubTensaoInv2;
  TxBufEst[53] = SobreTensaoInv2;
  TxBufEst[54] = SobreTempDrInv2;
  TxBufEst[55] = SobreTempTrInv2;
  TxBufEst[56] = DisjAbertoIv2;
  
  TxBufEst[57] = SD00;
  TxBufEst[58] = SD01;
  TxBufEst[59] = SD02;
  TxBufEst[60] = SD03;
  TxBufEst[61] = SD04;
  TxBufEst[62] = SD05;
  TxBufEst[63] = SD06;
  TxBufEst[64] = SD07;
  TxBufEst[65] = SD08;
  TxBufEst[66] = SD09;
  TxBufEst[67] = SD10;
  TxBufEst[68] = SD11;
  TxBufEst[69] = SD12;
  TxBufEst[70] = SD13;
  TxBufEst[71] = SD14;
  TxBufEst[72] = SD15;
  TxBufEst[73] = SD16;
  TxBufEst[74] = SD17;
  TxBufEst[75] = SD18;
  TxBufEst[76] = SD19;
  TxBufEst[77] = CDBat;
  TxBufEst[78] = CxAzNivBaixo;
  TxBufEst[79] = EdCxAzCheia;
  TxBufEst[80] = FonteCC2Lig;
  TxBufEst[81] = EstadoCxAz;
  TxBufEst[82] = FonteCC1Lig;
  TxBufEst[83] = SobreCorrenteInv1;
  TxBufEst[84] = SobreCorrenteInv2;
           
  // Medidas (48): bytes 110 a 253 - 3 bytes por medida
  Carrega_TxBufEst(VBat,110);         // Med[00] - VBat = Tensao do Barramento Principal 24Vcc
  Carrega_TxBufEst(0,113);            // Med[01] - Reserva
  Carrega_TxBufEst(TDInv1,116);       // Med[02] - Temperatura do Driver do Inversor 1
  Carrega_TxBufEst(ICircCC,119);      // Med[03] - Corrente CC: Circuitos de Corrente Continua
  Carrega_TxBufEst(VSIv1,122);        // Med[04] - Tensao CA: Saida do Inversor 1
  Carrega_TxBufEst(VRede,125);        // Med[05] - Tensao CA: Rede
  Carrega_TxBufEst(VSIv2,128);        // Med[06] - Tensao CA: Saida do Inversor 1
  Carrega_TxBufEst(TTInv1,131);       // Med[07] - Temperatura do Transformador do Inversor 1
  Carrega_TxBufEst(TDInv2,134);       // Med[08] - Temperatura do Driver do Inversor 2
  Carrega_TxBufEst(TTInv2,137);       // Med{09] - Temperatura do Transformador do Inversor 2
  Carrega_TxBufEst(ISInv2,140);       // Med[10] - Corrente CA: Saida do Inversor 2
  Carrega_TxBufEst(IFonteCgBat,143);  // Med[11] - Corrente CC: Saida da Fonte 24Vcc
  Carrega_TxBufEst(IEIv2,146);        // Med[12] - Corrente CC: Entrada do Inversor 2
  Carrega_TxBufEst(ISInv1,149);       // Med[13] - Corrente CA: Saida do Inversor 1
  Carrega_TxBufEst(Icarga3,152);      // Med[14] - Corrente CA:Carga 3
  Carrega_TxBufEst(IEIv1,155);        // Med[15] - Corrente CC: Entrada do Inversor 1
  Carrega_TxBufEst(EAME0,158);        // Med[16] - Media Estendida da Entrada Analogica 0 (Tensao 24Vcc Geral)
  Carrega_TxBufEst(TmpBombaLig,161);  // Med[17] - Temporizador de Bomba Ligada em segundos
  Carrega_TxBufEst(VP12,164);         // Med[18] - Controlador de Carga 1 - MODBUS=0x3100 - PV array voltage 1
  Carrega_TxBufEst(IS12,167);         // Med[19] - Controlador de Carga 1 - MODBUS=0x3101 - PV array current 1
  Carrega_TxBufEst(WS12,170);         // Med[20] - Controlador de Carga 1 - MODBUS=0x3102 - PV array power 1
  Carrega_TxBufEst(VBat1,173);        // Med[21] - Controlador de Carga 1 - MODBUS=0x3104 - Battery voltage 1
  Carrega_TxBufEst(ISCC1,176);        // Med[22] - Controlador de Carga 1 - MODBUS=0x3105 - Battery charging current 1
  Carrega_TxBufEst(WSCC1,179);        // Med[23] - Controlador de Carga 1 - MODBUS=0x3106 - Battery charging power 1
  Carrega_TxBufEst(TBat1,182);        // Med[24] - Controlador de Carga 1 - MODBUS=0x3110 - Battery Temperature 1
  Carrega_TxBufEst(0, 185);           // Med[25] - Reserva
  Carrega_TxBufEst(VP34,188);         // Med[26] - Controlador de Carga 2 - MODBUS=0x3100 - PV array voltage 2
  Carrega_TxBufEst(IS34,191);         // Med[27] - Controlador de Carga 2 - MODBUS=0x3101 - PV array current 2
  Carrega_TxBufEst(WS34,194);         // Med[28] - Controlador de Carga 2 - MODBUS=0x3102 - PV array power 2
  Carrega_TxBufEst(VBat2,197);        // Med[29] - Controlador de Carga 2 - MODBUS=0x3104 - Battery voltage 2
  Carrega_TxBufEst(ISCC2,200);        // Med[30] - Controlador de Carga 2 - MODBUS=0x3105 - Battery charging current 2
  Carrega_TxBufEst(WSCC2, 203);       // Med[31] - Controlador de Carga 2 - MODBUS=0x3106 - Battery charging power 2
  Carrega_TxBufEst(TBat2,206);        // Med[32] - Controlador de Carga 2 - End. MODBUS=0x3110 - Battery Temperature 2
  Carrega_TxBufEst(ITotGer,209);      // Med[33] - Corrente Total Gerada
  Carrega_TxBufEst(WTotGer,212);      // Med[34] - Potencia Total Gerada
  Carrega_TxBufEst(WCircCC,215);      // Med[35] - Potencia Consumida pelos Circuitos de 24Vcc
  Carrega_TxBufEst(WFonteCC,218);     // Med[36] - Potencia Fornecida pela Fonte 24Vcc
  Carrega_TxBufEst(IBat,221);         // Med[37] - Corrente de Carga ou Descarga do Banco de Baterias
  Carrega_TxBufEst(WEIv2,224);        // Med[38] - Potencia de Entrada do Inversor 2
  Carrega_TxBufEst(WSInv2,227);       // Med[39] - Potencia de Saida do Inversor 2
  //Carrega_TxBufEst(variavel,230);   // Med[40] - Reserva
  Carrega_TxBufEst(WEIv1,233);        // Med[41] - Potencia de Entrada do Inversor 1
  Carrega_TxBufEst(WSInv1,236);       // Med[42] - Potencia de Saida do Inversor 1
  //Carrega_TxBufEst(Variavel,239);   // Med[43] - Reserva
  Carrega_TxBufEst(ITotCg,242);       // Med[44] - Corrente Total Consumida pelas Cargas
  Carrega_TxBufEst(WTotCg,245);       // Med[45] - Potencia Total Consumida pelas Cargas
  Carrega_TxBufEst(TmpCxAzNvBx,248);  // Med[46] - Tempo da Caixa Azul em Nivel Baixo
  Carrega_TxBufEst(IFontesCC12,251);  // Med[47] - IFontesCC12
  
// Verificacao de erro: bytes 254 e 255
  unsigned int soma = 0;
  for (int i = 0; i < 254; i++) {
    soma += TxBufEst[i];
  }
  TxBufEst[254] = ByteL(soma);
  TxBufEst[255] = ByteH(soma);
  
  // Transmite a mensagem para o cliente
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(TxBufEst, 256);
  Udp.endPacket();
  
} // Fim da Rotina MontaMsgEstados()


//*********************************************************************************************************************
// Nome da Rotina: MontaMsgRspCoAP                                                                                    *
//                                                                                                                    *
// Funcao: monta e transmite a mensagem de resposta com os estados da UTR em Protocolo CoAP em UDP                    *
//                                                                                                                    *
// Entrada: codigo do comando que esta mensagem foi usada como resposta                                               *
// Saida: nao tem                                                                                                     *
// Byte |           0         |        1        |   2  |   3  |    4     |      5     |                               *
// Bit  | 7 6 | 5 4 | 3 2 1 0 | 7 6 5 4 3 2 1 0 |      |      |          |            |                               *
//      | Ver |Tipo |  Token  |  Código (c.m)   | Message ID  |  Option  | Payload ID |                               *
//                                                                                                                    *
// Ver (Versão) = 01 (O número da versão do protocolo CoAP é fixo)  / TKL (Token) = 0000 (não é usado)                *                            *
// Tipo: 00 Confirmável (CON) / 01 Não-Confirmável (NON) / 10 Reconhecimento (ACK) / 11 Reset (RST)                   *
//                                                                                                                    *            *
// Códigos de Solicitação: 00 EMPTY / 01 GET   / 02 POST / 03 PUT / 04 DELETE                                         * 
//                                                                                                                    *            *
// Cód. Resposta: 41 Created / 42 Deleted / 43 Valid / 44 Changed / 45 Content                                        *
//                                                                                                                    *
// Cód. Erro Cliente: 80 Bad Request / 81 Unauthorized / 82 Bad Option / 83 Forbidden                                 *
// 84 Not Found / 85 Method Not Allowed / 86 Not Acceptable / 8C Request Entity Incomplete                            *
//                                                                                                                    *
// Cód. Erro Servidor: A0 Internal Server Error / A1 Not Implemented / A2 Bad Gateway                                 *
//                     A3 Service Unavailable / A4 Gateway Timeout / A5 Proxying Not Supported                        *
//                                                                                                                    *
// Message ID (Identificação da mensagem): inteiro de 16 bits sem sinal                                               *
//                                                                                                                    *
// Option (Opções) = 0000 0000 (não é usado) / Identificador de Início do Payload: 1111 1111                          *
//                                                                                                                    *
//*********************************************************************************************************************
//
void MontaMsgRspCoAP(byte Codigo) {

  // Cabecalho CoAP: bytes 000 a 015
  TxBufEst[0] = 0x60;              // Versao = 1 (01) e Tipo = 2 (10) ACK 
  TxBufEst[1] = 0x45;              // Codigo de Resposta 2.05 (Resposta OK do tipo Content)
  TxBufEst[2] = ByteH(MsgCoAPId);  // Responde com o Identificador da Mensagem Recebido
  TxBufEst[3] = ByteL(MsgCoAPId);
  TxBufEst[4] = 0x0c1;             // Primeiro Campo de Opcoes => Delta = 12: Content-format 
  TxBufEst[5] = 0x2a;              // Opcao do Tipo do Payload: application/octet-stream
  TxBufEst[6] = 0x0ff;             // Indicador do fim do Campo de Opcoes
  TxBufEst[7] = 0;
  TxBufEst[8] = 0;
  TxBufEst[9] = 0;
  TxBufEst[10] = 0;
  TxBufEst[11] = 0;
  TxBufEst[12] = 0;
  TxBufEst[13] = 0;
  TxBufEst[14] = 0;
  TxBufEst[15] = 0;
  
  // Cabecalho do Payload: bytes 016 a 031
  TxBufEst[16] = Codigo;            // O comando foi recebido e executado (codigo do comando)
  TxBufEst[17] = 3;                 // Modelo da UTR (3 = Arduino Mega 2560)
  TxBufEst[18] = AcertaRelogio;     // Modo (AcertaRelogio=0 ou AcertaRelogio=1)
  TxBufEst[19] = 81;                // Numero de Estados Digitais Configurados (max 112)
  TxBufEst[20] = 48;                // Numero de Medidas Configuradas (max 48)
  TxBufEst[21] = HoraRec;           // Hora recebida da UTR Arduino Mega
  TxBufEst[22] = MinutoRec;         // Minuto recebido da UTR Arduino Mega
  TxBufEst[23] = SegundoRec;        // Segundo recebido da UTR Arduino Mega
  TxBufEst[24] = DiaRec;            // Dia recebido da UTR Arduino Mega
  TxBufEst[25] = MesRec;            // Mês recebido da UTR Arduino Mega
  TxBufEst[26] = AnoRec;            // Ano recebido da UTR Arduino Mega (00 a 99)
  TxBufEst[27] = EstComIED1;        // Estado de Comunicacao da UTR Arduino Mega
  TxBufEst[28] = VerCRCRec[0];      // Estado de Comunicacao do Controlador de Carga 1
  TxBufEst[29] = VerCRCRec[1];      // Estado de Comunicacao do Controlador de Carga 2
  TxBufEst[30] = 0;
  TxBufEst[31] = 0;
  
  // Estados Digitais de Entradas ou Variaveis (80): bytes 032 a 111 - 1 byte por estado
  TxBufEst[32] = ChLR;
  TxBufEst[33] = ChEN;
  TxBufEst[34] = ChMA;
  TxBufEst[35] = ChPrB;
  TxBufEst[36] = ChPrC;
  
  TxBufEst[37] = DJEINV1;
  TxBufEst[38] = CircuitoBoia;
  TxBufEst[39] = BoiaCxAzul;
  TxBufEst[40] = CircuitoBomba;
  TxBufEst[41] = AlimRedeBomba;
  TxBufEst[42] = EstadoRede;
  TxBufEst[43] = ModoOperacao;
  TxBufEst[44] = ModoComando;
  TxBufEst[45] = ModoControle;
  TxBufEst[46] = Carga1;
  TxBufEst[47] = Carga2;
  TxBufEst[48] = Carga3;
  TxBufEst[49] = Carga4;
  TxBufEst[50] = HabCom;
  TxBufEst[51] = EstadoInversor1;
  TxBufEst[52] = EstadoInversor2;
  TxBufEst[53] = EstadoCarga3;
  TxBufEst[54] = BombaLigada;
  TxBufEst[55] = ModoControle1;
  TxBufEst[56] = FalhaInversor1;
  TxBufEst[57] = SubTensaoInv1;
  TxBufEst[58] = SobreTensaoInv1;
  TxBufEst[59] = SobreTempDrInv1;
  TxBufEst[60] = SobreTempTrInv1;
  TxBufEst[61] = DisjAbertoIv1;
  TxBufEst[62] = FalhaInversor2;
  TxBufEst[63] = SubTensaoInv2;
  TxBufEst[64] = SobreTensaoInv2;
  TxBufEst[65] = SobreTempDrInv2;
  TxBufEst[66] = SobreTempTrInv2;
  TxBufEst[67] = DisjAbertoIv2;
  TxBufEst[68] = CDBat;
  TxBufEst[69] = CxAzNivBaixo;
  TxBufEst[70] = EdCxAzCheia;
  TxBufEst[71] = FonteCC2Lig;
  TxBufEst[72] = EstadoCxAz;
  TxBufEst[73] = FonteCC1Lig;
  TxBufEst[74] = SobreCorrenteInv1;
  TxBufEst[75] = SobreCorrenteInv2;

  // Estado das Saidas Digitais (48) 112 a 159
  TxBufEst[112] = SD00;
  TxBufEst[113] = SD01;
  TxBufEst[114] = SD02;
  TxBufEst[115] = SD03;
  TxBufEst[116] = SD04;
  TxBufEst[117] = SD05;
  TxBufEst[118] = SD06;
  TxBufEst[119] = SD07;
  TxBufEst[120] = SD08;
  TxBufEst[121] = SD09;
  TxBufEst[122] = SD10;
  TxBufEst[123] = SD11;
  TxBufEst[124] = SD12;
  TxBufEst[125] = SD13;
  TxBufEst[126] = SD14;
  TxBufEst[127] = SD15;
  TxBufEst[128] = SD16;
  TxBufEst[129] = SD17;
  TxBufEst[130] = SD18;
  TxBufEst[131] = SD19;
           
  // Medidas (64): bytes 160 a 288 - 2 bytes por medida
  CMedTxBuf(VBat,160);         // Med[00] - VBat = Tensao do Barramento Principal 24Vcc
  CMedTxBuf(0,162);            // Med[01] - Reserva
  CMedTxBuf(TDInv1,164);       // Med[02] - Temperatura do Driver do Inversor 1
  CMedTxBuf(ICircCC,166);      // Med[03] - Corrente CC: Circuitos de Corrente Continua
  CMedTxBuf(VSIv1,168);        // Med[04] - Tensao CA: Saida do Inversor 1
  CMedTxBuf(VRede,170);        // Med[05] - Tensao CA: Rede
  CMedTxBuf(VSIv2,172);        // Med[06] - Tensao CA: Saida do Inversor 1
  CMedTxBuf(TTInv1,174);       // Med[07] - Temperatura do Transformador do Inversor 1
  CMedTxBuf(TDInv2,176);       // Med[08] - Temperatura do Driver do Inversor 2
  CMedTxBuf(TTInv2,178);       // Med{09] - Temperatura do Transformador do Inversor 2
  CMedTxBuf(ISInv2,180);       // Med[10] - Corrente CA: Saida do Inversor 2
  CMedTxBuf(IFonteCgBat,182);  // Med[11] - Corrente CC: Saida da Fonte 24Vcc
  CMedTxBuf(IEIv2,184);        // Med[12] - Corrente CC: Entrada do Inversor 2
  CMedTxBuf(ISInv1,186);       // Med[13] - Corrente CA: Saida do Inversor 1
  CMedTxBuf(Icarga3,188);      // Med[14] - Corrente CA:Carga 3
  CMedTxBuf(IEIv1,190);        // Med[15] - Corrente CC: Entrada do Inversor 1
  CMedTxBuf(EAME0,192);        // Med[16] - Media Estendida da Entrada Analogica 0 (Tensao 24Vcc Geral)
  CMedTxBuf(TmpBombaLig,194);  // Med[17] - Temporizador de Bomba Ligada em segundos
  CMedTxBuf(VP12,196);         // Med[18] - Controlador de Carga 1 - MODBUS=0x3100 - PV array voltage 1
  CMedTxBuf(IS12,198);         // Med[19] - Controlador de Carga 1 - MODBUS=0x3101 - PV array current 1
  CMedTxBuf(WS12,200);         // Med[20] - Controlador de Carga 1 - MODBUS=0x3102 - PV array power 1
  CMedTxBuf(VBat1,202);        // Med[21] - Controlador de Carga 1 - MODBUS=0x3104 - Battery voltage 1
  CMedTxBuf(ISCC1,204);        // Med[22] - Controlador de Carga 1 - MODBUS=0x3105 - Battery charging current 1
  CMedTxBuf(WSCC1,206);        // Med[23] - Controlador de Carga 1 - MODBUS=0x3106 - Battery charging power 1
  CMedTxBuf(TBat1,208);        // Med[24] - Controlador de Carga 1 - MODBUS=0x3110 - Battery Temperature 1
  CMedTxBuf(0, 210);           // Med[25] - Reserva
  CMedTxBuf(VP34,212);         // Med[26] - Controlador de Carga 2 - MODBUS=0x3100 - PV array voltage 2
  CMedTxBuf(IS34,214);         // Med[27] - Controlador de Carga 2 - MODBUS=0x3101 - PV array current 2
  CMedTxBuf(WS34,216);         // Med[28] - Controlador de Carga 2 - MODBUS=0x3102 - PV array power 2
  CMedTxBuf(VBat2,218);        // Med[29] - Controlador de Carga 2 - MODBUS=0x3104 - Battery voltage 2
  CMedTxBuf(ISCC2,220);        // Med[30] - Controlador de Carga 2 - MODBUS=0x3105 - Battery charging current 2
  CMedTxBuf(WSCC2, 222);       // Med[31] - Controlador de Carga 2 - MODBUS=0x3106 - Battery charging power 2
  CMedTxBuf(TBat2,224);        // Med[32] - Controlador de Carga 2 - End. MODBUS=0x3110 - Battery Temperature 2
  CMedTxBuf(ITotGer,226);      // Med[33] - Corrente Total Gerada
  CMedTxBuf(WTotGer,228);      // Med[34] - Potencia Total Gerada
  CMedTxBuf(WCircCC,230);      // Med[35] - Potencia Consumida pelos Circuitos de 24Vcc
  CMedTxBuf(WFonteCC,232);     // Med[36] - Potencia Fornecida pela Fonte 24Vcc
  CMedTxBuf(IBat,234);         // Med[37] - Corrente de Carga ou Descarga do Banco de Baterias
  CMedTxBuf(WEIv2,236);        // Med[38] - Potencia de Entrada do Inversor 2
  CMedTxBuf(WSInv2,238);       // Med[39] - Potencia de Saida do Inversor 2
  CMedTxBuf(WEIv1,240);        // Med[41] - Potencia de Entrada do Inversor 1
  CMedTxBuf(WSInv1,242);       // Med[42] - Potencia de Saida do Inversor 1
  CMedTxBuf(ITotCg,244);       // Med[44] - Corrente Total Consumida pelas Cargas
  CMedTxBuf(WTotCg,246);       // Med[45] - Potencia Total Consumida pelas Cargas
  CMedTxBuf(TmpCxAzNvBx,248);  // Med[46] - Tempo da Caixa Azul em Nivel Baixo
  CMedTxBuf(IFontesCC12,250);  // Med[47] - IFontesCC12
  
  // Calcula o CRC16 e carrega nos ultimos dois bytes da mensagem
  unsigned int CRC = 0xffff;
  byte lsb;
  for (int j = 0; j < (TamMsgTxCoAP - 2); j++) {
    CRC = CRC ^ MsgTx[j];
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
  TxBufEst[TamMsgTxCoAP - 2] = ByteL(CRC);
  TxBufEst[TamMsgTxCoAP - 1] = ByteH(CRC);
  
  // Transmite a mensagem para o cliente
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(TxBufEst, TamMsgTxCoAP);
  Udp.endPacket();
  
} // Fim da Rotina


//********************************************************************************************************
// Nome da Rotina: Carrega_TxBufEst                                                                      *
//                                                                                                       *
// Funcao: carrega um valor double em 3 bytes no buffer BufTxEst no formato (Low, Mid, High)             *
//                                                                                                       *
// Entrada: valor double a ser carregado, endereco inicial de carga no buffer TxBufEst                   *
// Saida: nao tem                                                                                        *
//********************************************************************************************************
//
void Carrega_TxBufEst(double valor, int EndIni) {

  TxBufEst[EndIni] = ByteLow(valor);
  TxBufEst[EndIni + 1] = ByteMid(valor);
  TxBufEst[EndIni + 2] = ByteHigh(valor);

}


//********************************************************************************************************
// Nome da Rotina: CMedTxBuf                                                                             *
//                                                                                                       *
// Funcao: carrega um valor double em 2 bytes no buffer BufTxEst no formato (Low, High)                  *
//                                                                                                       *
// Entrada: valor double a ser carregado, endereco inicial de carga no buffer TxBufEst                   *
// Saida: nao tem                                                                                        *
//                                                                                                       *
//********************************************************************************************************
//
void CMedTxBuf(double valor, int EndIni) {

  TxBufEst[EndIni] = ByteL(valor);
  TxBufEst[EndIni + 1] = ByteH(valor);

}


//********************************************************************************************************************
// Nome da Rotina: EfetuaCalculos()                                                                                  *
//                                                                                                                   *
// Funcao: calcula as variaveis                                                                                      *
//                                                                                                                   *
// Entrada: nao tem                                                                                                  *
// Saida: nao tem                                                                                                    *
//********************************************************************************************************************
//
void EfetuaCalculos() {

  ITotGer = ISCC1 + ISCC2;               // Corrente Total Gerada
  WTotGer = WSCC1 + WSCC2;               // Potência Total Gerada
  ITotCg = IEIv2 + IEIv1 + (ICircCC/10); // Corrente Total Consumida pelas Cargas
  WCircCC = (VBat * ICircCC)/1000;       // Potencia Consumida pelo Circuito CC
  WTotCg = VBat * ITotCg;                // Potencia Total Consumida pelas Cargas
  WFonteCC = (VBat * IFonteCgBat) / 100; // Potencia de Carga das Baterias Fornecida pela Fonte CC1
  IBat = 0;
  
  // Inversor 2
  WEIv2 = (VBat * IEIv2)/100;            // Potência de Entrada do Inversor 2
  WSInv2 = (VSIv2 * ISInv2)/1000;        // Potência de Saída do Inversor 2
  
  WEIv1 = (VBat * IEIv1)/100;            // Potência de Entrada do Inversor 1
  WSInv1 = (VSIv1 * ISInv1)/1000;        // Potência de Saída do Inversor 1
  
  // Cargas
  ITotCg = (ICircCC/10) + IEIv2 + IEIv1;  // Calcula a Corrente Total Consumida pelas Cargas
  WTotCg = WCircCC + WEIv2 + WEIv1;       // Calcula a Potência Total Consumida pelas Cargas
  
}


//********************************************************************************************************
// Nome da Rotina: ComunicacaoIEDs()                                                                     *
//                                                                                                       *
// Funcao: efetua a comunicacao com os Controladores de Carga 1 e 2 por Interface Serial                 *
//                                                                                                       *
// Entrada: numero da interface serial (0 = A ou 1 = B)                                                  *
// Saida: nao tem                                                                                        *
//********************************************************************************************************
//
void ComunicacaoIEDs() {

  byte Ret = 3;
  VerCRCRec[0] = 0;
  while ((VerCRCRec[0] == 0) && (Ret > 0)) {
    TransmiteRecebeMsg(0);
    Serial.print("CRC[0] = ");
    Serial.println(VerCRCRec[0]);
    Ret = Ret - 1;
  }
  
  Ret = 3;
  VerCRCRec[1] = 0;
  while ((VerCRCRec[1] == 0) && (Ret > 0)) {
    TransmiteRecebeMsg(1);
    Serial.print("CRC[1] = ");
    Serial.print(VerCRCRec[1]);
    Serial.print(" - TmoIED[1] = ");
    Serial.println(TmoIED[1]);
    Ret = Ret - 1;
  }
  
} // Fim da Rotina ComunicacaoIEDs()


//********************************************************************************************************
// Nome da Rotina: TransmiteRecebeMsg()                                                                  *
//                                                                                                       *
// Funcao: monta e transmite a mensagem de chamada em protocolo MODBUS RTU para a Serial2                *
//         para leitura dos Controladores de Carga 1 e 2                                                 *
//                                                                                                       *
// Entrada: numero da interface serial (0 = A ou 1 = B)                                                  *
// Saida: nao tem                                                                                        *
//                                                                                                       *
//********************************************************************************************************
//
void TransmiteRecebeMsg(byte serial) {

  byte Endereco = 1;
  byte Funcao = 4;
  unsigned int RegIni = 0x3100;
  unsigned int NumReg = 17;
  byte NBMsgR = 5 + (NumReg * 2);
  unsigned int cont = 0;
  unsigned int CRC = 0xffff;
  byte lsb;
    
  // Carrega na mensagem a ser enviada as informacoes                               
  MsgTx[0] = Endereco;
  MsgTx[1] = Funcao;
  MsgTx[2] = ByteH(RegIni);
  MsgTx[3] = ByteL(RegIni);
  MsgTx[4] = ByteH(NumReg);
  MsgTx[5] = ByteL(NumReg);

  // Calcula o CRC16 e carrega nos ultimos dois bytes da mensagem
  for (int j = 0; j < 6; j++) {
    CRC = CRC ^ MsgTx[j];
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
     
  MsgTx[6] = ByteL(CRC);
  MsgTx[7] = ByteH(CRC);
  
  if (serial == 0) {           // Se a comunicacao com o CC1 esta habilitada,
    digitalWrite(RE1, HIGH);   // desabilita a recepcao do RS485 do CC1
    digitalWrite(DE1, HIGH);   // habilita a transmissao para o RS485 do CC1
  }

  if (serial == 1) {           // Se a comunicacao com o CC2 esta habilitada,
    digitalWrite(RE2, HIGH);   // desabilita a recepcao do RS485 do CC2  
    digitalWrite(DE2, HIGH);   // e habilita a transmissao para o RS485 do CC2
  }
 
  // Transmite a Mensagem de Chamada pela Serial2
  delay(10);
  for (int i = 0; i < 8; i++) {
    Serial2.write(MsgTx[i]);    // Envia o byte para o buffer de transmissao
    delayMicroseconds(EsperaByte);
  }

  delay(10);
  
  if (serial == 0) {           // Se a comunicacao com o CC1 esta habilitada,
    digitalWrite(DE1, LOW);    // Desabilita a transmissao para o RS485 do CC1,
    digitalWrite(RE1, LOW);    // e habilita a recepcao do RS485 do CC1
  }

  if (serial == 1) {           // Se a comunicacao com o CC2 esta habilitada,
    digitalWrite(DE2, LOW);    // Desabilita a transmissao para o RS485 do CC2,
    digitalWrite(RE2, LOW);    // e habilita a recepcao do RS485 do CC1
  }
  delay(20);

  // Inicia as variaveis de indicacao de mensagem OK e timeout
  cont = 0;
  CRC = 0xffff;
  TmoIED[serial] = 0;

  // Le os bytes recebidos pela Interface Serial2
  //Serial.print("MsgRx[");
  //Serial.print(serial);
  //Serial.print("] = ");
  int cntb = 0;
  while ((cntb < NBMsgR) && (cont < 10000)) {
    if (Serial2.available() > 0) {
      MsgRx[cntb] = Serial2.read();
      //Serial.print(MsgRx[cntb]);
      //Serial.print(" ");
      if (!((cntb == 0) && (MsgRx[cntb] == 0))) {
        cntb = cntb + 1;
      }
      delayMicroseconds(EsperaByte);
    }
    cont = cont + 1;
  }
  Serial.println("");
  delay(20);   
  if (serial == 0) {                          // Se esta selecionada a comunicacao com o CC1,
    digitalWrite(RE1, HIGH);                  // desabilita a recepcao do RS485 do CC1
  }
  else {                                      // Se esta selecionada a comunicacao com o CC2,
    digitalWrite(RE2, HIGH);                  // desabilita a recepcao do RS485 do CC2
  }

  if (cont >= 10000) {                         // Se ocorreu timeout na mensagem recebida,
    TmoIED[serial] = 1;                        // aciona o indicador de timeout
  }
  else {                                      // Se nao ocorreu timeout,
    for (int j = 0; j < (NBMsgR - 2); j++) {  // Calcula o CRC da mensagem recebida,
      CRC = CRC ^ MsgRx[j];
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
    
    // Verifica se a mensagem chegou corretamente
    if ((MsgRx[NBMsgR - 2] == ByteL(CRC)) && (MsgRx[NBMsgR - 1] == ByteH(CRC))) {   // Se chegou,
      VerCRCRec[serial] = 1;                                      // aciona o indicador de mensagem OK
      byte d = 0;

      // Converte os Registros do Buffer de Recepcao do Controlador de Carga 1 e carrega no Array de Medidas
      if (serial == 0) {
        for (int i = 0; i < NumReg; i++) {
          Med1[i] = (256 * MsgRx[(2 * i) + 3]) + MsgRx[(2 * i) + 4];
        }

        // Carrega as medidas recebidas do Controlador de Carga 1 nas variaveis
        VP12 = Med1[0];                    // 0x3100 - PV array voltage 1
        IS12 = Med1[1];                    // 0x3101 - PV array current 1
        WS12 = Med1[2] + (256 * Med1[3]);  // 0x3102 - PV array power 1
        VBat1 = Med1[4];                   // 0x3104 - Battery voltage 1
        ISCC1 = Med1[5];                   // 0x3105 - Battery charging current 1
        WSCC1 = Med1[6] + (256 * Med1[7]); // 0x3106 - Battery charging power 1
        TBat1 =  Med1[16];                 // 0x3110 - Battery Temperature 1
      }
      else {  // Converte os Registros do Buffer de Recepcao do Controlador de Carga 2 e carrega no Array de Medidas
        for (int i = 0; i < NumReg; i++) {
          Med2[i] = (256 * MsgRx[(2 * i) + 3]) + MsgRx[(2 * i) + 4];
        }

        // Carrega as medidas recebidas do Controlador de Carga 2 nas variaveis
        VP34 = Med2[0];                    // 0x3100 - PV array voltage 2
        IS34 = Med2[1];                    // 0x3101 - PV array current 2
        WS34 = Med2[2] + (256 * Med2[3]);  // 0x3102 - PV array power 2
        VBat2 = Med2[4];                   // 0x3104 - Battery voltage 2
        ISCC2 = Med2[5];                   // 0x3105 - Battery charging current 2
        WSCC2 = Med2[6] + (256 * Med2[7]); // 0x3106 - Battery charging power 2
        TBat2 =  Med2[16];                 // 0x3110 - Battery Temperature 1
      }
    }
  }

  byte Ch;
  while (Serial2.available() > 0) {
    Ch = Serial2.read();
  }

} // Fim da Rotina TransmiteRecebeMsg()


//*********************************************************************************************************************
// Nome da Rotina: ByteH                                                                                              *
//                                                                                                                    *
// Funcao: obtem o byte mais significativo de um inteiro de dois bytes                                                *
//                                                                                                                    *
// Entrada: valor double                                                                                              *
// Saida: byte mais significativo                                                                                     *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte ByteH(double valor) {
  byte BH = (valor / 256);
  return(BH);
}


//*********************************************************************************************************************
// Nome da Rotina: ByteL                                                                                              *
//                                                                                                                    *
// Funcao: obtem o byte menos significativo de um inteiro de dois bytes                                               *
//                                                                                                                    *
// Entrada: valor double                                                                                              *
// Saida: byte mais significativo                                                                                     *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte ByteL(double valor) {
  byte BH = (valor / 256);
  byte BL = valor - (256 * BH);
  return(BL);
}


//*********************************************************************************************************************
// Nome da Rotina: ByteHigh                                                                                           *
//                                                                                                                    *
// Funcao: obtem o byte mais significativo de um inteiro de tres bytes                                                *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: byte mais significativo                                                                                     *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte ByteHigh(double valor) {
  byte BH = valor / 65536;
  byte BM = (valor - (65536 * BH)) / 256;
  return(BH);
}

//*********************************************************************************************************************
// Nome da Rotina: ByteMid                                                                                            *
//                                                                                                                    *
// Funcao: obtem o byte do meio de um inteiro de tres bytes                                                           *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: byte mais significativo                                                                                     *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte ByteMid(double valor) {
  byte BH = valor / 65536;
  byte BM = (valor - (65536 * BH)) / 256;
  return(BM);
}


//*********************************************************************************************************************
// Nome da Rotina: ByteLow                                                                                            *
//                                                                                                                    *
// Funcao: obtem o byte menos significativo de um inteiro de tres bytes                                               *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: byte menos significativo                                                                                    *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte ByteLow(double valor) {
  byte BH = valor / 65536;
  double V1 = valor - (65536 * BH);
  byte BM = V1 / 256;
  byte BL = V1 - (256 * BM);
  return(BL);
}

//*********************************************************************************************************************
// Nome da Rotina: Centena                                                                                            *
//                                                                                                                    *
// Funcao: obtem a centena mais significativa de um inteiro na faixa de 0000 a 9999                                   *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: centena mais significativa                                                                                  *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte Centena(unsigned int valor) {
  byte CH = valor / 100;
  return(CH);
}

//*********************************************************************************************************************
// Nome da Rotina: Dezena                                                                                             *
//                                                                                                                    *
// Funcao: obtem a dezena de um inteiro na faixa de 0000 a 9999                                  *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: centena menos significativa                                                                                 *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte Dezena(unsigned int valor) {
  byte CH = valor / 100;
  byte CL = valor - (100 * CH);
  return(CL);
}


//*********************************************************************************************************************
// Nome da Rotina: DoisBytesParaInt                                                                                   *
//                                                                                                                    *
// Funcao: converte dois bytes para um inteiro                                                                        *
//                                                                                                                    *
// Entrada: byte menos significativo, byte mais significativo                                                         *
// Saida: valor inteiro                                                                                               *
//                                                                                                                    *
//*********************************************************************************************************************
//
unsigned int DoisBytesParaInt(byte BL, byte BH) {
  int ByteL = BL;
  int ByteH = BH;
  if (BL < 0) { ByteL = 256 + BL; }
  if (BH < 0) { ByteH = 256 + BH; }
  return (ByteL + 256*ByteH);
}
