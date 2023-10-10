//*************************************************************************************************************************
//                                                                                                                        *
//                               Programa do Concentrador de Utilidades Arduino UNO                                       *
//                                                                                                                        *
// Autor: Antonio Bernardo de Vasconcellos Praxedes                                                                       *
// Data: 23/02/2021                                                                                                       *
//                                                                                                                        *
//*************************************************************************************************************************

// Inclusao das bibliotecas usadas no programa
//#include <EEPROM.h>
#include <TimerOne.h>
#include <TimeLib.h>
#include <SPI.h>                 // needed for Arduino versions later than 0018
#include <Ethernet.h>
//#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <Wire.h>

// Definicao das constantes de conversao dos valores analogicos
#define KMed00     25.1163     // Tensao Fase 1
#define OffMed00    0.0        // Offset da Medida00
#define KMed01     25.2042     // Tensao Fase 2
#define OffMed01    0.0        // Offset da Medida01
#define KMed02      4.9378     // Corrente Fase 1
#define OffMed02    0.0        // Offset
#define KMed03      9.7820     // Corrente Fase 2
#define OffMed03    0.0        // Offset
#define KMed04      0.0        // Constante
#define OffMed04    0.0        // Offset
#define KMed05      0.0        // Constante
#define OffMed05    0.0        // Offset

// Definicao dos numeros dos pinos para leitura das Entradas Analogicas (EA)
#define EA0                  0   // EA0
#define EA1                  1   // EA1
#define EA2                  2   // EA2
#define EA3                  3   // EA3
#define EA4                  4   // EA4
#define EA5                  5   // EA5

// Definicao dos numeros dos pinos para leitura das Entradas Digitais (ED) - Ativas em Nivel Zero
#define ED0                  2   //
#define ED1                  3   // 
#define ED2                  4   // 
#define ED3                  5   // 
#define ED4                  6   //  
#define ED5                  7   // 

#define SDLED               13

#define BaseTempo         4167

#define NumAmostras        256


//**************************************************************************************************************************
//                                                                                                                         *
//                                                 Declaracao das Variaveis                                                *
//                                                                                                                         *
//**************************************************************************************************************************
//

// Array com os estados das Entradas Digitais (ED)
byte ED[8];

// Medidas
long SomaTensao1 = 0;
long SomaTensao1F = 0;
long Tensao1 = 0;

long SomaTensao2 = 0;
long SomaTensao2F = 0;
long Tensao2 = 0;

long SomaCorrente1 = 0;
long SomaCorrente1F = 0;
long Corrente1 = 0;

long SomaCorrente2 = 0;
long SomaCorrente2F = 0;
long Corrente2 = 0;


int contador = 0;
int ContAmostras = 0;

// Relogio
byte HoraRec;
byte MinutoRec;
byte SegundoRec;
byte DiaRec;
byte MesRec;
int AnoRec;

// Definir Endereco MAC e Endereco IP do Medidor
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE0 };
IPAddress ip(192,168,0,152);
//unsigned int localPort = 5683;
unsigned int localPort = 100;

// Variaveis da comunicacao ethernet
const unsigned int TamBufTx = 256;
unsigned int Indice = 0;
unsigned int Contador = 0;
byte BufSVCheio = 0;
byte HabAquisicao = 0;
byte MsgTr = 0;

byte TxBufEst[84];

// buffers para enviar e receber dados pela interafce ethernet
char RxBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

tmElements_t tm;

 
//****************************************************************************************************************************
//                                                                                                                           *
//                                               Procedimentos de Inicializacao                                              *
//                                                                                                                           *
//****************************************************************************************************************************
//
void setup() {

  // Definir a Funcao dos Pinos de Entrada Digital
  pinMode(ED0, INPUT);
  pinMode(ED1, INPUT);
  pinMode(ED2, INPUT);
  pinMode(ED3, INPUT);
  pinMode(ED4, INPUT);
  pinMode(ED5, INPUT);
  
  pinMode(SDLED, OUTPUT);
  
  digitalWrite(SDLED, LOW); // Inicia o LED da placa apagado
  
  HoraRec = 0;
  MinutoRec = 0;
  SegundoRec = 0;
  DiaRec = 1;
  MesRec = 1;
  AnoRec = 2021;

  // Inicia a Interface Serial Assincrona
  Serial.begin(9600);

  // Inicia a Ethernet e o UDP:
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.");
  }
  else if (Ethernet.hardwareStatus() == EthernetW5100) {
    Serial.println("W5100 Ethernet controller detected.");
  }
  else if (Ethernet.hardwareStatus() == EthernetW5200) {
    Serial.println("W5200 Ethernet controller detected.");
  }
  else if (Ethernet.hardwareStatus() == EthernetW5500) {
    Serial.println("W5500 Ethernet controller detected.");
  }
  

  // Inicia o timer e a chamada da rotina de interrupcao
  //Timer1.initialize(BaseTempo);
  //Timer1.attachInterrupt(Aquisicao);
    
} // Fim dos Procedimentos de Inicializacao


//**************************************************************************************************************************
//                                                                                                                         *
//                                              Inicio do Programa Principal                                               *
//                                                                                                                         *
//**************************************************************************************************************************
//
void loop() {

  // Comunicacao Serial
  //ComunicacaoSerial();
  
  // Comunicacao Ethernet
  ComunicacaoEthernet();
  
  delay(50);
            
} // Final do Loop Principal do Programa


//**************************************************************************************************************************
// Nome da Rotina: Aquisicao                                                                                               *
// Funcao: aquisicao de medidas                                                                                            *
// Entradas: nenhuma                                                                                                       *
// Saidas: nenhuma                                                                                                         *
//**************************************************************************************************************************
//
void Aquisicao() {

  SomaTensao1 = SomaTensao1 + analogRead(EA0);
  SomaTensao2 = SomaTensao2 + analogRead(EA1);
  SomaCorrente1 = SomaCorrente1 + analogRead(EA2);
  SomaCorrente2 = SomaCorrente2 + analogRead(EA3);
  
  ContAmostras = ContAmostras + 1;
  if (ContAmostras >= NumAmostras) {
    SomaTensao1F = SomaTensao1;
    SomaTensao2F = SomaTensao2;
    SomaCorrente1F = SomaCorrente1;
    SomaCorrente2F = SomaCorrente2;
    SomaTensao1 = 0;
    SomaTensao2 = 0;
    SomaCorrente1 = 0;
    SomaCorrente2 = 0;
    ContAmostras = 0;
  }
      
} // fim da rotina Aquisicao


//*******************************************************************************************************
// Nome da Rotina: ComunicacaoSerial                                                                    *
// Funcao: verifica se tem requisicao pela interface serial e responde a solicitacao                    *
// Entrada: nenhum                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void ComunicacaoSerial() {
   
   // Verificacao da Requisicao de Leitura de Estados pela Interface Serial
  if (Serial.available() == 1) {
    byte ByteRec = Serial.read();
   
    if (ByteRec == 'a') {
      Tensao1 = KMed00 * (SomaTensao1F / NumAmostras);
      Tensao2 = KMed01 * (SomaTensao2F / NumAmostras);
      Corrente1 = KMed02 * (SomaCorrente1F / NumAmostras);
      Corrente2 = KMed03 * (SomaCorrente2F / NumAmostras);
      Serial.print("Tensao 1 = ");
      Serial.print(Tensao1);
      Serial.print(" Vrms   Corrente 1 = ");
      Serial.print(Corrente1);
      Serial.println(" mA");
      Serial.print("Tensao 2 = ");
      Serial.print(Tensao2);
      Serial.print(" Vrms   Corrente 2 = ");
      Serial.print(Corrente2);
      Serial.println(" mA");
    }
    
    ByteRec = ' ';
  } // if (Serial.available() == 1)

} // Fim da Rotina ComunicacaoSerial


//*******************************************************************************************************
// Nome da Rotina: ComunicacaoEthernet                                                                  *
// Funcao: verifica se tem requisicao pela interface ethernet (UDP) e responde a solicitacao            *
// Entrada: nenhum                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void ComunicacaoEthernet() {

  byte Comando = 0; 
  int packetSize = Udp.parsePacket();
  if (packetSize) {                             // Verifica se chegou mensagem UDP pela ethernet
    IPAddress remote = Udp.remoteIP();          // Se chegou mensagem, obter o endereco IP do cliente
    Udp.read(RxBuffer, UDP_TX_PACKET_MAX_SIZE); // e ler o pacote de dados recebido do cliente
    
    Comando = RxBuffer[0];                      // Obtem o codigo do comando enviado pelo cliente
    ExecutaComando(Comando);                    // Executar o comando referente ao codigo recebido
    Comando = 0; 
    Serial.println("Recebida Mensagem UDP pela Ethernet");
    
  } // if (packetSize)
  
    
} // Fim da Rotina ComunicacaoEthernet


//***************************************************************************************************
// Nome da Rotina: ExecutaComando                                                                   *
// Funcao: executa um comando                                                                       *
// Entrada: codigo de identificacao do comando e habilitacao de impressao de mensagem               *
// Saida: nenhum                                                                                    *
//***************************************************************************************************
//
void ExecutaComando(byte codigo) {

switch (codigo) {
   
  case 1: // Resposta a Solicitacao de Leitura de Estados
    MontaMsgEstados(codigo);
  break;

  case  2: 
    
  break;

  case 3: // Comando de Acerto de Relogio
//    HoraRec = RxBuffer[1];
//    MinutoRec = RxBuffer[2];
    SegundoRec = RxBuffer[3];
    DiaRec = RxBuffer[4];
    MesRec = RxBuffer[5];
    AnoRec = (RxBuffer[6] + 256*RxBuffer[7]);
    
    MontaMsgEstados(codigo);
  break;

  case 4:
   
  break;
  
  case 5:

  break;
 
  }
  
} // Fim da rotina ExecutaComando


//*******************************************************************************************************
// Nome da Rotina: MontaMsgEstados                                                                      *
// Funcao: monta e transmite a mensagem de resposta com os estados da UTR                               *
// Entrada: codigo do comando que esta mensagem foi usada como resposta                                 *
// Saida: preenche o array TxBufEst e transmite pela ethernet em protocolo UDP                          *
//*******************************************************************************************************
//
void MontaMsgEstados(byte Codigo) {

  // Cabecalho da Mensagem: bytes 000 a 015
  TxBufEst[0] = 2;                 // Endereco da UTR
  TxBufEst[1] = 1;                 // Modelo da UTR (1 = Arduino UNO)
  TxBufEst[2] = 0;                 // Modo (AcertaRelogio=0 ou AcertaRelogio=1)
  TxBufEst[3] = 8;                 // Numero de Estados Digitais Configurados (max 64)
  TxBufEst[4] = 6;                 // Numero de Medidas Configuradas (max 32)
  
  TxBufEst[15] = Codigo;                 // O comando foi recebido e executado (codigo do comando)
  
  // Estados de Indicacao Digital (16): bytes 016 a 031 - 1 byte por estado
  TxBufEst[16] = 0;    // Estado Digital 00
  TxBufEst[17] = 0;    // Estado Digital 01
  TxBufEst[18] = 0;    // Estado Digital 02
  TxBufEst[19] = 0;    // Estado Digital 03
  
  // Alarmes (16): bytes 032 a 047 - 1 byte por alarme
  TxBufEst[32] = 0;
  TxBufEst[33] = 0;
  TxBufEst[34] = 0;
  TxBufEst[35] = 0;
  TxBufEst[36] = 0;
    
  // Medidas (8): bytes 048 a 071 - 3 bytes por medida
  // Medida 00
  TxBufEst[48] = 0;
  TxBufEst[49] = 0;
  TxBufEst[50] = 0;

  // Medida 01
  TxBufEst[51] = 0;
  TxBufEst[52] = 0;
  TxBufEst[53] = 0;

  // Medida 02
  TxBufEst[54] = 0;
  TxBufEst[55] = 0;
  TxBufEst[56] = 0;
  
  // Medida 03
  TxBufEst[57] = 0;
  TxBufEst[58] = 0;
  TxBufEst[59] = 0;

  // Medida 04
  TxBufEst[60] = 0;
  TxBufEst[61] = 0;
  TxBufEst[62] = 0;
  
  // Medida 05
  TxBufEst[63] = 0;
  TxBufEst[64] = 0;
  TxBufEst[65] = 0;
  
  // Medida 06
  TxBufEst[66] = 0;
  TxBufEst[67] = 0;
  TxBufEst[68] = 0;
  
  // Medida 07
  TxBufEst[69] = 0;
  TxBufEst[70] = 0;
  TxBufEst[71] = 0; // Byte de Estado da Medida 07
  
  
   // Estados de Saidas Digitais (4): bytes 072 a 075 - 1 byte por estado
  TxBufEst[72] = 0;
  TxBufEst[73] = 0;
  TxBufEst[74] = 0;
  TxBufEst[75] = 0;
 
   // Estados de Saidas Analogicas (4): bytes 076 a 079 - 2 bytes por estado
  TxBufEst[76] = 0;
  TxBufEst[77] = 0;
  TxBufEst[78] = 0;
  TxBufEst[79] = 0;

// Verificacao de erro: bytes 080 a 083
  unsigned int soma = 0;
  for (int i = 0; i < 80; i++) {
    soma += TxBufEst[i];
  }
  TxBufEst[80] = ByteLow(soma);
  TxBufEst[81] = ByteHigh(soma);
  TxBufEst[82] = 127;
  TxBufEst[83] = 255;

  // Transmite a mensagem para o cliente
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(TxBufEst, 84);
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
// Nome da Rotina: Centena                                                                          *
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
