//*****************************************************************************************************
//                                                                                                    *
//                     Programa do Controlador da Agua Quente Arduino UNO                             *
//                                                                                                    *
// Autor: Antonio Bernardo de Vasconcellos Praxedes                                                   *
//                                                                                                    *
// Data: 13/09/2023                                                                                   *
//                                                                                                    *
//*****************************************************************************************************

// Inclusao das bibliotecas usadas no programa
#include <EEPROM.h>
#include <TimerOne.h>
#include <TimeLib.h>
#include <SPI.h>                 // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

// Definicao das constantes de conversao dos valores analogicos
#define KMed00      10.00   // Constante de temperatura do Boiler
#define OffMed00    0.0     // Offset da Medida00
#define KMed01      10.00   // Constante de temperatura da Placa Solar
#define OffMed01    0.0     // Offset da Medida01
#define KMed02      0.0     // Constante
#define OffMed02    0.0     // Offset
#define KMed03      0.0     // Constante
#define OffMed03    0.0     // Offset
#define KMed04      0.0     // Constante
#define OffMed04    0.0     // Offset
#define KMed05      0.0     // Constante
#define OffMed05    0.0     // Offset

// Definicao dos numeros dos pinos para leitura das Entradas Analogicas (EA)
#define EA0                  0   // EA0 - Sensor de Temperatura do Boiler
#define EA1                  1   // EA1 - Sensor de Temperatura da Placa Solar
#define EA2                  2   // EA2 - Sensor de Temperatura da Geladeira 
#define EA3                  3   // EA3
#define EA4                  4   // EA4
#define EA5                  5   // EA5

// Definição dos pinos de leitura Entrada Digitais
#define flowPin              2   // Recebe os pulsos do sensor de fluxo de água da bomba do poço

// Definicao dos numeros dos pinos para escrita nas Saidas Digitais (SD) - Ativas em Nivel Um
#define SD0                  8  // Saida Digital 0: Controle do Rele da Resistencia do Boiler
#define SD1                  9  // Saida Digital 1: Controle da Bomba do Boiler para a Placa Solar

#define BaseTempo       999673   // 1000000


//******************************************************************************************************
//                                                                                                     *
//                                    Declaracao das Variaveis                                         *
//                                                                                                     *
//******************************************************************************************************
//
// Array com os estados das Entradas Analogicas
unsigned int EA[6];

// Array com os estados das Entradas Digitais (ED)
byte ED[8];

// Array com os estados das Saidas Digitais (SD)
byte SDg[4];

// *** Medida00 = Temperatura 1 em Graus Celsius x 100 ***
int Medida00;            // Valor da Medida00 
byte EstMed00;           // Estado da Medida 00
byte EstAntMed00;        // Estado Anterior da Medida00

// *** Medida01 = Temperatura 2 em Graus Ceusius x 100 ***
int Medida01;              // Valor da Medida01 
byte EstMed01;             // Estado da Medida 01
byte EstAntMed01;          // Estado Anterior da Medida01

// *** Medida02 ***
int Medida02;              // Valor da Medida 02 
byte EstMed02;             // Estado da Medida 02
byte EstAntMed02;          // Estado Anterior da Medida02

// *** Medida03 ***
int Medida03;              // Valor da Medida03
byte EstMed03;             // Estado da Medida 03
byte EstAntMed03;          // Estado Anterior da Medida03

// *** Medida04 ***
int Medida04;              // Valor da Medida04
byte EstMed04;             // Estado da Medida 04
byte EstAntMed04;          // Estado Anterior da Medida04

// *** Medida05 = Reserva ***
int Medida05;              // Valor da Medida05
byte EstMed05;             // Estado da Medida 05
byte EstAntMed05;          // Estado Anterior da Medida05

//boolean HabContTmpBomba;       // Flag de Habilitacao da contagem de tempo da bomba
unsigned int CntTmpBmbLig;     // Contador de tempo que a bomba fica ligada em cada vez

//unsigned int CntTmpBmbLigDia;  // Contador de tempo que a bomba fica ligada durante um dia
boolean HabLigBmb;

boolean FlagCont1;
boolean FlagCont2;
unsigned int Contador1;
unsigned int Contador2;

unsigned int ContPulsosFluxo;
unsigned int PulsosPorSeg;
unsigned int TmpBmbLig;
double PulsosBmbLigada;
boolean BmbPocoLig;
double VazaoMedia;

boolean LED = false;

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

// Variaveis de gerenciamento dos Registros de Eventos na EEPROM
int PontReg; // Ponteiro para a proxima posicao a ser gravado registro na EEPROM
int NumReg;  // Numero de Registros de Eventos disponiveis na EEPROM

// Definir Endereco MAC e Endereco IP do Controlador de Agua Quente
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,0,155);

// Variaveis da comunicacao ethernet
const unsigned int TamBufTx = 256;
unsigned int Indice = 0;
unsigned int localPort = 100;
unsigned int Contador = 0;
byte BufSVCheio = 0;
byte HabAquisicao = 0;
byte MsgTr = 0;

byte TxBufEst[84];
byte TxBufConf[20];

// buffers para enviar e receber dados pela interafce ethernet
char RxBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,

// An EthernetUDP instance to let us send and receive packets over UDP
 EthernetUDP Udp;

tmElements_t tm;

 
//********************************************************************************************************
//                                                                                                       *
//                                 Procedimentos de Inicializacao                                        *
//                                                                                                       *
//********************************************************************************************************
//
void setup() {

  // Definir a Funcao do Pino de Entrada Digital para Contagem de Pulsos do Sensor de Vazão
  pinMode(flowPin, INPUT);
  
  // Definir a Funcao dos Pinos de Saida Digital
  pinMode(SD0, OUTPUT);
  pinMode(SD1, OUTPUT);
  
  // Inicia todas as sadas digitais com valor zero
  digitalWrite(SD0, LOW);
  SDg[0] = LOW;
  digitalWrite(SD1, LOW);
  SDg[1] = LOW;
  
  // Inicia todas as Entradas Analogicas com valor Zero
  for (int i = 0; i < 6; i++) {
    EA[i] = 0;
  }

  CntTmpBmbLig = 0;
  
  FlagCont1 = false;
  FlagCont2 = false;
  Contador1 = 0;
  Contador2 = 0;
  
  HabLigBmb = true;

  ContPulsosFluxo = 0;
  PulsosPorSeg = 0;
  TmpBmbLig = 0;
  PulsosBmbLigada = 0;
  BmbPocoLig = false;
  VazaoMedia = 0;
    
  HoraRec = 0;
  MinutoRec = 0;
  SegundoRec = 0;
  DiaRec = 1;
  MesRec = 1;
  AnoRec = 2017;

  // Inicia a Interface Serial Assincrona
  Serial.begin(9600);

  // Inicia a Ethernet e o UDP:
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  // Inicia o timer e a chamada da rotina de interrupcao
  Timer1.initialize(BaseTempo);
  Timer1.attachInterrupt(Temporizacao);
  attachInterrupt(0, Flow, RISING); // Pino de Entrada do Sensor de Vazão para a Interrupção 0 na Borda de Subida

  // Inicia as Variavies de Medidas e de Estados Atuais e Anteriores
  EstMed00 = 0;
  EstMed01 = 0;
  EstMed02 = 0;
  EstMed03 = 0;
  EstMed04 = 0;
  EstMed05 = 0;
 
  Aquisicao();
  CarregaMedidas();

  FlagCont2 = true;
  interrupts();
   
} // Fim dos Procedimentos de Inicializacao


//******************************************************************************************************
//                                                                                                     *
//                                   Inicio do Programa Principal                                      *
//                                                                                                     *
//******************************************************************************************************
//
void loop() {

  Aquisicao();
    
  CarregaMedidas();

  ControleBomba(500);

  ControleAquecedor(3100, 3600);
 
  ComunicacaoEthernet();

  if ((PulsosPorSeg > 10) && (!BmbPocoLig)) {
    BmbPocoLig = true;
    TmpBmbLig = 0;
    PulsosBmbLigada = 0;
  }
  if ((BmbPocoLig) && (PulsosPorSeg <= 10)) {
    BmbPocoLig = false;
  }

  delay(50);
            
} // Final do Loop Principal do Programa


//*******************************************************************************************************
// Nome da Rotina: Aquisicao                                                                            *
// Funcao: aquisicao de medidas por amostragem periodica usando timer por interrupcao                   *
// Entradas: nenhuma                                                                                    *
// Saidas: nenhuma                                                                                      *
//*******************************************************************************************************
//
void Aquisicao() {

  // Carrega os valores dos pontos analogicos nas variaveis de Entradas Analogicas
  EA[0] = analogRead(EA0);
  EA[1] = analogRead(EA1);
  EA[2] = analogRead(EA2);
  EA[3] = analogRead(EA3);
  EA[4] = analogRead(EA4);
  EA[5] = analogRead(EA5);
      
} // fim da rotina Aquisicao


//*******************************************************************************************************
// Nome da Rotina: CarregaMedidas                                                                       *
// Funcao: carrega as medidas a partir das entradas analogicas (fisicas ou simuladas)                   *
// Entrada: nenhum                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void CarregaMedidas() {

  Medida00 = (KMed00*EA[0]) + OffMed00;
  Medida01 = (KMed01*EA[1]) + OffMed01;
  Medida02 = (KMed02*EA[2]) + OffMed02;
  Medida03 = (KMed03*EA[3]) + OffMed03;
  Medida04 = (KMed04*EA[4]) + OffMed04;
  Medida05 = (KMed05*EA[5]) + OffMed05;
  
} // Fim da Rotina CarregaMedidas


//*******************************************************************************************************
// Nome da Rotina: ControleBomba                                                                        *
//                                                                                                      *
// Funcao: controla a bomba que circula a água do boiler pela placa solar                               *
//                                                                                                      *
// Entrada: nenhum                                                                                      *
//                                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void ControleBomba(int DifTempLiga) {

  if (Medida00 > 1500) {    // Se o sensor de temperatura do boiler está OK.
    if (Medida01 > 3000) {  // Se a placa solar está com mais de 30 graus
  
      // Se a bomba esta desligada verifica:
      // Se a temperatura da placa solar for maior que a temperatura do boiler mais 5 graus e
      // se a temperatura do boiler for menor que 50 graus, liga a bomba.
      if (SDg[1] == LOW) {
        if ((Medida01 > (Medida00 + DifTempLiga)) && (Medida00 < 5000)) {
            SDg[1] = HIGH;              // Sinaliza a bomba ligada
            digitalWrite(SD1, HIGH);    // Liga a bomba
            Contador1 = 0;              // zera o contador de tempo da bomba ligada,
            FlagCont1 = true;           // habilita a contagem de tempo de bomba ligada.
        }
      }

      // Se a bomba esta ligada verifica:
      // Se a temperatura da placa solar for menor que a temperatura do boiler, 
      // ou se a temperatura do boiler for maior que 60 graus, desliga a bomba.
      if (SDg[1] == HIGH) {
        if ((Medida01 < Medida00) || (Medida00 > 5000)) {
          SDg[1] = LOW;                 // Sinaliza a bomba desligada
          digitalWrite(SD1, LOW);       // Desliga a bomba
          FlagCont1 = false;            // desabilita a contagem de tempo da bomba ligada.
        }
      }
    }
    else {
      SDg[1] = LOW;                 // Sinaliza a bomba desligada
      digitalWrite(SD1, LOW);       // Desliga a bomba
      FlagCont1 = false;            // desabilita a contagem de tempo da bomba ligada.
    }
  }
  else {
    SDg[1] = LOW;                 // Sinaliza a bomba desligada
    digitalWrite(SD1, LOW);       // Desliga a bomba
    FlagCont1 = false;            // desabilita a contagem de tempo da bomba ligada.
  }

}


//*******************************************************************************************************
// Nome da Rotina: ControleAquecedor                                                                    *
//                                                                                                      *
// Funcao: controla a resistẽncia de aquecimento do boiler                                              *
//                                                                                                      *
// Entrada: temperatura mínima para ligar e temperatura máxima para desligar a resistência              *
//                                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void ControleAquecedor(int TempMin, int TempMax) {

  if (Medida00 > 1500) { // Se o sensor de temperatura do boiler está OK.

    // Se a temperatura do boiler for menor que 33 graus liga o aquecedor eletrico
    if (Medida00 < TempMin) {
      SDg[0] = HIGH;
      digitalWrite(SD0, HIGH);
    }
  
    // Se a temperatura do boiler for maior que 38 graus desliga o aquecedor eletrico
    if (Medida00 > TempMax) {
      SDg[0] = LOW;
      digitalWrite(SD0, LOW);
    }

  }
  else {
    SDg[0] = LOW;
    digitalWrite(SD0, LOW);
  }

}


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

    case  2: // Zera o contador de pulsos até encher a caixa
      PulsosBmbLigada = 0;
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
  
  TxBufEst[15] = Codigo;           // O comando foi recebido e executado (codigo do comando)
    
  // Medidas (8): bytes 048 a 071 - 3 bytes por medida
  
  // Medida Especial 01
  TxBufEst[32] = ByteLow(VazaoMedia);
  TxBufEst[33] = ByteHigh(VazaoMedia);
  TxBufEst[34] = 0;
  
  // Medida 00
  TxBufEst[48] = ByteLow(Medida00);
  TxBufEst[49] = ByteHigh(Medida00);
  TxBufEst[50] = EstMed00; // Byte de Estado da Medida 00

  // Medida 01
  TxBufEst[51] = ByteLow(Medida01);
  TxBufEst[52] = ByteHigh(Medida01);
  TxBufEst[53] = EstMed01; // Byte de Estado da Medida 01

  // Medida 02
  TxBufEst[54] = ByteLow(Medida02);
  TxBufEst[55] = ByteHigh(Medida02);
  TxBufEst[56] = EstMed02; // Byte de Estado da Medida 02
  
  // Medida 03
  TxBufEst[57] = ByteLow(Medida03);
  TxBufEst[58] = ByteHigh(Medida03);
  TxBufEst[59] = EstMed03; // Byte de Estado da Medida 03

  // Medida 04
  TxBufEst[60] = ByteLow(Medida04);
  TxBufEst[61] = ByteHigh(Medida04);
  TxBufEst[62] = EstMed04; // Byte de Estado da Medida 04
  
  // Medida 05
  TxBufEst[63] = ByteLow(Medida05);
  TxBufEst[64] = ByteHigh(Medida05);
  TxBufEst[65] = EstMed05; // Byte de Estado da Medida 05
  
  // Medida 06
  TxBufEst[66] = ByteLow(Contador1);
  TxBufEst[67] = ByteHigh(Contador1);
  TxBufEst[68] = 0;
  
  // Medida 07
  TxBufEst[69] = 0;
  TxBufEst[70] = 0;
  TxBufEst[71] = 0;
   
  // Estados de Saidas Digitais (4): bytes 072 a 075 - 1 byte por estado
  TxBufEst[72] = SDg[0];
  TxBufEst[73] = SDg[1];
  TxBufEst[74] = SDg[2];
  TxBufEst[75] = SDg[3];
 
   // Vazão da Bomba do Poço
  TxBufEst[76] = ByteLow(PulsosPorSeg);
  TxBufEst[77] = ByteHigh(PulsosPorSeg);
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
// Nome da Rotina: Temporizacao                                                                         *
// Modo de Chamada: interrupcao de timer                                                                *
// Funcao: incrementa os contadores de tempo (Contador1 a n) se os flags = true (FlagCont1 a n)         *
// Entradas: nenhuma                                                                                    *
// Saidas: nenhuma                                                                                      *
//*******************************************************************************************************
//
void Temporizacao() {

  if (FlagCont1) {
    Contador1 += 1;
  }

  if (BmbPocoLig) {
    TmpBmbLig++;
  }
  
  PulsosPorSeg = ContPulsosFluxo;
  PulsosBmbLigada = PulsosBmbLigada + PulsosPorSeg;
  if (TmpBmbLig > 0) {
    VazaoMedia = 100 * ((PulsosBmbLigada / TmpBmbLig) / 4.8);
  }
  
  ContPulsosFluxo = 0;

  Serial.println(BmbPocoLig);
  Serial.println(PulsosPorSeg);
  Serial.println(PulsosBmbLigada);
  Serial.println(TmpBmbLig);
  Serial.println(VazaoMedia);

} // fim da rotina Temporizacao


//*******************************************************************************************************
// Nome da Rotina: Flow                                                                                 *
// Funcao: incrementa o contador de vazão ao receber um pulso do sensor                                 *
// Modo de Chamada: interrupcao de entrada digital                                                      *
// Entradas: nenhuma                                                                                    *
// Saidas: nenhuma                                                                                      *
//*******************************************************************************************************
//
void Flow() {
   ContPulsosFluxo++;   //Quando essa função é chamada, soma-se 1 a variável "count" 
}


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
