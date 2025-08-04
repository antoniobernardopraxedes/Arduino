//*****************************************************************************************************
//                                                                                                    *
//              Programa do Controlador da Agua Quente Casa Vermelha Arduino UNO                      *
//                                                                                                    *
// Autor: Antonio Bernardo de Vasconcellos Praxedes                                                   *
//                                                                                                    *
// Data: 31/07/2025                                                                                   *
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
#define KMed00      8.474576 // Constante de temperatura do Boiler
#define OffMed00    0.0      // Offset da Medida00
#define KMed01      25.1163  // Constante de tensão CA da rede
#define OffMed01    0.0      // Offset da Medida01
#define KMed02      1.5686   // Constante da Corrente do Boiler
#define OffMed02    0.0      // Offset
#define KMed03      1.0      // Constante
#define OffMed03    0.0      // Offset
#define KMed04      1.0      // Constante
#define OffMed04    0.0      // Offset
#define KMed05      1.0      // Constante
#define OffMed05    0.0      // Offset

// Definicao dos numeros dos pinos para leitura das Entradas Analogicas (EA)
#define EA0                  0   // EA0 - Sensor de Temperatura do Boiler
#define EA1                  1   // EA1 - Sensor de Tensão da Rede
#define EA2                  2   // EA2 - Sensor de Corrente da Resistência do Boiler
#define EA3                  3   // EA3
#define EA4                  4   // EA4
#define EA5                  5   // EA5

// Definicao dos numeros dos pinos para escrita nas Saidas Digitais (SD) - Ativas em Nivel Um
#define SD0                  8  // Saida Digital 0: Controle do Rele da Resistencia do Boiler

#define TMaxBoiler        4500  // Temperatura máxima do boiler para controle da resistência de aquecimento
#define TMinBoiler        4000  // Temperatura mínima do boiler para controle da resistência de aquecimento

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

// *** Medida01 = Tensão da Rede x 100 ***
int Medida01;              // Valor da Medida01 
byte EstMed01;             // Estado da Medida 01

// *** Medida02 = Corrente na Resistência do Boiler x 100
int Medida02;              // Valor da Medida 02 
byte EstMed02;             // Estado da Medida 02

// *** Medida03 ***
int Medida03;              // Valor da Medida03
byte EstMed03;             // Estado da Medida 03

// *** Medida04 ***
int Medida04;              // Valor da Medida04
byte EstMed04;             // Estado da Medida 04

// *** Medida05 = Reserva ***
int Medida05;              // Valor da Medida05
byte EstMed05;             // Estado da Medida 05

boolean FlagCont1;
unsigned int Contador1;

boolean LED = false;

// Relogio
byte Hora;
byte Minuto;
byte Segundo;
byte Dia;
byte Mes;
int Ano;

// Flag que indica Acerto de Relogio pelo Host
byte AcertaRelogio;

// Variaveis de gerenciamento dos Registros de Eventos na EEPROM
int PontReg; // Ponteiro para a proxima posicao a ser gravado registro na EEPROM
int NumReg;  // Numero de Registros de Eventos disponiveis na EEPROM

// Definir Endereco MAC e Endereco IP do Controlador de Agua Quente da Casa Vermelha
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xDF };
IPAddress ip(192,168,0,152);

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

// buffers para enviar e receber dados pela interaface ethernet
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

  // Definir a Funcao dos Pinos de Saida Digital
  pinMode(SD0, OUTPUT);
      
  // Inicia todas as saídas digitais com valor zero
  digitalWrite(SD0, LOW);
      
  // Inicia todas as Entradas Analogicas com valor Zero
  for (int i = 0; i < 6; i++) {
    EA[i] = 0;
  }

  FlagCont1 = false;
  Contador1 = 0;
    
  // Inicia a Interface Serial Assincrona
  Serial.begin(9600);

  // Inicia a Ethernet e o UDP:
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  // Inicia o timer e a chamada da rotina de interrupcao
  Timer1.initialize(BaseTempo);
  Timer1.attachInterrupt(Temporizacao);

  // Inicia as Variavies de Medidas e de Estados Atuais e Anteriores
  EstMed00 = 0;
  EstMed01 = 0;
  EstMed02 = 0;
  EstMed03 = 0;
  EstMed04 = 0;
  EstMed05 = 0;
 
  Aquisicao();
  CarregaMedidas();

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

  ControleAquecedor(TMinBoiler, TMaxBoiler);
 
  ComunicacaoEthernet();

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

  if (Medida00 > 1000) {        // Se o sensor de temperatura do boiler está OK.

    if (Medida00 < TempMin) {   // Se a temperatura do boiler for menor que a temperatura mínima,
      digitalWrite(SD0, HIGH);  // liga o aquecedor eletrico, e
      FlagCont1 = true;         // habilita a contagem de tempo.
    }
  
    if (Medida00 > TempMax) {  // Se a temperatura do boiler for maior que a temperatura máxima,
      digitalWrite(SD0, LOW);  // desliga o aquecedor eletrico, e
      FlagCont1 = false;       // desabilita a contagem de tempo.
    }

  }
  else {                       // Se o sensor de temperatura não está funcionando,
    digitalWrite(SD0, LOW);    // desliga o aquecedor eletrico, e
    FlagCont1 = false;         // Desabilita a contagem de tempo.
  }
}


//*********************************************************************************************************************
// Nome da Rotina: ComunicacaoEthernet                                                                                *
//                                                                                                                    *
// Funcao: verifica se tem requisicao pela interface ethernet (UDP), recebe a mensagem, encaminha para o              *
//         multimedidor em protocolo MODBUS RTU, espera a resposta do Multimedidor e envia os bytes da resposta       *
//         pela ethernet.                                                                                             *
//                                                                                                                    *
// Entrada: nenhum                                                                                                    *
//                                                                                                                    *
// Saida: nenhum                                                                                                      *
//*********************************************************************************************************************
//
void ComunicacaoEthernet() {

  int packetSize = Udp.parsePacket();
  if (packetSize) {                             // Verifica se chegou mensagem UDP pela ethernet
    IPAddress remote = Udp.remoteIP();          // Se chegou mensagem, obter o endereco IP do cliente
    Udp.read(RxBuffer, UDP_TX_PACKET_MAX_SIZE); // e ler o pacote de dados recebido do cliente

    Serial.println(analogRead(EA1));
    MontaMsgEstados(0);
    Contador1 = 0;
        
  } // if (packetSize)
    
} // Fim da Rotina ComunicacaoEthernet


//*******************************************************************************************************
// Nome da Rotina: MontaMsgEstados                                                                      *
// Funcao: monta e transmite a mensagem de resposta com os estados da UTR                               *
// Entrada: codigo do comando que esta mensagem foi usada como resposta                                 *
// Saida: preenche o array TxBufEst e transmite pela ethernet em protocolo UDP                          *
//*******************************************************************************************************

void MontaMsgEstados(byte codigo) {

  // Cabecalho da Mensagem: bytes 000 a 015
  TxBufEst[0] = 2;                 // Endereco da UTR
  TxBufEst[1] = 1;                 // Modelo da UTR (1 = Arduino UNO)
  TxBufEst[2] = 0;                 // Modo (AcertaRelogio=0 ou AcertaRelogio=1)
  TxBufEst[3] = 8;                 // Numero de Estados Digitais Configurados (max 64)
  TxBufEst[4] = 6;                 // Numero de Medidas Configuradas (max 32)
  
  TxBufEst[15] = codigo;           // O comando foi recebido e executado (codigo do comando)
    
  // Medidas (8): bytes 048 a 071 - 3 bytes por medida
  
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
  TxBufEst[72] = digitalRead(SD0);
  TxBufEst[73] = 0;
  TxBufEst[74] = 0;
  TxBufEst[75] = 0;
 
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

} // fim da rotina Temporizacao


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
