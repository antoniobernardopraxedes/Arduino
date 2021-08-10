//*****************************************************************************************************
//                                                                                                    *
//             Programa do Controlador da Agua Quente Arduino UNO                                     *
//                                                                                                    *
// Autor: Antonio Bernardo de Vasconcellos Praxedes                                                   *
// Data: 13/06/2020                                                                                   *
//                                                                                                    *
//*****************************************************************************************************

// Inclusao das bibliotecas usadas no programa
#include <EEPROM.h>
#include <SPI.h>                 // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
//#include <Wire.h>

// Definicao das constantes de conversao dos valores analogicos
#define KMed00      10.00   // Constante de temperatura do Boiler
#define OffMed00    0.0     // Offset da Medida00
#define KMed01      10.00   // Constante de temperatura da Placa Solar
#define OffMed01    0.0     // Offset da Medida01
#define KMed02      1.9949  // Constante da Temperatura da Geladeira
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

// Definicao dos numeros dos pinos para leitura das Entradas Digitais (ED) - Ativas em Nivel Zero
#define ED0                  2   //
#define ED1                  3   // 
#define ED2                  4   // 
#define ED3                  5   // 
#define ED4                  6   //  
#define ED5                  7   // 

// Definicao dos numeros dos pinos para escrita nas Saidas Digitais (SD) - Ativas em Nivel Um
#define SD0                  8  // Saida Digital 0: Controle do Rele da Resistencia do Boiler
#define SD1                  9  // Saida Digital 1: Controle da Bomba do Boiler para a Placa Solar

#define NumMaxSD             2   // Numero maximo de Saidas Digitais
#define NumMaxSA             4   // Numero maximo de Saidas Analogicas

#define AlarmON              1   // Estado da Variavel de Alarme Ativo
#define AlarmOFF             0   // Estado da Variavel de Alarme Inativo

#define EDTrue               1   // Nivel Ativo da Entrada Digital do Tipo Invertida
#define EDFalse              0   // Nivel Inativo da Entrada Digital do Tipo Invertida

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

// Array com os estados das Saidas Analogicas (SA)
int SA[4];

// Array com os Flags de uso geral
boolean Flag[8];


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

// Declaracao das Variaveis dos Alarmes da Medida de Tensao CC
byte AlarmeOpBat;


// ********** Declaracao da Varivel de Estado 0 - Ventilador Ligado *********
byte EstDig00;           // Estado na Varredura Atual
byte EstAntDig00;        // Estado na Varredura Anterior

// ********** Declaracao da Varivel de Estado 1 *********
byte EstDig01;          // Estado na Varredura Atual
byte EstAntDig01;       // Estado na Varredura Anterior

// ********** Declaracao da Varivel de Estado 2 *********
byte EstDig02;          // Estado na Varredura Atual
byte EstAntDig02;       // Estado na Varredura Anterior

// ********** Declaracao da Varivel de Estado 3 *********
byte EstDig03;          // Estado na Varredura Atual
byte EstAntDig03;       // Estado na Varredura Anterior

// *** Declaracao dos Alarmes ***
boolean Alarm01;
boolean Alarm02;
boolean Alarm03;
boolean Alarm04;
boolean Alarm05;         // Alarme bomba nao Desliga

// Contadores de tempo
boolean HabContTmpBomba;       // Flag de Habilitacao da contagem de tempo da bomba
unsigned int CntTmpBmbLig;     // Contador de tempo que a bomba fica ligada em cada vez
unsigned int CntTmpBmbLigDia;  // Contador de tempo que a bomba fica ligada durante um dia
long Contador3;                // Contador de Segundos no Dia (Relogio de Segundos)
boolean HabLigBmb;;
boolean FlagCont1;
boolean FlagCont2;
boolean FlagCont3;
unsigned int Contador1;
unsigned int Contador2;
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

 // String de mensagens para impressao
 static String MsgImp;

//tmElements_t tm;

 
//********************************************************************************************************
//                                                                                                       *
//                                 Procedimentos de Inicializacao                                        *
//                                                                                                       *
//********************************************************************************************************
//
void setup() {

  // Definir a Funcao dos Pinos de Entrada Digital
  pinMode(ED0, INPUT);
  pinMode(ED1, INPUT);
  pinMode(ED2, INPUT);
  pinMode(ED3, INPUT);
  pinMode(ED4, INPUT);
  pinMode(ED5, INPUT);
  
  // Definir a Funcao dos Pinos de Saida Digital
  pinMode(SD0, OUTPUT);
  pinMode(SD1, OUTPUT);
    
  // Inicia todas as sadas digitais com valor zero\
  digitalWrite(SD0, LOW);
  SDg[0] = LOW;
  digitalWrite(SD1, LOW);
  SDg[1] = LOW;
  
  // Inicia todas as Entradas Analogicas com valor Zero
  for (int i = 0; i < 6; i++) {
    EA[i] = 0;
  }

  // Inicia todas as Entradas Digitais com valor EDFalse
  for (int i = 0; i < 8; i++) {
    ED[i] = EDFalse;
  }

  CntTmpBmbLig = 0;
  CntTmpBmbLigDia = 0;
  
  // Iniciar os Alarmes
  Alarm01 = false;
  Alarm02 = false;
  Alarm03 = false;
  Alarm04 = false;
  Alarm05 = false;

  FlagCont1 = false;
  FlagCont2 = false;
  FlagCont3 = false;
  Contador1 = 0;
  Contador2 = 0;
  Contador3 = 0;

  HabLigBmb = true;        // Inicia com a Bomba Habilitada
  
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
  //Timer1.initialize(BaseTempo);
  //Timer1.attachInterrupt(Temporizacao);

  // Inicia as Variavies de Medidas e de Estados Atuais e Anteriores
  EstMed00 = 0;
  EstMed01 = 0;
  EstMed02 = 0;
  EstMed03 = 0;
  EstMed04 = 0;
  EstMed05 = 0;
   
  AquisicaoControle();
  CarregaMedidas();
   
} // Fim dos Procedimentos de Inicializacao


//******************************************************************************************************
//                                                                                                     *
//                                   Inicio do Programa Principal                                      *
//                                                                                                     *
//******************************************************************************************************
//
void loop() {

  // Atualiza o estado das variaveis de EDs e EAs e das saidas de comando
  AquisicaoControle();
    
  // Carrega as Medidas a partir das Entradas Analogicas (fisicas ou simuladas)
  CarregaMedidas();
 
  // Comunicacao Serial
  ComunicacaoSerial();
  
  // Comunicacao Ethernet
  ComunicacaoEthernet();
  
  // Se a bomba esta desligada verifica:
  // Se a temperatura da placa solar for maior que a temperatura do boiler mais 5 graus e
  // se a temperatura do boiler for menor que 50 graus, liga a bomba.
  if (SDg[1] == LOW) {
    if ((Medida01 > (Medida00 + 700)) && (Medida00 < 5000)) {
        SDg[1] = HIGH;              // Sinaliza a bomba ligada
        digitalWrite(SD1, HIGH);    // Liga a bomba
        Contador1 = 0;              // zera o contador de tempo da bomba ligada,
        FlagCont1 = true;           // habilita a contagem de tempo de bomba ligada.
    }
  }

  // Se a bomba esta ligada verifica:
  // Se a temperatura da placa solar for menor que a temperatura do boiler, 
  // ou se a temperatura do boiler for maior que 52 graus, desliga a bomba.
  if (SDg[1] == HIGH) {
    if ((Medida01 < (Medida00 + 200)) || (Medida00 > 5200)) {
      SDg[1] = LOW;                 // Sinaliza a bomba desligada
      digitalWrite(SD1, LOW);       // Desliga a bomba
      FlagCont1 = false;            // desabilita a contagem de tempo da bomba ligada.
    }
  }
  
  // Se a temperatura do boiler for menor que 30 graus liga o aquecedor eletrico
  if (Medida00 < 3000) {
    SDg[0] = HIGH;
    digitalWrite(SD0, HIGH);
  }
  
  // Se a temperatura do boiler for maior que 35 graus desliga o aquecedor eletrico
  if (Medida00 > 3500) {
    SDg[0] = LOW;
    digitalWrite(SD0, LOW);
  }

  //if (Contador1 > 7200) {         // Se o tempo que a bomba esta ligada foi ultrapassado,
  //  HabLigBmb = false;            // desabilita ligar a bomba e
  //  FlagCont2 = true;             // habilita a contagem de tempo da bomba desligada
  //}

  //if (Contador2 > 600) {          // Se o tempo que a bomba esta desligada foi ultrapassado,
  //  HabLigBmb = true;             // habilita ligar a bomba e
  //  FlagCont2 = false;            // desabilita a contagem de tempo da bomba desligada
  //}

  delay(100);
            
} // Final do Loop Principal do Programa


//*******************************************************************************************************
// Nome da Rotina: Aquisicao                                                                            *
// Funcao: aquisicao de medidas por amostragem periodica usando timer por interrupcao                   *
// Entradas: nenhuma                                                                                    *
// Saidas: nenhuma                                                                                      *
//*******************************************************************************************************
//
void AquisicaoControle() {

  // Carrega os valores dos pontos analogicos nas variaveis de Entradas Analogicas
  EA[0] = analogRead(EA0);
  EA[1] = analogRead(EA1);
  EA[2] = analogRead(EA2);
  EA[3] = analogRead(EA3);
  EA[4] = analogRead(EA4);
  EA[5] = analogRead(EA5);
      
  // Carrega os estados dos pontos digitais nas variaveis de Entradas Digitais
  ED[0] = digitalRead(ED0);
  ED[1] = digitalRead(ED1);
  ED[2] = digitalRead(ED2);
  ED[3] = digitalRead(ED3);
  ED[4] = digitalRead(ED4);
  ED[5] = digitalRead(ED5);
      
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
// Nome da Rotina: ContagemTempo                                                                        *
// Funcao: gerencia os contadores de tempo                                                              *
// Entrada: nenhum                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void ContagemTempo() {
  
  delay(250);
  
} // Fim da Rotina ContagemTempo


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
      Serial.println("");
      Serial.print("Temperatura do Boiler = ");
      Serial.println(Medida00);
      Serial.print("Temperatura da Placa Solar = ");
      Serial.println(Medida01);
      if (SDg[1] == HIGH) {
        Serial.println("Bomba Ligada");
      }
      else {
        Serial.println("Bomba Desligada");
      }
      if (SDg[0] == HIGH) {
        Serial.println("Aquecedor Ligado");
      }
      else {
        Serial.println("Aquecedor Desligado");
      }
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
    HoraRec = RxBuffer[1];
    MinutoRec = RxBuffer[2];
    SegundoRec = RxBuffer[3];
    DiaRec = RxBuffer[4];
    MesRec = RxBuffer[5];
    AnoRec = (RxBuffer[6] + 256*RxBuffer[7]);
    
    MontaMsgConfirma(codigo);
  break;

  case 4:
   
  break;
  
  case 5:  // Comando Pulsado em Saida Digital
           // Rotina ComandoPulsado(Pino, Tempo em multiplos de 10ms)
    MontaMsgConfirma(codigo);
    ComandoPulsado(RxBuffer[1], RxBuffer[2]);
    
  break;
 
  }
  
} // Fim da rotina ExecutaComando


//*******************************************************************************************************
// Nome da Rotina: ComandoPulsado                                                                       *
// Funcao: aciona uma saida digital por um tempo determinado                                            *
// Entrada: numero do pino, tempo em multiplos de 10ms                                                  *
// Saida: nao tem                                                                                       *
//*******************************************************************************************************
//
void ComandoPulsado(byte pino, byte tempo) {
  digitalWrite(pino, HIGH);
  delay(100*tempo);
  digitalWrite(pino, LOW);
}


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
  
  //TxBufEst[5] = tm.Hour;
  //TxBufEst[6] = tm.Minute;
  //TxBufEst[7] = tm.Second;
  //TxBufEst[8] = tm.Day;
  //TxBufEst[9] = tm.Month;
  //TxBufEst[10] = ByteLow(tm.Year + 1970);
  //TxBufEst[11] = ByteHigh(tm.Year + 1970);
  
  TxBufEst[15] = Codigo;                 // O comando foi recebido e executado (codigo do comando)
  
  // Estados de Indicacao Digital (16): bytes 016 a 031 - 1 byte por estado
  TxBufEst[16] = EstDig00;    // Estado Digital 00
  TxBufEst[17] = EstDig01;    // Estado Digital 01
  TxBufEst[18] = EstDig02;    // Estado Digital 02
  TxBufEst[19] = EstDig03;    // Estado Digital 03
  
  // Alarmes (16): bytes 032 a 047 - 1 byte por alarme
  TxBufEst[32] = Alarm01;     // Alarme boiler nao liga
  TxBufEst[33] = Alarm02;     // Alarme boiler nao desliga
  TxBufEst[34] = Alarm03;     // Alarme boiler nao esquenta
  TxBufEst[35] = Alarm04;     // Alarme bomba nao liga
  TxBufEst[36] = Alarm05;     // Alarme bomba nao desliga
    
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
  TxBufEst[68] = 0; // Byte de Estado da Medida 06
  
  // Medida 07
  TxBufEst[69] = ByteLow(Contador2);
  TxBufEst[70] = ByteHigh(Contador2);
  TxBufEst[71] = 0; // Byte de Estado da Medida 07
  
  
   // Estados de Saidas Digitais (4): bytes 072 a 075 - 1 byte por estado
  TxBufEst[72] = SDg[0];
  TxBufEst[73] = SDg[1];
  TxBufEst[74] = SDg[2];
  TxBufEst[75] = SDg[3];
 
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
// Nome da Rotina: MontaMsgConfirma                                                                     *
// Funcao: monta e transmite a mensagem de resposta de confirmacao de comando recebido                  *
// Entrada: codigo do comando que esta mensagem foi usada como resposta                                 *
// Saida: preenche o array TxBufConf e transmite pela ethernet em UDP                                   *
//*******************************************************************************************************
//
void MontaMsgConfirma(byte Codigo) {

  //#define TamMsgConf 20                   // A mensagem de confirmacao tem 20 bytes
  //byte TxBufConf[TamMsgConf];

  // Cabecalho da Mensagem: bytes 000 a 015
  TxBufConf[0] = 1;                       // Endereco da UTR
  TxBufConf[1] = 3;                       // Modelo da UTR (3 = Arduino Mega 2560)
  TxBufConf[2] = 0;
  TxBufConf[3] = 8;                       // Numero de Estados Digitais Configurados (max 64)
  TxBufConf[4] = 4;                       // Numero de Medidas Configuradas (max 32)
  TxBufConf[5] = 4;                       // Numero de Comandos Digitais configurados
  TxBufConf[6] = 0;                       // Numero de Comandos Analogicos configurados
  TxBufConf[7] = ByteLow(NumReg);         // LSB do Numero de Registros de Eventos da EEPROM
  TxBufConf[8] = ByteHigh(NumReg);        // MSB do Numero de Registros de Eventos da EEPROM
                                          // Bytes 9 a 14 sao reserva 
  TxBufConf[15] = Codigo;                 // O comando foi recebido e executado (codigo do comando)

  // Verificacao de erro: bytes 016 a 019
  unsigned int soma = 0;
  for (int i = 0; i < 16; i++) {
    soma += TxBufConf[i];
  }
  TxBufConf[16] = ByteLow(soma);
  TxBufConf[17] = ByteHigh(soma);
  TxBufConf[18] = 127;
  TxBufConf[19] = 255;

  // Transmite a mensagem para o cliente
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(TxBufConf, 20);
  Udp.endPacket();
  
} // Fim da Rotina MontaMsgConfirma


//*******************************************************************************************************
// Nome da Rotina: Temporizacao (chamada por interrupcao de timer)                                      *
// Funcao: incrementa os contadores de tempo (Contador1 a n) se os flags = true (FlagCont1 a n)         *
// Entradas: nenhuma                                                                                    *
// Saidas: nenhuma                                                                                      *
//*******************************************************************************************************
//
void Temporizacao() {

  if (FlagCont1) { Contador1 += 1; }
  if (FlagCont2) { Contador2 += 1; }
  
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
// Nome da Rotina: imprime                                                                              *
// Funcao: envia uma string para uma interface de comunicacao                                           *
// Entradas: string a ser enviada                                                                       *
// Saidas: nenhuma                                                                                      *
//*******************************************************************************************************
//
void imprime(String msg) {
  
  //Serial.println(msg);
  //delay(500);
  
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
