https://www.youtube.com/watch?v=20lAyKzfSEU
//*****************************************************************************************************
//                                                                                                    *
//                 Programa do Controlador da Energia do Starlink Arduino UNO                         *
//                                                                                                    *
// Autor: Antonio Bernardo de Vasconcellos Praxedes                                                   *
//                                                                                                    *
// Data: 01/02/2024                                                                                   *
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
#define KMed00      29.966  // Constante da tensão CA
#define OffMed00    0.0     // Offset da Medida00
#define KMed01      1.527   // Constante
#define OffMed01    0.0     // Offset
#define KMed02      0.0     // Constante
#define OffMed02    0.0     // Offset
#define KMed03      0.0     // Constante
#define OffMed03    0.0     // Offset
#define KMed04      0.0     // Constante
#define OffMed04    0.0     // Offset
#define KMed05      0.0     // Constante
#define OffMed05    0.0     // Offset

// Definicao dos numeros dos pinos para leitura das Entradas Analogicas (EA)
#define EA0                  0   // EA0 - Sensor de Tensão da Rede
#define EA1                  1   // EA1
#define EA2                  2   // EA2 
#define EA3                  3   // EA3
#define EA4                  4   // EA4
#define EA5                  5   // EA5

// Definicao dos numeros dos pinos para escrita nas Saidas Digitais (SD) - Ativas em Nivel Um
#define SD0                  2  // Saida Digital 0: Controle do Rele de Ativação do Inversor
#define SD1                  3  // Saida Digital 1: Controle do Rele de Desativação do Relé 220VAC
#define Led                 13  // Saída Digital LED

#define BaseTempo       999673   // 1000000


//******************************************************************************************************
//                                                                                                     *
//                                    Declaracao das Variaveis                                         *
//                                                                                                     *
//******************************************************************************************************
//
// Array com os estados das Entradas Analogicas
unsigned int EA[6];

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

boolean FlagCont1;
boolean FlagCont2;
unsigned int Contador1;
unsigned int Contador2;

boolean FaltaCA = true;
boolean BatOK = false;
unsigned int ContRetornoCA;

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
  pinMode(SD1, OUTPUT);
  pinMode(Led, OUTPUT);
  
  // Inicia todas as sadas digitais com valor zero
  digitalWrite(SD0, LOW);
  digitalWrite(SD1, LOW);
  digitalWrite(Led, LOW);
  
  FlagCont1 = false;
  FlagCont2 = false;
  Contador1 = 0;
  Contador2 = 0;

  CarregaMedidas();

  if (Medida00 > 18000) {
    FaltaCA = false;
  }

  if (Medida00 < 14000) {
    FaltaCA = true;
  }

  if (Medida01 > 1250) {
    BatOK = true;
  }

  if (Medida01 < 1100) {
    BatOK = false;
  }
  
  ContRetornoCA = 0;
  
  // Inicia a Interface Serial Assincrona
  Serial.begin(9600);

  // Inicia o timer e a chamada da rotina de interrupcao
  Timer1.initialize(BaseTempo);
  Timer1.attachInterrupt(Temporizacao);

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

  CarregaMedidas();

  if (FaltaCA) {                  // Se ocorreu Falta de CA,
    if (Medida00 > 18000) {       // e se a tensão da rede é maior que 180,00 VCA,
      ContRetornoCA++;            // incrementa o contador de espera
      if (ContRetornoCA >= 40) {  // Se o contador de espera chegou a 40s com a tensão CA maior que 180,00 VCA,
        FaltaCA = false;          // reseta o flag indicador de falta de CA,
        ContRetornoCA = 0;        // zera o contador de espera
      }
    }
    else {                        // Se a tensão CA cair abaixo de 180,00 VCA,
      ContRetornoCA = 0;          // zera o contador de espera
    }
  }
  else {                          // Se a tensão CA estava normal,
    if (Medida00 < 14000) {       // e se a tensão da rede cair abaixo de 140,00 VCA,
      FaltaCA = true;             // sinaliza na flag,
      ContRetornoCA = 0;          // e zera o contador de espera
    }
  }

  if (Medida01 > 1250) {
    BatOK = true;
  }

  if (Medida01 < 1100) {
    BatOK = false;
  }

  Serial.print("Tensão Rede CA = ");
  Serial.print(Medida00);
  Serial.print(" / Tensão CC = ");
  Serial.print(Medida01);
  Serial.print(" - Contador = ");
  Serial.println(ContRetornoCA);
  //Serial.print("Flag Falta CA:  = ");
  //Serial.print(FaltaCA);
  //Serial.print(" - Flag Bateria:  = ");
  //Serial.println(BatOK);
  

  if (FaltaCA) {                // Se ocorreu falta de CA,
    if (Medida01 >= 1250) {     // e se a tensão das baterias for maior ou igual a 12,5 Volts,
      digitalWrite(SD1, HIGH);  // passa o relé principal para alimentação pelo inversor,
      delay(3000);              // espera 3 segundos,
      digitalWrite(SD0, HIGH);  // e liga o inversor.
    }
  }
  else {                        // Se a tensão CA está normal,
    digitalWrite(SD0, LOW);     // desliga o inversor,
    delay(3000);                // espera 3 segundos,
    digitalWrite(SD1, LOW);     // e passa o relé principal para alimentação pela rede.
  }
  
  if (digitalRead(SD0) == HIGH) {  // Se o inversor estiver ligado,
    if (!BatOK) {                  // e se a tensão das baterias estiver baixa,
      digitalWrite(SD0, LOW);      // desliga o inversor
      delay(3000);                 // espera 3 segundos
      digitalWrite(SD1, LOW);      // e passa o relé principal para alimentação pela rede
    }
  }

  if (FaltaCA) {
    digitalWrite(Led, HIGH);
    delay(200);
    digitalWrite(Led, LOW);
    delay(800);
  }
  else {
    digitalWrite(Led, HIGH);
  }
            
} // Final do Loop Principal do Programa

//*******************************************************************************************************
// Nome da Rotina: CarregaMedidas                                                                       *
// Funcao: carrega as medidas a partir das entradas analogicas (fisicas ou simuladas)                   *
// Entrada: nenhum                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void CarregaMedidas() {

  Medida00 = (KMed00*analogRead(EA0)) + OffMed00;
  Medida01 = (KMed01*analogRead(EA1)) + OffMed01;
  Medida02 = (KMed02*analogRead(EA2)) + OffMed02;
  Medida03 = (KMed03*analogRead(EA3)) + OffMed03;
  Medida04 = (KMed04*analogRead(EA4)) + OffMed04;
  Medida05 = (KMed05*analogRead(EA5)) + OffMed05;
  
} // Fim da Rotina CarregaMedidas


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
