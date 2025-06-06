//************************************************************************************************************
//                                                                                                           *
//                     UTR com comunicação UDP em Rede WiFi usando o Controlador ESP8266                     *
//                                                                                                           *
//************************************************************************************************************
//
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Configurações de Comunicação
const char *rede = "CasaVerde";
const char *senha = "luccasofia";
IPAddress local_IP(192,168,0,175);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,0,0);
IPAddress primaryDNS(192,168,2,1);
IPAddress secondaryDNS(192,168,0,1);

// Definicao das constantes de conversao dos valores analogicos
#define KMed00        1.0  // Constante da Medida00
#define OffMed00      0.0  // Offset da Medida00

// Definicao dos numeros dos pinos para leitura das Entradas Analogicas (EA)
#define PinoEA0         0  // EA0 - Sensor de Temperatura do Boiler

// Definição dos pinos para leitura Entrada Digitais
#define PinoED0         0  // Entrada Digital 3 - GPIO0 D3
#define PinoED1         4  // Entrada Digital 2 - GPIO4 D2
#define PinoED2         5  // Entrada Digital 0 - GPIO5 D1
#define PinoED3        15  // Entrada Digital 1 - GPIO15 D8

//int sensor = 5; //GPIO5 D1 Pin number on which sensor is connected
//int buzzer = 15; //GPIO15 D8 Pin number on which buzzer is connected
//int alarm_led_on = 4; //GPIO4 D2 Pin number on which alarm status led is connected
//int alarm_led_off = 0; //GPIO0 D3 Pin number on which alarm status led is connected

//pinMode(sensor, INPUT_PULLUP);
//pinMode(buzzer, OUTPUT);
//pinMode(alarm_led_on, OUTPUT);
//pinMode(alarm_led_off, OUTPUT);

// Definicao dos pinos para escrita nas Saidas Digitais
#define PinoSD0         3  // Saida Digital 0
#define PinoSD1         4  // Saida Digital 1

// Definição dos números dos pinos de controle dos LEDs da placa
#define LED1            2  // Led indicador de comunicação com o servidor em protocolo UDP
#define LED2           14  // Led indicador de conexão à rede WiFi

#define BaseTempo  999673  // 1000000

WiFiUDP Udp;
unsigned int localPort = 5683;
byte TxBufEst[64];
char RxBuffer[255];
unsigned int MsgCoAPId;

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

  // Definir a Funcao dos Pinos de Entrada Digital
  pinMode(PinoED0, INPUT);
  pinMode(PinoED1, INPUT);
  pinMode(PinoED2, INPUT);
  pinMode(PinoED3, INPUT);
  
  // Definir a Funcao dos Pinos de Saida Digital
  //pinMode(PinoSD0, OUTPUT);
  //pinMode(PinoSD1, OUTPUT);
  
  // Inicia todas as sadas digitais com valor zero
  //digitalWrite(PinoSD0, LOW);
  //digitalWrite(PinoSD1, LOW);

  // Inicia as saídas de controle dos LEDs da placa
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);
  
  AtualizaEstadosMedidas();
 
  Serial.begin(115200);

  Serial.println();
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  Serial.printf("Tentando conectar na rede WiFi %s ", rede);
  Serial.println();
  WiFi.begin(rede, senha);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  Serial.printf(" Conectado à rede WiFi %s ", rede);
  Serial.println();
  digitalWrite(LED2, HIGH);

  Udp.begin(localPort);
  Serial.println();
  Serial.printf("Aguardando Mensagens UDP no endereço IP %s, na Porta UDP %d\n", WiFi.localIP().toString().c_str(), localPort);

}


//************************************************************************************************************
//                                                                                                           *
//                                          Programa Principal                                               *
//                                                                                                           *
//************************************************************************************************************
//
void loop() {

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED2, HIGH);
    AtualizaEstadosMedidas();
    ComunicacaoEthernet();
    delay(50);
    digitalWrite(LED1, HIGH);
  }
  else {
    digitalWrite(LED2, LOW);
    delay(50);
  }
}


//*******************************************************************************************************
// Nome da Rotina: AtualizaEstadosMedidas()                                                             *
// Funcao: carrega os estados das entradas digitais e carrega e calcula a medida                        *
// Entrada: nenhum                                                                                      *
// Saida: nenhum                                                                                        *
//*******************************************************************************************************
//
void AtualizaEstadosMedidas() {

  ED0 = digitalRead(PinoED0);
  ED1 = digitalRead(PinoED1);
  ED2 = digitalRead(PinoED2);
  ED3 = digitalRead(PinoED3);
  EA0 = (KMed00 * analogRead(PinoEA0)) + OffMed00;
    
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
  if (packetSize > 0) {                     // Verifica se chegou mensagem UDP pela ethernet
    IPAddress remote = Udp.remoteIP();  // Se chegou mensagem, obter o endereco IP do cliente
    Udp.read(RxBuffer, 255);            // e ler o pacote de dados recebido do cliente

    if ((RxBuffer[0] == 0x40) && (RxBuffer[1] == 0x01)) {
      
      MsgCoAPId = DoisBytesParaInt(RxBuffer[3], RxBuffer[2]);
      Comando = RxBuffer[7];                  // Le o Comando do Payload
      Hora = RxBuffer[8];                     // Le a Hora do Payload
      Minuto = RxBuffer[9];                   // Le o Minuto do Payload
      Segundo = RxBuffer[10];                 // Le o Segundo do Payload
      Dia = RxBuffer[11];                     // Le o Dia do Payload
      Mes = RxBuffer[12];                     // Le o Mes do Payload
      Ano = 256*RxBuffer[13] + RxBuffer[14];  // Le o Ano do Payload
      
      digitalWrite(LED1, LOW);
      ExecutaComando(Comando);         // Executar o comando referente ao codigo recebido
    }
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
      digitalWrite(PinoSD0,LOW);
      digitalWrite(LED1,LOW);
    break;

    case  3:
      digitalWrite(PinoSD0,HIGH);
      digitalWrite(LED1,HIGH);
    break;

    case  4:
      digitalWrite(PinoSD1,LOW);
    break;

    case  5:
      digitalWrite(PinoSD1,HIGH);
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
void MontaMsgEstados(byte Codigo) {

  // Cabecalho da Mensagem CoAP
  TxBufEst[0] = 0x60;
  TxBufEst[1] = 0x45;
  TxBufEst[2] = ByteHigh(MsgCoAPId);
  TxBufEst[3] = ByteLow(MsgCoAPId);
  TxBufEst[4] = 0x0c1;
  TxBufEst[5] = 0x2a;
  TxBufEst[6] = 0x0ff;
  TxBufEst[7] = Codigo;

  // Payload da Mensagem CoAP
  TxBufEst[8] = Hora;            // Relógio - Hora
  TxBufEst[9] = Minuto;          // Relógio - Minuto
  TxBufEst[10] = Segundo;        // Relógio - Segundo
  TxBufEst[11] = Dia;            // Relógio - Dia
  TxBufEst[12] = Mes;            // Relógio - Mês
  TxBufEst[13] = ByteHigh(Ano);  // Relógio - Ano
  TxBufEst[14] = ByteLow(Ano);   // Relógio - Ano
  TxBufEst[15] = 0;              // Reserva
  
  TxBufEst[16] = ED0;
  TxBufEst[17] = ED1;
  TxBufEst[18] = ED2;
  TxBufEst[19] = ED3;

  TxBufEst[20] = 0;
  TxBufEst[21] = 0;
  TxBufEst[22] = 0;
  TxBufEst[23] = 0;

  TxBufEst[24] = ByteLow(EA0);
  TxBufEst[25] = ByteHigh(EA0);

  TxBufEst[26] = 0;
  TxBufEst[27] = 0;
  TxBufEst[28] = 0;
  TxBufEst[29] = 0;

  TxBufEst[30] = 127;
  TxBufEst[31] = 255;

  // Transmite a mensagem para o cliente
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(TxBufEst, 32);
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
