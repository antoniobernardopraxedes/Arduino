//************************************************************************************************************
//                                                                                                           *
//                    UTR com comunicação MQTT em Rede WiFi usando o Controlador ESP8266                     *
//                                                                                                           *
//************************************************************************************************************
//
#include <ESP8266WiFi.h>
//#include <WiFiUdp.h>
#include <PubSubClient.h>
//#include <EspSoftwareSerial>

// Configurações de Comunicação WiFi
const char *rede = "CasaVerde";
const char *senha = "luccasofia";
IPAddress local_IP(192,168,0,175);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,0,0);
IPAddress primaryDNS(192,168,2,1);
IPAddress secondaryDNS(192,168,0,1);

// Configurações de Comunicação MQTT
const char *mqtt_broker = "200.98.140.180";
const char *topic = "CmdLocal";
const char *mqtt_username = "usuario1";
const char *mqtt_password = "senha1";
const int  mqtt_port = 1883;

// Variáveis de Comunicação MQTT
bool mqttStatus = 0;

// Objetos
WiFiClient espClient;
PubSubClient client(espClient);

// Prototipos
bool connectMQTT();
void callback(char *topic, byte *payload, unsigned int length);

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

// Definicao dos pinos para escrita nas Saidas Digitais
#define PinoSD0         3  // Saida Digital 0
#define PinoSD1         4  // Saida Digital 1

// Definição dos números dos pinos de controle dos LEDs da placa
#define LED1            2  // Led indicador de comunicação com o servidor em protocolo UDP
#define LED2           14  // Led indicador de conexão à rede WiFi

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
  
    // Inicia as saídas de controle dos LEDs da placa
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);
  
  AtualizaEstadosMedidas();

  // Inicia as funções de comunicação
  Serial.begin(9600);

  Serial.println();
  // Configura o endereço IP estático
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  Serial.printf("Tentando conectar na rede WiFi %s ", rede);
  Serial.println();
  WiFi.begin(rede, senha);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.printf(" Conectado à rede WiFi %s ", rede);
  Serial.println();
  //digitalWrite(LED2, HIGH);

  mqttStatus = connectMQTT();
   
}


//************************************************************************************************************
//                                                                                                           *
//                                          Programa Principal                                               *
//                                                                                                           *
//************************************************************************************************************
//
void loop() {

  if (mqttStatus) {
    client.loop();
  }
 
}

//*******************************************************************************************************
// Nome da Rotina: connectMQTT()                                                                        *
// Funcao: conecta a UTR8266 ao Broker                                                                  *
// Entrada: nenhum                                                                                      *
// Saida: nenhum                                                                                        *
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

  if (tentativa < 5) {
    //client.publish(topic, "2.5");
    client.subscribe(topic);
    Serial.print("Assinante do Tópico: ");
    Serial.println(topic);
    return 1;
  }
  else {
    Serial.println("Não conectado");
    return 0;
  }
  
}

void callback(char *topic, byte *payload, unsigned int length) {
  digitalWrite(LED1, LOW);
  Serial.print("Recebida mensagem no tópico ");
  Serial.print(topic);
  Serial.print(" : ");
  for (int i = 0; i < length; i++) {
    Serial.print(payload[i]);
    Serial.print(" ");
  }
  Serial.println();
  delay(100);
  digitalWrite(LED1, HIGH);
  
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
