/*

SALAMAKER
Controlando um LED e lendo um LDR por uma página web com D1 - Wemos - ESP8266 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define LED 14

/* Configuração de rede e senha */
const char *rede = "CasaVerde";
const char *senha = "luccasofia";
int estado_led = 0; //desligado

IPAddress local_IP(192, 168, 0, 175); // Set your Static IP address
IPAddress gateway(192, 168, 0, 1);    // Set your Gateway IP address
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(192, 168, 2, 1);   //optional
IPAddress secondaryDNS(192, 168, 0, 1); //optional

ESP8266WebServer server(80); //Objeto "servidor" na porta 80(porta HTTP)


void setup() {

  Serial.begin(115200);

  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(rede);
  
  WiFi.begin(rede, senha);

  while (WiFi.status() != WL_CONNECTED) //Aguarda a conexao
  {
    Serial.print("Tentando conectar com ");
    Serial.println(WiFi.SSID()); //Imprime o nome da rede
    delay(500);
  }
  Serial.print("Conectado a rede! Endereco IP ESP -> ");
  Serial.println(WiFi.localIP()); //Imprime o IP local do ESP

  /* Cria a associacao entre endereço HTML as funções que serão utilizadas */
  server.on("/", paginaInicial);
 
  server.begin(); //Inicia o servidor
  pinMode(LED, OUTPUT); //Configura LED Embutido como saída
  digitalWrite(LED, LOW); //Inicia apagado -
  
}

void loop() {
  server.handleClient();  //Analise das solicitacoes via web
}

void paginaInicial() {

  String valor_sensor = "";
  valor_sensor = String(analogRead(A0));

  //============================
   String htmlMessage = "<!DOCTYPE html>"
                             "<html>"
                             "<head>"
                             "<meta http-equiv=\"refresh\" content=\"1\">" //Atualizar a pagina a cada 1s
                             "<title>Supervisão e Controle WiFi - ESP8266</title>"
                             "</head>"
                             "<body><font face=\"verdana\">"
                             "<h2>Controlar um LED pela GPIO 14 do ESP</h2>";
                             

                              if(estado_led == 0)
                              {
                                htmlMessage +=
                                "<a href=\"/?ledstatus=0\" > Acende LED</a>";
                                
                                if(server.arg("ledstatus") == "0")
                                {
                                  digitalWrite(LED,HIGH);
                                  estado_led=1;
                                }
                              }
                              else{
                                htmlMessage +=
                             " <a href=\"/?ledstatus=1\" >  Apaga LED</a>";
                             
                             
                                if(server.arg("ledstatus") == "1")
                                {
                                  digitalWrite(LED,LOW);
                                   estado_led=0;
                                }
                               
                              }

                               htmlMessage +=
                              "<h2>Dados do LDR:</h2>";

                              htmlMessage += valor_sensor;

                              htmlMessage += 
                              "<hr>"
                             "</html>";
                             

  //============================
  server.send(200, "text/html", htmlMessage); //Retorna resposta HTTP
}
