#include <Ethernet.h>
#include <SPI.h>

byte mac[] = {0xDE,0xAD,0xBE,0xEF,0xFE,0xE0};
//byte ip[] = {192,168,0,58};
//byte server[] = {192,168,0,170};
byte server[] = {200,98,140,180};

EthernetClient client;

void setup() {
  
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  IPAddress ip(192,168,0,58);
  IPAddress gateway(192,168,0,1);
  IPAddress dns(192,168,2,1);
  IPAddress subnet(255,255,255,0);
   
  Ethernet.begin(mac, ip, dns, gateway, subnet);
  delay(1000);

  ImpConfig();
  
  

  
  if (client.connect(server,8080) == 1) {
    Serial.println("Conectado ao Servidor");
    client.println("GET /local001.xml");
    client.println();
  } else {
    Serial.println("Falha de Conexão");
  }
}

void loop()
{
  if (client.available()) {
    char c = client.read();
    Serial.print(c);
  }

  if (!client.connected()) {
    Serial.println();
    Serial.println("Desconectando");
    client.stop();
    for(;;)
      ;
  }
}

void ImpConfig() {

  Serial.println("********** Programa Cliente TCP/IP **********");
  Serial.print("Endereço IP Local: ");
  Serial.print(Ethernet.localIP());
  Serial.print("  Endereço IP do Gateway: ");
  Serial.print(Ethernet.gatewayIP());
  Serial.print("  Endereço IP do Servidor DNS: ");
  Serial.println(Ethernet.dnsServerIP());
  Serial.print("Endereço IP do Servidor: ");
  Serial.print(server[0]);
  Serial.print(".");
  Serial.print(server[1]);
  Serial.print(".");
  Serial.print(server[2]);
  Serial.print(".");
  Serial.println(server[3]);
  if(client){
    Serial.println("Cliente TCP/IP OK - Conectando...");;
  }
    
}
