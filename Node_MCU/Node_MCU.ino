#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

#define SendKey 0  //Button to send data Flash BTN on NodeMCU

int tcp_port = 61054;
IPAddress TCP_SERVER_IP(192, 168, 1, 69);
char TCP_SERVER_NAME[] = "LAPTOP-61RQS678";
WiFiClient tcp_client;

int udp_port = 61050;  //Port number
int udp2_port = 61040;
IPAddress local_IP(192, 168, 1, 222); // Set your static IP address
IPAddress gateway(192, 168, 1, 1); // Set your Gateway IP address
IPAddress subnet(255, 255, 255, 0); // Set your Gateway subnet
WiFiServer udp_server(udp_port); // Set Server port
WiFiServer udp2_server(udp2_port);

//Server connect to WiFi Network
String newHostname = "RCAGV01";
//const char *ssid = "CR_MW_2.4G";  //Enter your wifi SSID
//const char *password = "@3A-KT4-12FK7P-xhf!";  //Enter your wifi Password
const char *ssid = "HW_M_T_U";  //Enter your wifi SSID
const char *password = "AcFcJHGX@12310";  //Enter your wifi Password

bool disconnect_state = true;

WiFiUDP Udp;
WiFiUDP Udp2;

int count=0;
//=======================================================================
//                    Power on setup
//=======================================================================
void setup() 
{
  Serial.begin(115200);
  pinMode(SendKey,INPUT_PULLUP);  //Btn to send data
  Serial.println();

  start_wifi();
  wifi_status();
  enable_rc_comms();

  start_udp_listener();
  start_udp2_listener();
}
void enable_rc_comms()
{
  Serial.write('E'); // enable arduino to receive comms
  Serial.write('C');
}
void start_wifi()
{
  WiFi.hostname(newHostname.c_str());
  WiFi.mode(WIFI_STA);
  // Serial.println("New hostname: %s\n", WiFi.hostname().c_str());
  WiFi.begin(ssid, password); //Connect to wifi
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, gateway)) {
    Serial.println("STA Failed to configure");
  }
  // Wait for connection  
  while (WiFi.status() != WL_CONNECTED) {   
    delay(500);
    Serial.print(".");
    delay(500);
  }
  
}
void wifi_status()
{ 
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());
}
void start_udp_listener()
{
  Udp.begin(udp_port);
}
void start_udp2_listener()
{
  Udp2.begin(udp2_port); 
}
//=======================================================================
//                    Loop
//=======================================================================
void notify_arduino()
{
  if (disconnect_state == true)
      {
        Serial.write('+'); // new connection established
        disconnect_state = false;        
      }
}
void loop() 
{ 
  Serial.println(tcp_client.connect(TCP_SERVER_IP, tcp_port)); // Debug line
  
  if (tcp_client.connect(TCP_SERVER_IP, tcp_port)) {
      
    if(tcp_client.connected())
    {
      //Serial.println("TCP Client Connected");
      notify_arduino(); // if arduino does not know notify of connection  
    }
    if(tcp_client.connected() == false)
    {
      //Serial.println("TCP Client Disconnected");
      if (disconnect_state == false) // notify arduino of disconnection
      {
        Serial.write('-'); // new connection established
        disconnect_state = true;        
      }
    }
    while(tcp_client.connected()){      
      notify_arduino(); // if arduino does not know notify of connection 
      // receive data from tcp_client
      while(tcp_client.available()>0){
        // read data from the connected tcp_client
        Serial.write(tcp_client.read()); 
      }
      // Send Data to connected tcp_client
      udp_to_robot();
      udp_to_pc();
      udp2_to_robot();
    }   
  }
  else if (disconnect_state == false) // notify arduino of disconnection
  {
    Serial.write('-'); // new connection established
    disconnect_state = true;        
  }
}
void udp_to_robot()
{
  // working
  int size = Udp.parsePacket();
  if ( size ) {
    byte packetBuffer[size];
    Udp.readBytes(packetBuffer, size); // read the packet into the buffer
    Serial.write(packetBuffer, size); // send packet to arduino
  }
}
void udp2_to_robot()
{
  // working
  int size = Udp2.parsePacket();
  if ( size ) {
    byte packetBuffer[size];
    Udp2.readBytes(packetBuffer, size); // read the packet into the buffer
    Serial.write(packetBuffer, size); // send packet to arduino
  }
}
void udp_to_pc()
{
  if (Serial.available() >= 1)
  {    
    delay(3.3);
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    while (Serial.available() > 0){
      Udp.write(Serial.read());
    }
    Udp.endPacket();    
  }
}
//=======================================================================