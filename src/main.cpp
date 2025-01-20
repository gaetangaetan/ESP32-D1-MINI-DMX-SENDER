#define VERSION 145
/*
// basic dmx input wireless emitter 
// utiliser pin GPIO16 comme rx du dmx
// version épurée sans bouton
l'adresse normale est 1
le canal dmx 500 donne l'offset
- s'il vaut 0, l'adresse est 1
- s'il vaut 47, par exemple, l'adresse est 48 (1+47)
*/

#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_SIZE 32
#include <esp_now.h>
#include <WiFi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

#include <dmx.h>


const int universe = 1;      // The Art-Net universe you want to receive
const int numChannels = 512; // Total number of channels in the universe


#define RUNNING true
#define SETUP false

#define DMXMODE true
#define ARTNETMODE false

bool etat = RUNNING;

bool mode = DMXMODE;

bool manualMode = true;

int flashInterval;

int DMXaddress = 1;
int offsetDMXaddress;

uint8_t artnetvalues[512]; // tableau dans lequel on gardera en mémoire les valeurs DMX à transmettre
                          // ce tableau est un peu redondant 

typedef struct struct_dmx_packet
{
  uint8_t blockNumber; // on divise les 512 adresses en 4 blocs de 128 adresses
  uint8_t dmxvalues[128];
} struct_dmx_packet;

struct_dmx_packet outgoingDMXPacket;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

esp_now_peer_info_t peerInfo;

uint8_t compteur = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) // cette fonction ne fait rien mais on doit quand même la déclarer
{
  //  Serial.print("\r\nLast Packet Send Status:\t");
  //  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data)
{
  for (int i = 0; i < length; i++)
  {
    artnetvalues[i] = data[i];
  }

 
}



void setup()
{
  pinMode(16, INPUT);

 
  DMXaddress = 1;

  
  Serial.begin(115200);

    mode = DMXMODE;
    Serial.println("mode DMX");

    DMX::Initialize();
    
     //display.setSegments(SEG_DMX);
    delay(2000);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return;
    }
    

  delay(100);

}



void sendDMXvalues()
{
  int offsetDMXaddress = DMXaddress - 1;
  offsetDMXaddress = (int)(DMX::Read(400));
  
  for (int packetNumber = 0; packetNumber < 4; packetNumber++)
  {
    outgoingDMXPacket.blockNumber = packetNumber;
    for (int i = 0; i < 128; i++)
    {
      if (((packetNumber * 128) + i + offsetDMXaddress + 1) > 512)
      {
        outgoingDMXPacket.dmxvalues[i] = 0;
      }
      else
      {
        if (mode == DMXMODE)
        {
          outgoingDMXPacket.dmxvalues[i] = DMX::Read((packetNumber * 128) + i + offsetDMXaddress + 1);
        }
        else
        {
          outgoingDMXPacket.dmxvalues[i] = artnetvalues[(packetNumber * 128) + i + offsetDMXaddress];
        }
      }
    
    }
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingDMXPacket, sizeof(outgoingDMXPacket));
  }

  

    
  
}

void loop()
{

 
 sendDMXvalues();

  delay(10);

}

