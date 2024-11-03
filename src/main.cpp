#define VERSION 145
/*
//basic dmx input wireless emitter 02112024
// version épurée sans bouton
l'adresse normale est 1
le canal dmx 500 donne l'offset
- s'il vaut 0, l'adresse est 1
- s'il vaut 47, par exemple, l'adresse est 48 (1+47)
attention que la première adresse est celle du mode
  

*/

// commandes manuelles
/*
PINS
34 39 36 (34 vn vp) analog in (pour potards)
25 26 boutons
33 32 display
19 21 22 23 switches
// les switches sont ON vers le bas et OFF au milieu ou vers le haut
switch1 : alimentation
switch2 : DMX/ARTNET OFF - MANUAL ON


switch4 : mode DIMMER, les valeurs RGB sont enregistrées et le premier potard devient le dimmer
Quand switch4 est ON, un clic sur le bouton 1 calibre les potards (enregistre leur valeur minimale)
il faut tourner le premier potard complètement à droite (les boutons ont été montés à l'envers)


slider :
switch3 off : mode programmation, les boutons permettent d'enregistrer les positions
  s4 off : les boutons passent d'une mémoire à l'autre en sauvegardant la mémoire actuelle
  s4 on : les boutons passent d'une position à l'autre
switch3 on : mode déclenchement
  s4 off : le bouton 1 revient à la première position, le bouton 2 déclenche la série de mouvements
  s4 on : les boutons passent d'une position à l'autre
*/

#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_SIZE 32

// #define POT_1 34
// #define POT_2 39
// #define POT_3 36

// int currentSliderPosition = 0;
// int currentSliderPositionX = 0;
// int currentSliderPositionPan = 0;
// int currentSliderPositionTilt = 0;
// int currentSliderPositionSpeed = 0;
// int currentSliderPositionDelay = 0;

// int sliderPositions[10][5];

// bool setupSlider = false;

// int lastValuePot_1 = 0;
// int lastValuePot_2 = 0;
// int lastValuePot_3 = 0;

// int maxValuePot_1 = 255;
// int maxValuePot_2 = 255;
// int maxValuePot_3 = 255;

// bool lastStateSwitch_4 = true;

// int calibration = 0; 
// la valeur minimale des potards n'est plus 0 quand l'appareil fonctionne sur batterie 
//(aucune idée pourquoi, probablement un problème de masse mais j'ai la flemme de rouvrir la boîte)

// int position = 1;

// #define SWITCH_2 21
// #define SWITCH_3 22
// #define SWITCH_4 23

// #define CHANNEL_MODE_SLIDER 299



// #include "OneButton.h"
// OneButton button1(GPIO_NUM_25, true);
// OneButton button2(GPIO_NUM_26, true);

// #include <TM1637Display.h>
// TM1637Display display(GPIO_NUM_33, GPIO_NUM_32); // CLK, DIO

// const uint8_t SEG_DMX[] = {
//     0, //  
//     SEG_B |SEG_C |SEG_D | SEG_E | SEG_G,                                 // D
//     SEG_E | SEG_G | SEG_C,                         // M
//     SEG_B | SEG_C | SEG_E | SEG_F                          // X

// };

// const uint8_t SEG_ART[] = {
//     SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // A
//     SEG_E | SEG_G,                                 // R
//     SEG_E | SEG_F | SEG_G,                         // T
//     SEG_C | SEG_E | SEG_G                          // N

// };

// const uint8_t SEG_PORT[] = {
//     SEG_A | SEG_B | SEG_E | SEG_F | SEG_G, // P
//     SEG_C |SEG_D |SEG_E | SEG_G,           // O
//     SEG_E | SEG_G,                         // R
//     SEG_E | SEG_F | SEG_G,                 // T

// };



#include <esp_now.h>

#include <WiFi.h>


#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

#include <dmx.h>

// #include <ArtnetWifi.h>
// WiFiUDP UdpSend;
// ArtnetWifi artnet;

// #include <ESPAsyncE131.h>
// ESPAsyncE131 e131(1);

const int universe = 1;      // The Art-Net universe you want to receive
const int numChannels = 512; // Total number of channels in the universe

// const char *ssid = "ESP_ARTNET_NODE";
// const char *password = "youhououhou";

//#define LED 23

#define RUNNING true
#define SETUP false

#define DMXMODE true
#define ARTNETMODE false

bool etat = RUNNING;

bool mode = DMXMODE;

bool manualMode = true;

// bool startMovement = false;
// 0 ESP32 DMX RECEIVER :  3C:61:05:13:FD:D0
// 1 DMX RECEIVER : 3C:61:05:D1:CC:57 / COM17 serial …BIA
// 2 mrLEDTUBE : 3C:61:05:D3:19:32  COM18 serial 7

int flashInterval;

int DMXaddress = 1;
int offsetDMXaddress;

uint8_t artnetvalues[512];

typedef struct struct_dmx_packet
{
  uint8_t blockNumber; // on divise les 512 adresses en 4 blocs de 128 adresses
  uint8_t dmxvalues[128];
} struct_dmx_packet;

// struct_message incomingMessage;
// struct_message outgoingMessage;

// struct_dmx_message outgoingDMXMessage;
struct_dmx_packet outgoingDMXPacket;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t mrLEDTUBE1Address[] = {0x3C, 0x61, 0x05, 0xD3, 0x19, 0x32};

esp_now_peer_info_t peerInfo;

uint8_t compteur = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  //  Serial.print("\r\nLast Packet Send Status:\t");
  //  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// void setCalibration(int meanlength)
// {
//    double x = 0;
//   for(int i=0;i<meanlength;i++)
//   {
//     x=x+analogRead(POT_1);    
//   }
//   calibration = (int)(x/meanlength);  
// }

// ----- buttons callback functions

// void click1()
// {  
//       display.showNumberDec(1000);
//     delay(1000);
    
//   // Serial.print("click 1");

 

//   // if(digitalRead(SWITCH_4))
//   // {
//   //   setCalibration(20);
//   //   for(int i=0;i<10;i++)
//   //   {
//   //     display.showNumberDec(calibration);
//   //     delay(100);
//   //     display.clear();
//   //     delay(100);
//   //   }
    
//   // }

//   if (etat == SETUP)
//   {
//     DMXaddress--;
//     if (DMXaddress < 1)
//       DMXaddress = 512;
//   }
// }

// void click2()
// {
//   display.showNumberDec(0001,true);
//     delay(1000);
//   // Serial.print("click 2 =  ");

  

//   if (etat == SETUP)
//   {
//     DMXaddress++;
//     if (DMXaddress > 512)
//       DMXaddress = 1;
//   }
// }

// void doubleClick1()
// {

//  display.showNumberDec(1111);
//     delay(1000);
  
  
//   if (etat == SETUP) // avant de sortir du SETUP, on enregistre les données en mémoire persistante
//   {
//     EEPROM.writeInt(0, DMXaddress);
//     Serial.print("Commit =  ");
//     Serial.println(EEPROM.commit());
//   }

//   etat = !etat;
// } // on passe de RUNNING à SETUP

// void longPress1()
// {
//   click1();
// }

// void doubleClick2()
// {
  
//  display.showNumberDec(1111);
//     delay(1000);
//   doubleClick1();
// }

// void longPress2()
// {
//   click2();
// }

void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data)
{
  for (int i = 0; i < length; i++)
  {
    artnetvalues[i] = data[i];
  }

  // if(digitalRead(SWITCH_3)) // en mode contrôle du slider, on réserve les canaux 300 à 355 pour les mémoires de position
  // {
  //   artnetvalues[299]=1;
  //   artnetvalues[300]=currentSliderPositionX;
  //   artnetvalues[301]=currentSliderPositionPan;
  //   artnetvalues[302]=currentSliderPositionTilt;
  //   artnetvalues[303]=currentSliderPositionSpeed;
  //   artnetvalues[304]=currentSliderPositionDelay;
  //   for(int i=0;i<10;i++)
  //   {
  //        for(int j=0;j<5;j++)artnetvalues[305+(5*i)+j] = sliderPositions[i][j];
  
  //   }
  // }
}



// int meanAnalogRead(int pin, int meanlength, int maxvalue, int delayms)
// {
//   double x = 0;
  
//   for(int i=0;i<meanlength;i++)
//   {    
//     x=x+ ((((double)(analogRead(pin))-(double)(calibration)))/((double)(4095)-(double)(calibration)))*(double)(maxvalue);
//     if(delayms!=0)delay(delayms);
//   }
//   x = x/meanlength;
//   if(x<0)
//   {
//     return 0;
//   }
//   else if (x>maxvalue)
//   {
//     return maxvalue;
//   }
//   else return x;

// }

void setup()
{
  pinMode(16, INPUT);

  // on utilise les pins 21 22 et 23 comme GND pour les boutons
  // pinMode(GPIO_NUM_21, OUTPUT);
  // pinMode(GPIO_NUM_22, OUTPUT);
  // pinMode(GPIO_NUM_23, OUTPUT);
  // digitalWrite(GPIO_NUM_21, LOW);
  // digitalWrite(GPIO_NUM_22, LOW);
  // digitalWrite(GPIO_NUM_23, LOW);

  // EEPROM.begin(EEPROM_SIZE);  
  // DMXaddress = EEPROM.readInt(0);
  // if ((DMXaddress < 1) || (DMXaddress > 512))
  //   DMXaddress = 1;
  DMXaddress = 1;

  // display.setBrightness(7);

  // link the buttons functions.
  // button1.attachClick(click1);
  // button1.attachDoubleClick(doubleClick1);
  // button1.attachDuringLongPress(longPress1);

  // button2.attachClick(click2);
  // button2.attachDoubleClick(doubleClick2);
  // button2.attachDuringLongPress(longPress2);

  //pinMode(LED, OUTPUT);
  Serial.begin(115200);
  if (true) // mode DMX, le bouton n'est pas enfoncé au démarrage
   //if (digitalRead(GPIO_NUM_26) == HIGH) // mode DMX, le bouton n'est pas enfoncé au démarrage
  //if (false) // mode ArtNet systématique (pour les tests)
  {
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
    
  }
//   else // mode artnet, le bouton 1 est enfoncé au démarrage
//   {
//     mode = ARTNETMODE;
//     Serial.println("mode artnet");
//     display.setSegments(SEG_ART);
//     delay(2000);
//     if (digitalRead(GPIO_NUM_26) == LOW) // portal to choose wifi network
//     {
//       display.setSegments(SEG_PORT);
//         WiFiManager wm;

//     // reset settings - wipe stored credentials for testing
//     // these are stored by the esp library
//     // wm.resetSettings();

//     // Automatically connect using saved credentials,
//     // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
//     // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
//     // then goes into a blocking loop awaiting configuration and will return success result

//     bool res;
//     // res = wm.autoConnect(); // auto generated AP name from chipid
//     // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
//     res = wm.startConfigPortal();
//     }
//     else // create own AP
//     {
//        // Set up the ESP32 as an access point
//        WiFi.mode(WIFI_AP_STA);
//     WiFi.softAP("ESP_ARTNET_NODE", "youhououhou");
    

//     // Print the IP address of the access point
//     Serial.print("Access Point IP address: ");
//     Serial.println(WiFi.softAPIP());
//     Serial.println("wifi.softap");
//     delay(5000);
//     }

   
//     artnet.setArtDmxCallback(onDmxFrame);
//     Serial.println("dmxcallback");
//     artnet.begin();
//     Serial.println("artnet.begin");
//     // Set device as a Wi-Fi Station
//       //WiFi.disconnect(); 
      
      
//     //ESP.eraseConfig();


    
// //     WiFi.printDiag(Serial);
// //     esp_wifi_set_promiscuous(true);
// // esp_wifi_set_channel(2, WIFI_SECOND_CHAN_NONE);
// // esp_wifi_set_promiscuous(false);
// // WiFi.printDiag(Serial); // Uncomment to verify channel change after

//     // Init ESP-NOW
    
//     if (esp_now_init() != ESP_OK)
//     {
//       Serial.println("Error initializing ESP-NOW");
//       return;
//     }

//     // Once ESPNow is successfully Init, we will register for Send CB to
//     // get the status of Trasnmitted packet
//     esp_now_register_send_cb(OnDataSent);

//     // Register peer
//     memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//     peerInfo.channel = 0;
//     peerInfo.encrypt = false;

//     // Add peer
//     if (esp_now_add_peer(&peerInfo) != ESP_OK)
//     {
//       Serial.println("Failed to add peer");
//       return;
//     }
//   }

  delay(100);
 //display.showNumberDec(100*digitalRead(SWITCH_2)+10*digitalRead(SWITCH_3)+digitalRead(SWITCH_4));
   // delay(5000);
}



void sendDMXvalues()
{
  //int offsetDMXaddress = DMXaddress - 1;
  // offsetDMXaddress = (int)(DMX::Read(400));
  offsetDMXaddress = 0;
  outgoingDMXPacket.blockNumber=0;
  outgoingDMXPacket.dmxvalues[0]=0;
  outgoingDMXPacket.dmxvalues[1]=200;
  outgoingDMXPacket.dmxvalues[2]=0;
  outgoingDMXPacket.dmxvalues[3]=30;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingDMXPacket, sizeof(outgoingDMXPacket));
  
 
  
  // for (int packetNumber = 0; packetNumber < 4; packetNumber++)
  // {
  //   outgoingDMXPacket.blockNumber = packetNumber;
  //   for (int i = 0; i < 128; i++)
  //   {
  //     if (((packetNumber * 128) + i + offsetDMXaddress + 1) > 512)
  //     {
  //       outgoingDMXPacket.dmxvalues[i] = 0;
  //     }
  //     else
  //     {
  //       if (mode == DMXMODE)
  //       {
  //         //outgoingDMXPacket.dmxvalues[i] = DMX::Read((packetNumber * 128) + i + offsetDMXaddress + 1);

  //           outgoingDMXPacket.dmxvalues[2] == compteur;
  //           compteur = (compteur + 1) % 255;
          
  //       }
  //       else
  //       {
  //         outgoingDMXPacket.dmxvalues[i] = artnetvalues[(packetNumber * 128) + i + offsetDMXaddress];
  //       }
  //     }

    
  //   }

  //   // if (digitalRead(SWITCH_2)) // MODE MANUEL (ne modifie que le premier bloc de 128, avec le mode 0 (ALL RGB) et les valeurs rgb des potards)
  //   // {
     
  //   //     if(packetNumber==0)
  //   //     {
  //   //       outgoingDMXPacket.blockNumber=0;
  //   //       outgoingDMXPacket.dmxvalues[0] = 0; // ALL RGB
  //   //     if (digitalRead(SWITCH_4))
  //   //     {
  //   //       int dimmer = 255 - meanAnalogRead(POT_1, 20, 255, 0);
  //   //       outgoingDMXPacket.dmxvalues[1] = (lastValuePot_1 * dimmer) / 255;
  //   //       outgoingDMXPacket.dmxvalues[2] = (lastValuePot_2 * dimmer) / 255;
  //   //       outgoingDMXPacket.dmxvalues[3] = (lastValuePot_3 * dimmer) / 255;
  //   //     }
  //   //     else
  //   //     {
  //   //       lastValuePot_1 = 255 - meanAnalogRead(POT_1, 20, 255, 0);
  //   //       lastValuePot_2 = 255 - meanAnalogRead(POT_2, 20, 255, 0);
  //   //       lastValuePot_3 = 255 - meanAnalogRead(POT_3, 20, 255, 0);
  //   //       // outgoingDMXPacket.dmxvalues[1] = 255 - meanAnalogRead(POT_1, 20, 255, 0);
  //   //       // outgoingDMXPacket.dmxvalues[2] = 255 - meanAnalogRead(POT_2, 20, 255, 0);
  //   //       // outgoingDMXPacket.dmxvalues[3] = 255 - meanAnalogRead(POT_3, 20, 255, 0);
  //   //       outgoingDMXPacket.dmxvalues[1] = lastValuePot_1;
  //   //       outgoingDMXPacket.dmxvalues[2] = lastValuePot_2;
  //   //       outgoingDMXPacket.dmxvalues[3] = lastValuePot_3;
  //   //     }

  //   //     }
        
      
  //   // }
    
  //   // if (mode == DMXMODE)
  //   //     {
  //   //       position = DMX::Read(CHANNEL_MODE_SLIDER);
  //   //     }
  //   //     else
  //   //     {
  //   //       position = artnetvalues[CHANNEL_MODE_SLIDER - 1];
  //   //     }


  //   // for(int i=1;i<32;i++)
  //   // {
  //   //   Serial.print(DMX::Read(i));
  //   //   Serial.print(" ");
  //   // }
  //   // Serial.println();

  //   // for(int i=0;i<32;i++)
  //   // {
  //   //   Serial.print(outgoingDMXPacket.dmxvalues[i]);
  //   //   Serial.print(" ");
  //   // }
  //   // Serial.println();


  //   esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingDMXPacket, sizeof(outgoingDMXPacket));

    
  // }
}

void loop()
{

  //
  // Serial.print(digitalRead(SWITCH_2));
  // Serial.print(" switch3 = ");
  // Serial.print(digitalRead(SWITCH_3));
  // Serial.print(" switch4 = ");
  // Serial.println(digitalRead(SWITCH_4));
  
  // pour les tests, on désactive les boutons
   //button1.tick();
   //
   //button2.tick();
   //display.showNumberDec(DMX::Read(1));

 if(digitalRead(16))
 {
  Serial.println("Tilt!");
 }  
 else
 {
  Serial.println("pas tilt :)");
 }
 sendDMXvalues();
 delay(10);
//display.showNumberDec(position);

  // if (etat == RUNNING)
  // {

  //   if (mode == ARTNETMODE)
  //   {
  //     artnet.read();
      
  //   }

  //   sendDMXvalues();

  //   display.showNumberDec(position);

    
    
  // }
  // else  // afficher adresse
  // {
  //   display.showNumberDec(DMXaddress);
  //   delay(5);
  //   display.clear();
  //   delay(20);

  //   // faire clignotter adresse
  // }
}

// setup et loop temporaires pour les tests


