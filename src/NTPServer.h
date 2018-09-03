#include <Arduino.h>
//----------------------------------------------------------------------librerias de TIEMPO NTP
#include <TimeLibEsp.h>                                                  //TimeTracking
#include <WiFiUdp.h>                                                  //UDP packet handling for NTP request
//----------------------------------------------------------------------Librerias de manejo de setup de redes 
//----------------------------------------------------------------------Inicio de cliente UDP
WiFiUDP udp;                                                            //Cliente UDP para WIFI
//----------------------------------------------------------------------Codigo para estblecer el protocolo de tiempo en red NTP
const int NTP_PACKET_SIZE = 48;                                         //NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE];                                     //Buffer to hold incoming & outgoing packets
boolean NTP = false;                                                    //Bandera que establece el estado inicial del valor de NTP

//----------------------------------------------------------------------send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // 
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

//----------------------------------------------------------------------Funcion para obtener el paquee de TP y procesasr la fecha hora desde el servidor de NTP
time_t getNtpTime(){
  while (udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println(F("Transmit NTP Request"));
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println(F("Receive NTP Response"));
      NTP = true;
      udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println(F("No NTP Response :-("));
  return 0; // return 0 if unable to get the time
}

