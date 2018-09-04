// Librerias de ESP // MQTT/ JSON FORMAT data
#include <ESP8266WiFi.h>                                              //Libreira de ESPCORE ARDUINO
#include <PubSubClient.h>                                             //https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <Arduino.h>
#include "settings.h"

char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;               //Variable de Identificacion de Cliente para servicio de MQTT Bluemix 

//----------------------------------------------------------------------Variables Propias del CORE ESP8266 Para la administracion del Modulo
String NodeID = String(ESP.getChipId());                                //Variable Global que contiene la identidad del nodo (ChipID) o numero unico

//----------------------------------------------------------------------Funcion remota para administrar las actulizaciones remotas de las variables configurables desde IBMbluemix
void handleUpdate(byte* payload)
{
    //La Funcion recibe lo que obtenga Payload de la Funcion Callback que vigila el Topico de subcripcion (Subscribe TOPIC)
    StaticJsonBuffer<300> jsonBuffer;                                   //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
    JsonObject& root = jsonBuffer.parseObject((char*)payload);          //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
    if (!root.success())
    {
        //Si no se encuentra el objeto Raiz del Json
        Serial.println(F("ERROR en la Letura del JSON Entrante"));      //Se imprime un mensaje de Error en la lectura del JSON
        return;                                                         //Nos salimos de la funcion
    }                                                                   //se cierra el condicional
    //Seimprime el mensaje que se recibio
    Serial.println(F("handleUpdate payload:"));                         //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
    root.prettyPrintTo(Serial);                                         //y se imprime el mensaje recibido al Serial  
    Serial.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
    //Buscamos los datos enviados al sensor
    const char* sensor = root["sensor"];                                // buscamos el valor del objeto dentro del Json
    unsigned long new_data_Interval = root["interval"][0];              // buscamos en en un arreglo de objetos en la posicion 0
    unsigned long new_update_interval = root["interval"][1];            // bucamos en un arreglo de ojetos la posicion 1
    Serial.println("los datos recibidos son:");
    Serial.print(F("Tipo de sensor: "));
    Serial.println(sensor);                                             //imprimimos las valores recibidos en variables
    Serial.print(F("intervalo de datos :"));                               //imprimimos las valores recibidos en variables
    Serial.println(new_data_Interval);                                  //imprimimos las valores recibidos en variables
    Serial.println(F("intervalo de calibracion: "));                       //imprimimos las valores recibidos en variables
    Serial.println(new_update_interval );                               //imprimimos las valores recibidos en variables
}
//----------------------------------------------------------------------Funcion remota para mandar a dormir el esp despues de enviar un RFID
void handleResponse (byte* payloadrsp) {
  StaticJsonBuffer<200> jsonBuffer;                                   //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
  JsonObject& root = jsonBuffer.parseObject((char*)payloadrsp);       //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
  if (!root.success()) {                                              //Si no se encuentra el objeto Raiz del Json
    Serial.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
    return;                                                           //Nos salimos de la funcion
  }                                                                   //se cierra el condicional
  
  Serial.println(F("handleResponse payload:"));                       //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
  root.printTo(Serial);                                               //y se imprime el mensaje recibido al Serial  
  Serial.println(F(""));                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
}
//----------------------------------------------------------------------Funcion de vigilancia sobre mensajeria remota desde el servicion de IBM bluemix
void callback(char* topic, byte* payload, unsigned int payloadLength){//Esta Funcion vigila los mensajes que se reciben por medio de los Topicos de respuesta;
  Serial.print(F("callback invoked for topic: "));                    //Imprimir un mensaje seÃ±alando sobre que topico se recibio un mensaje
  Serial.println(topic);                                              //Imprimir el Topico
  
  if (strcmp (responseTopic, topic) == 0) {                            //verificar si el topico conicide con el Topico responseTopic[] definido en el archivo settings.h local
    handleResponse(payload);
    //return; // just print of response for now                         //Hacer algo si conicide (o en este caso hacer nada)
  }
  
  if (strcmp (rebootTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    Serial.println(F("Rebooting..."));                                //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
    ESP.reset();                                                    //Emitir comando de reinicio para ESP8266
  }
  
  if (strcmp (updateTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico updateTopic[] definido en el archivo settings.h local
    handleUpdate(payload);                                            //enviar a la funcion handleUpdate el contenido del mensaje para su parseo.
  } 
}
//----------------------------------------------------------------------definicion de Cliente WIFI para ESP8266 y cliente de publicacion y subcripcion
WiFiClient wifiClient;                                                //Se establece el Cliente Wifi
PubSubClient client(MQTTServer, 1883, callback, wifiClient);              //se establece el Cliente para el servicio MQTT
//----------------------------------------------------------------------Funcion de Conexion a Servicio de MQTT
void mqttConnect() {
  if (!!!client.connected()) {                                         //Verificar si el cliente se encunetra conectado al servicio
  Serial.print(F("Reconnecting MQTT client to: "));                    //Si no se encuentra conectado imprimir un mensake de error y de reconexion al servicio
  Serial.println(MQTTServer);                                             //Imprimir la direccion del servidor a donde se esta intentado conectar 
  char charBuf[30];
  String CID (clientId + NodeID); 
  CID.toCharArray(charBuf, 30);  
  #if defined (InternetServer)
    while (!!!client.connect(charBuf, "flatboxadmin", "FBx_admin2012")) {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
    Serial.print(F("."));                                             //imprimir una serie de puntos mientras se da la conexion al servicio
    }  
  #else
    while (!!!client.connect(charBuf)) {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
    Serial.print(F("."));                                             //imprimir una serie de puntos mientras se da la conexion al servicio
    }  
  #endif  
  Serial.println();                                                   //dejar un espacio en la terminal para diferenciar los mensajes.
 }
}

//----------------------------------------------------------------------Funcion de REConexion a Servicio de MQTT
void MQTTreconnect() 
{
  int retry = 0;
  // Loop until we're reconnected
  while (!client.connected()) 
  {    
    Serial.print(F("Attempting MQTT connection..."));
    char charBuf[30];
    String CID (clientId + NodeID);
    CID.toCharArray(charBuf, 30);  
    
     #if defined (InternetServer)
     if (client.connect(charBuf, "flatboxadmin", "FBx_admin2012")) 
     {
      Serial.println(F("connected"));
     }
     #else
     if (client.connect(charBuf)) 
     {
      Serial.println(F("connected"));
     }
     #endif
     else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.print(F(" try again in 3 seconds,"));
      Serial.print(F(" retry #:"));
      Serial.println(retry);
      if (retry > 10){
        ESP.restart();
        retry=0;
      }
      retry++;
      // Wait 3 seconds before retrying
      delay(3000);
    }
  }
}
