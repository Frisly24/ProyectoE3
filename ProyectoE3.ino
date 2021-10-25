
#include <WiFi.h>
#include <PubSubClient.h> //Líberia descargar
#include <Wire.h>

//-------------RFID-----------------

#include <SPI.h>
#include <MFRC522.h>
//-------------------------------------


//-------------SERVO-----------------
#include <ESP32Servo.h>
//-------------------------------------



WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

float temperatura = 0;  //Señal1 que queremos publicar al brocker
int temperatura_cc = 0;

const int ledPin1 = 22;  //LedPin GPIO2
const int ledPin2 = 4;  //LedPin GPIO4

const int ledPinAC = 26; //LedPin GPIO15

//-------------PWM-----------------

//-------------------------------------

//-------------BUZZER-----------------
int Buzzer = 14; //for ESP32 Microcontroller
int Buz_c = 0;
//-------------------------------------


//-------------ULTRASONICO-----------------

int TRIG = 15;      // trigger en pin 15
int ECO = 2;      // echo en pin 2

int DURACION;
int DISTANCIA;

int Ultra_C = 0;

//-------------------------------------

//-------------LASER-----------------
int laserPin = 13;
int laser_c = 0;
//-------------------------------------

//-------------FOTORRESISTENCIA-----------------

int sensor = 35;
int lectura;
//-------------------------------------


//-------------RFID-----------------

constexpr uint8_t RST_PIN = 5;     // Configurable, see typical pin layout above
constexpr uint8_t SS_PIN = 21;     // Configurable, see typical pin layout above


MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key;

String tag;

int puerta = 0;
//-------------------------------------


//-------------LUMINOSIDAD-----------------


const int LDRPin = 32;   //Pin del LDR


float V;


int luz = 0;

//-------------------------------------

//-------------SERVO-----------------
Servo miServo;
int PINSERVO = 12;
//-------------------------------------


//-------------SENSOR DE VIENTO-----------------
int ckviento = 0;


float viento=0;


//**********SSID-Pasword de nuestro servidor a Internet*******
const char* ssid = "Network"; // wifi
const char* password = "5978451705"; // Contreseña

//**********Dirección del MQTT Broker IP address*************
const char* mqtt_server = "192.168.7.112";


//-----------------------VOID SETUP()---------------------
void setup()
{
   Serial.begin(115200);
   setup_wifi(); 
   client.setServer(mqtt_server, 1883); 
   client.setCallback(callback);

   pinMode(ledPin1, OUTPUT);
   pinMode(ledPin2, OUTPUT);

   pinMode(ledPinAC, OUTPUT);


  //-------------VOID SETUP()Buzzer-----------------
  pinMode (Buzzer, OUTPUT);
  digitalWrite (Buzzer, HIGH); //turn buzzer off
  //-------------------------------------

  //-----------------------VOID SETUP()ULTRASONICO--------------------- 
  pinMode(TRIG, OUTPUT);  // trigger como salida
  pinMode(ECO, INPUT);    // echo como entrada

  //-------------VOID SETUP()Laser_Fotorresistencia-----------------
  pinMode(laserPin, OUTPUT);
  //-------------------------------------

  //-------------RFID-----------------

  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522
  //-------------------------------------

  
  //-------------LUMINOSIDAD-----------------
  pinMode(27, INPUT);
  Serial.flush(); while(Serial.available()>0)Serial.read();

   
  //--------------------------------------

  //-------------SERVO-----------------
  miServo.attach(PINSERVO);
  miServo.write(0);       //Ángulo de giro en grados.
  //--------------------------------------
 }

//----------------------VOID SETUP-WIFI()------Realiza conección con el WIFI de internet
void setup_wifi()
{
   delay(10);
   Serial.println();
   Serial.print("Conectando a ");
   Serial.print(ssid);
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED)
  {
  delay(500);
  Serial.print(".");
  }
   Serial.println("");
   Serial.println("Wifi conectado");
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());
}

//---------------------VOID CALLBACK()-----------Realiza la conexión MQTT (Suscriber: recibe datos)
void callback(char* topic, byte* message, unsigned int length)
{
   Serial.print("mensaje recibido en topic: ");
   Serial.print(topic);
   Serial.print(", Message: ");
   String messageTemp;
   for (int i = 0; i < length; i++){
  Serial.print((char)message[i]);
  messageTemp += (char)message[i];
  }
Serial.println();   


 //***************Primer Output (usando topic: esp32//output1)*************
 if (String(topic) == "esp32/output1") //Si recibe mensaje sobre topic 'esp32/output', verifica si es "on" o "off"
 {
    Serial.print("Cambio de salida: ");
    if(messageTemp == "on")
    {
      Serial.println("on");
      digitalWrite(ledPin1,HIGH);
    }
    else if(messageTemp == "off")
    {
      Serial.println("off");
      digitalWrite(ledPin1, LOW);
    }
 }

 //***************Segundo Output (usando topic: esp32//output2)*************
 if (String(topic) == "esp32/output2") //Si recibe mensaje sobre topic 'esp32/output2', verifica si es "on" o "off"
 {
    Serial.print("Cambio de salida: ");
    if(messageTemp == "on")
    {
      Serial.println("on");
      digitalWrite(ledPin2,HIGH);
    }
    else if(messageTemp == "off")
    {
      Serial.println("off");
      digitalWrite(ledPin2, LOW);
    }
 }



//*****************Quinto Output (usando topic: esp32/buzzer)******************
 if (String(topic) == "esp32/buzzer") //Si recibe mensaje sobre topic 'esp32/buzzer', Manda al PWM (0-255)
 {
       
    if(messageTemp == "on")
    {
     Buz_c=1;
     Serial.print("Buzzer encendido");
     Serial.print("El valor de la condicion es:");
     Serial.print(Buz_c);
     digitalWrite (Buzzer, LOW);  //turn buzzer on
    }
    else if(messageTemp == "off")
    {
     Buz_c=0;
     Serial.print("Buzzer Apagado");
     Serial.print("El valor de la condicion es:");
     Serial.print(Buz_c);
     digitalWrite (Buzzer, HIGH); //turn buzzer off
    }

 }



//***************Sexto Output (usando topic: esp32//Ultra)*************
 if (String(topic) == "esp32//Ultra") //Si recibe mensaje sobre topic 'esp32//Ultra', verifica si es "on" o "off"
 {
    
    if(messageTemp == "on")
    {
     Ultra_C=1;
     Serial.print("Sistema Ultrasonico encendido");
     Serial.print("El valor de la condicion es:");
     Serial.print(Ultra_C);
    
    }
    else if(messageTemp == "off")
    {
     Ultra_C=0;
     Serial.print("Sistema Ultrasonico apagado");
     Serial.print("El valor de la condicion es:");
     Serial.print(Ultra_C);
     client.publish("esp32/buzzer", "off"); //Topic: esp32/buzzer
    }


 }

//***************Septimo Output (usando topic: esp32/Laser)*************
 if (String(topic) == "esp32/Laser") //Si recibe mensaje sobre topic 'esp32/Laser', verifica si es "on" o "off"
 {
    
    if(messageTemp == "on")
    {
     laser_c = 1;
     Serial.print("Sistema Laser encendido");
     Serial.print("El valor de la condicion es:");
     Serial.print(laser_c);
     digitalWrite (laserPin, HIGH); //turn Laser on
    }
    else if(messageTemp == "off")
    {
     laser_c = 0;
     Serial.print("Sistema Laser apagado");
     Serial.print("El valor de la condicion es:");
     Serial.print(laser_c);
     digitalWrite (laserPin, LOW); //turn Laser off
     client.publish("esp32/buzzer", "off"); //Topic: esp32/buzzer
    }


 }


//***************Octavo Output (usando topic: esp32/puerta)*************
 if (String(topic) == "esp32/puerta") //Si recibe mensaje sobre topic 'esp32/puerta', verifica si es "on" o "off"
 {
    
    if(messageTemp == "on")
    {
     puerta = 1;
     Serial.print("Puerta abierta");

     
     miServo.write(120);
    }
    else if(messageTemp == "off")
    {
     puerta = 0;
     Serial.print("Puerta Cerrada");


     miServo.write(0);
    }


 }



//***************Noveno Output (usando topic: esp32/luminosidad)*************
 if (String(topic) == "esp32/luminosidad") //Si recibe mensaje sobre topic 'esp32/luminosidad', verifica si es "on" o "off"
 {
    
    if(messageTemp == "on")
    {
     luz = 1;
     Serial.print("Sistema Luminosidad encendido");
    }
    else if(messageTemp == "off")
    {
     luz = 0;
     Serial.print("Sistema Luminosidad Apagado");
    }


 }


//***************Decimo Output (usando topic: esp32/luminosidad)*************
 if (String(topic) == "esp32/temp") //Si recibe mensaje sobre topic 'esp32/luminosidad', verifica si es "on" o "off"
 {
    
    if(messageTemp == "on")
    {
     temperatura_cc = 1;
     Serial.print("Sistema temperatura encendido");
    }
    else if(messageTemp == "off")
    {
     temperatura_cc = 0;
     Serial.print("Sistema temperatura Apagado");
    }


 }


 //***************Onceavo Output (usando topic: esp32/AC)*************
 if (String(topic) == "esp32/AC") //Si recibe mensaje sobre topic 'esp32/AC', verifica si es "on" o "off"
 {
    
    if(messageTemp == "on")
    {

     
     Serial.print("luz AC encendida");
     digitalWrite(ledPinAC,LOW);
    }
    else if(messageTemp == "off")
    {
     
     Serial.print("luz AC apagada");
     digitalWrite(ledPinAC,HIGH);
    }


 }


//***************Doceavo Output (usando topic: esp32/checkviento)*************
 if (String(topic) == "esp32/checkviento") //Si recibe mensaje sobre topic 'esp32/checkviento', verifica si es "on" o "off"
 {
    
    if(messageTemp == "on")
    {
    Serial.print("Sensor de viento activado");
    ckviento = 1;      
    }
    else if(messageTemp == "off")
    {
    Serial.print("Sensor de viento desactivado"); 
    ckviento = 0;    
    }


 }
 
 
 
 //Aqui es pueden agregar más GPIO controladas por el protocolo MQTT
}




//---------------------------VOID RECONNECT()------------------------ Realiza la reconexión en caso de fallo
void reconnect()
{
   //Bucle hasta que se reconecte
   while (!client.connected())
   {
     Serial.print("Intentando conexión MQTT...");
     String clienteId = "ESP32";
     if (client.connect(clienteId.c_str()))
     {
     Serial.println("conectado");
     client.subscribe("esp32/output1"); //******Topic: 'esp32/output1'*****
     client.subscribe("esp32/output2"); //******Topic: 'esp32/output2'*****
     client.subscribe("esp32/output3"); //******Topic: 'esp32/output3'*****
     client.subscribe("esp32/output4"); //******Topic: 'esp32/output4'*****
     client.subscribe("esp32/output5"); //******Topic: 'esp32/output4'*****
     client.subscribe("esp32/buzzer"); //******Topic: 'esp32/buzzer'*****
     client.subscribe("esp32//Ultra"); //******Topic: 'esp32/buzzer'*****
     client.subscribe("esp32/Laser"); //******Topic: 'esp32/buzzer'*****
     client.subscribe("esp32/puerta"); //******Topic: 'esp32/buzzer'*****
     client.subscribe("esp32/luminosidad"); //******Topic: 'esp32/buzzer'*****     
     client.subscribe("esp32/temp"); //******Topic: 'esp32/buzzer'*****    
     client.subscribe("esp32/AC"); //******Topic: 'esp32/buzzer'*****    
     client.subscribe("esp32/viento"); //******Topic: 'esp32/buzzer'*****
     client.subscribe("esp32/checkviento"); //******Topic: 'esp32/buzzer'*****
     }else{
     Serial.print("Fallo,rc=");
     Serial.print(client.state());
     Serial.println(" Intentelo de nuevo en 5s");
     delay(5000);
     }
   }
}

//------------------------VOID LOOP()------------- Realiza conección MQTT (Publisher: envía datos)
void loop()
{
   if (!client.connected())
  {
  reconnect();
  }
   client.loop();

   long now = millis();
   if (now - lastMsg > 1000) // 100ms(tiempo de muestreo)
   {
     lastMsg = now;


     if (temperatura_cc == 1){

     //***********señal 1 que queremos enviar al Brocker************

     
     temperatura = ((analogRead(A0)*(3300.0/4096.0))/10)+9;

     //Convertir el valor a char array
     char tempString[8];
     dtostrf(temperatura, 1, 2, tempString);
     Serial.print("Temperatura: ");
     Serial.println(tempString);
     client.publish("esp32/temperature", tempString); //Topic: 'esp32/temperature'
      
     }


          //------------------------VOID LOOP VIENTO()------------- 

     if (ckviento == 1){
     
     viento = analogRead(39)*(0.190);

     //Convertir el valor a char array
     char tempString2[8];
     dtostrf(viento, 1, 2, tempString2);
     Serial.print("La velocidad del viento es: ");
     Serial.println(tempString2);
     client.publish("esp32/viento", tempString2); //Topic: esp32/viento
      
     }
     
    //------------------------LUMINOSIDAD()------------- 

    if (luz ==1)
    {
      
    V = analogRead(LDRPin);         
    
  
 

        //PUBLICACIÓNN


     //Convertir el valor a char array
     char tempString3[8];
     dtostrf(V, 1, 2, tempString3);
     Serial.print("La iluminación es: ");
     Serial.println(tempString3);
     client.publish("esp32/luminosidad", tempString3); //Topic: esp32/viento


    }

     


//FIN
   }


     //------------------------VOID LOOP ULTRASONICO()------------- 
     if (Ultra_C == 1)
      {
      
     digitalWrite(TRIG, HIGH);    // generacion del pulso a enviar
     delay(1);        // al pin conectado al trigger
     digitalWrite(TRIG, LOW);   // del sensor
  
     DURACION = pulseIn(ECO, HIGH); // con funcion pulseIn se espera un pulso
            // alto en Echo
     DISTANCIA = DURACION / 58.2;   // distancia medida en centimetros
     Serial.print("La distancia es: ");
     Serial.println(DISTANCIA);   // envio de valor de distancia por monitor serial
     delay(200);        // demora entre datos

     if (DISTANCIA >= 1205 && DISTANCIA >= 0 && Ultra_C == 1) {  // si distancia entre 0 y 5 cms.
 
      client.publish("esp32/buzzer", "on"); //Topic: esp32/buzzer
       }
      
      }
    

     //------------------------VOID LOOP FOTORRESISTENCIA()------------- 

     if (laser_c == 1){

     lectura = analogRead(sensor);
     Serial.print("Laser es: ");
     Serial.println(lectura);
      if(lectura < 1300 && laser_c == 1 ){
        Serial.println("Alarma");
        client.publish("esp32/buzzer", "on"); //Topic: esp32/buzzer
      }
      
      
     }
     
     


    
     
     //------------------------RFID()-------------       

      if ( ! rfid.PICC_IsNewCardPresent())
        return;
      if (rfid.PICC_ReadCardSerial()) {
        for (byte i = 0; i < 4; i++) {
          tag += rfid.uid.uidByte[i];
        }
      Serial.println(tag);

        if (tag == "1063810037") {
      Serial.println("Access Granted!");


      if (puerta == 0)
        {
        client.publish("esp32/puerta", "on"); //Topic: esp32/puerta
        puerta = 1;
        }
      else if(puerta == 1) {
        client.publish("esp32/puerta", "off"); //Topic: esp32/puerta
        puerta = 0;
      }


      
      } 
      
      else {
      Serial.println("Access Denied!");
      }
      
      tag = "";
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
     } 
     


 


     
}
