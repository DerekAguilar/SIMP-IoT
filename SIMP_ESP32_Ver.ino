/*Librerías*/
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h> // generador tokens para sesiones
#include <addons/RTDBHelper.h> // Apoyo a uso de RTDB

/*Pines*/
#define pHSensor 34
#define tempSensorPin 33
#define tdsSensor 32
#define ldr 35
#define leds 22
/*Constantes*/
const int id=1;
#define VREF 3.3 // Referencia de voltaje para sensores pH y TSD
#define SCOUNT 30 // # Muestras a tomar de sensor TDS
const int interval=1900; // intervalo entre lecturas y para cambiar estado led
#define lightInterval 100 // intervalo del estado de luz
/*1. Definición de WiFi*/
const String ssid="";
#define pass ""
/*2. Clave API*/
#define API_KEY "" // <- Ir a menú -> Config -> "" del proyecto. Copiar valor y pegar aquí
/*3. Definición de URL RTDB*/
#define DBURL "" // <- Colocar URL que genera Realtime Database
/*4. Definición de correo y contraseñas*/
#define USEREMAIL ""
#define USERPASS ""
/*Declaración de objetos para base de datos*/
FirebaseData DBData; 
FirebaseAuth auth; 
FirebaseConfig conf;
/*Declarar objeto JSON para guardar datos*/
FirebaseJsonData qResult;
FirebaseJson json;
/*Declaración de sensores como objetos*/
OneWire tempSensor(tempSensorPin); // Pin como OneWire
DallasTemperature tSensor(&tempSensor); // Configurar como sensor
/*Variables*/
float ph; // valor de pH
/*valores para medir TSD*/
int analogBuffer[SCOUNT],phAnalogBuffer[SCOUNT]; // recolección de datos
int analogBufferTemp[SCOUNT];
int analogBufferIndex=0, copyIndex=0;
float tdsValue=0,temperature; //temperature=25 se usaba en el sensor TDS de referencia, se obtendrá antes del sensor de temperatura
/*variables para el sistema de luz*/
bool lightAutoMode=true, // <- Eliminar el true cuando una BD le de el valor
ledState=false, // Comprueba estado de leds
ledFlag=false; // Trigger para ejecutar función solo una vez
unsigned long prevMillis=0, // Cuenta de milisegundos anteriores
lightPrevInterval=0, // intervalos de lectura ldr
lightPrevMillis=0; // registra tiempo de luz antes de cambio de estado
unsigned long currentMillis; // contador general
int ldrVal; // resistencia que emite el ldr
bool signupOK;
/*Variable paths to database*/
const String phPath=String(id)+"/ph",
tdsPath=String(id)+"/tds",
tempPath=String(id)+"/temperature",
autoModePath=String(id)+"/lightAuto",
ledPath=String(id)+"/ledState";

void setup() {
  /*inicializar pines*/
  pinMode(pHSensor,INPUT);
  pinMode(tdsSensor,INPUT);
  pinMode(ldr,INPUT);
  pinMode(leds,OUTPUT);
  tSensor.begin();
  Serial.begin(9600); // iniciar Serial
  /*Inicializar WiFi y base de datos*/
  WiFi.begin(ssid,pass); // <- anotados mientras no se usen
  Serial.printf("Conectando a %s\n",ssid);
  while(WiFi.status()!= WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.printf("\nConectado. IP: ");
  Serial.println(WiFi.localIP());
  Serial.printf("Firebase Client v%s\n\n",FIREBASE_CLIENT_VERSION);
  /*Asignación de url bd y llave API (obligatorio)*/
  conf.api_key = API_KEY;
  conf.database_url = DBURL;
  /*Asignación de credenciales para iniciar sesión de usuario (necesitan tokens.h)*/
  auth.user.email=USEREMAIL;
  auth.user.password=USERPASS;
  /*Ajustar límite de datos de RX y TX*/
  DBData.setBSSLBufferSize(4096,1024); // RX, TX. Ambos pueden usar un búfer de 512-16384 bytes
  /*Asignar generación de tokens*/
  conf.token_status_callback=tokenStatusCallback;
  /*Iniciar base de datos*/
  Firebase.begin(&conf,&auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Conectado a Firebase.");
  printf("Set initial light mode value... %s\n",Firebase.setBool(DBData,F(&autoModePath),true) ? "ok": DBData.errorReason().c_str());
  printf("Set ledState placeholder... %s\n",Firebase.setBool(DBData,F(&ledPath),false) ? "ok": DBData.errorReason().c_str());
}

void loop() {
  currentMillis = millis();
  // Lecturas periódicas
  readTemp();
  readTDS();
  if(Firebase.ready()&&(currentMillis-prevMillis>=interval)) {
    readPH();
    prevMillis=currentMillis;
    //Introducción de datos a bd
    String modePath=id+"lightAutoMode";
    Serial.printf("Set ph... %s\n",Firebase.setInt(DBData,F(&phPath),ph) ? "ok" : DBData.errorReason().c_str());
    Serial.printf("Set TDS... %s\n",Firebase.setFloat(DBData,F(&tdsPath),tdsValue) ? "ok" : DBData.errorReason().c_str());
    Serial.printf("Set temperature... %s\n",Firebase.setFloat(DBData,F(&tempPath),temperature) ? "ok" : DBData.errorReason().c_str());
    Serial.printf("Get lightAutoMode value... %s\n",Firebase.getBool(DBData,F(&autoModePath),&lightAutoMode) ? lightAutoMode ? "ON" : "OFF" : DBData.errorReason().c_str());
  } 
  // Sistema de luz
  if(lightAutoMode)
    lightAuto();
  else
    lightManual();
}

// lectura Temperatura
void readTemp(){
  tSensor.requestTemperatures(); // Obtiene temperatura
  temperature=tSensor.getTempCByIndex(0); // Devuelve temperatura °C de sensor[0]
}

// lectura pH
void readPH(){
  const int samples=10;
  int buffer_arr[samples],placeholder;
  for (int i=0;i<samples;i++) {
    buffer_arr[i]=analogRead(pHSensor)/1.024;
    delay(10); //<- ver si pH varía mucho antes de cambiar
  }
  for(int i=0;i<samples-1;i++) {
    for(int j=i+1;j<samples;j++) {
      if(buffer_arr[i]>buffer_arr[j]) {
        placeholder=buffer_arr[i];
        buffer_arr[i]=buffer_arr[j];
        buffer_arr[j]=placeholder;
      }
    }
  }
  int avgval=0;
  for(int i=2;i<8;i++)
    avgval+=buffer_arr[i];
  float volt=(float)avgval*VREF/4096/6; // para 5V, volt=(float)avgval*5.0/1024/6;
  ph=getPh(volt);
}

float getPh(float volt) {
  float calibration_value=21.34+2.5; // valor calibración de sensor pH. 1.7 obtenido de prueba y error.
  return -5.70*volt+calibration_value;
}

// lectura a sampling TDS cada 40 milisegundos
void readTDS() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis()-analogSampleTimepoint>40U) { 
    analogSampleTimepoint=millis();
    analogBuffer[analogBufferIndex]=analogRead(tdsSensor); // lee y manda al búfer
    analogBufferIndex++;
    if (analogBufferIndex==SCOUNT)
      analogBufferIndex=0;
  }
  static unsigned long printTimepoint = millis();
  if (millis()-printTimepoint>800U) {
    printTimepoint=millis();
    for (copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      analogBufferTemp[copyIndex]=analogBuffer[copyIndex];
    float averageVoltage=getMedianNum(analogBufferTemp, SCOUNT)*(float)VREF/4096.0; // obtiene mediana y convierte a valor de voltaje -> poner 1024 en lugar de 4096 con Arduino regular
    float compensationCoefficient=1.0+0.02*(25.0-25.0); // fórmula de compensación de temperatura: fFinalResult(25^C)=fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVoltage=averageVoltage/compensationCoefficient; // compensación de temperatura
    tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage-255.86*compensationVoltage*compensationVoltage+857.39*compensationVoltage)*0.5; // voltaje a TSD
    //Serial.print("voltage:"); <- impresión de datos en forma de voltaje
    //Serial.print(averageVoltage,2);
    //Serial.print("V ");
    // Serial.print("TDS Value:"); <- Impresión de datos en ppm
    // Serial.print(tdsValue, 0);
    // Serial.println("ppm");
  }
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

// Sistema de luces
// Modo automático
void lightAuto(){
  // Lectura periódica de datos
  if (currentMillis - lightPrevInterval>=lightInterval) {
    ldrVal=analogRead(ldr);
  lightPrevInterval=currentMillis;
  }
  // Cambiar el valor booleano si lectura análoga <100
  if (ldrVal>3900&&ledState==false){
    if (ledFlag==false) {
      lightPrevMillis=currentMillis;
      ledFlag=true;
    }
  } 
  else {
    if(ldrVal<=3900&&ledState==true){
      if (ledFlag==true){
        lightPrevMillis=currentMillis;
        ledFlag=false;
      }
    }
  }

  // Encender o apagar el LED pasado intervalo del cambio
  if (ledFlag && (currentMillis - lightPrevMillis >= interval)) {
    ledState=true;
    digitalWrite(leds, HIGH);
    printf("Set ledState to ON... %s\n",Firebase.setBool(DBData,F(&ledPath),true) ? "ok": DBData.errorReason().c_str());
  } else if (!ledFlag && (currentMillis - lightPrevMillis >= interval)) {
    ledState=false;
    digitalWrite(leds, LOW);
    printf("Set ledState to OFF... %s\n",Firebase.setBool(DBData,F(&ledPath),false) ? "ok": DBData.errorReason().c_str());
  }
}

void lightManual(){
  if (currentMillis - lightPrevInterval>=lightInterval) {
    Serial.printf("Get ledState... %s\n", Firebase.getBool(DBData,F(&ledPath),&ledState) ? ledState ? "ON" : "OFF" : DBData.errorReason().c_str());
    if (ledState)
      digitalWrite(leds,HIGH);
    else
      digitalWrite(leds,LOW);
    lightPrevInterval=currentMillis;
  }
}
