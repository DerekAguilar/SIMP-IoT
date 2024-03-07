/*Pines*/
#define pHSensor 34
#define tempSensorPin 33
#define tdsSensor 32
#define ldr 23
#define leds 22
/*Constantes*/
const byte id=1;
#define VREF 3.3 // Referencia de voltaje para sensores pH y TSD
#define SCOUNT 30 // # Muestras a tomar de sensor TDS
#define interval 2000 // intervalo entre lecturas y para cambiar estado led
#define lightInterval 100 // intervalo del estado de luz
const String ssid="UTT-Docentes P";
#define pass="Docente%$";
/*Librerías*/
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
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

void setup() {
  // inicializar pines
  pinMode(pHSensor,INPUT);
  pinMode(tdsSensor,INPUT);
  pinMode(ldr,INPUT);
  pinMode(leds,OUTPUT);
  tSensor.begin();
  Serial.begin(9600); // iniciar Serial
  //WiFi.begin(ssid,pass); // <- anotados mientras no se usen
  //Serial.print("Conectando a ");
  //Serial.print(ssid);
  //while(WiFi.status!= WL_CONNECTED) {
  //  Serial.print(".");
  //  delay(500);
  //}
  //Serial.println("Conectado.");
}

void loop() {
  currentMillis = millis();
  // Lecturas periódicas
  readTemp();
  readTDS();
  if(currentMillis-prevMillis>=interval) {
    readPH();
    prevMillis=currentMillis;
    Serial.print("Tmp: ");
    Serial.print(temperature);
    Serial.print("°C pH: ");
    Serial.print(ph);
    Serial.print(" STD: ");
    Serial.print(tdsValue);
    Serial.print(" ppm ldr: ");
    Serial.println(ldrVal);
  }
  // Sistema de luz
  if(lightAutoMode)
    lightAuto();
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
    // delay(); <- ver si pH varía mucho antes de cambiar
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
  float calibration_value=21.34+1.7; // valor calibración de sensor pH. 1.7 obtenido de prueba y error.
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
    float averageVoltage=getMedianNum(analogBufferTemp, SCOUNT)*(float)VREF/1024.0; // obtiene mediana y convierte a valor de voltaje
    float compensationCoefficient=1.0+0.02*(temperature-25.0); // fórmula de compensación de temperatura: fFinalResult(25^C)=fFinalResult(current)/(1.0+0.02*(fTP-25.0));
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
  if (ldrVal>600&&ledState==false){
    if (ledFlag==false) {
      lightPrevMillis=currentMillis;
      ledFlag=true;
    }
  } 
  else {
    if(ldrVal<=600&&ledState==true){
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
  } else if (!ledFlag && (currentMillis - lightPrevMillis >= interval)) {
    ledState=false;
    digitalWrite(leds, LOW);
  }
}
