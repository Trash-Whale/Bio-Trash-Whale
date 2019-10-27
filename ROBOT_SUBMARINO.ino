//Trash Whale

//It consists of an unmanned submarine that collects small, micros and nano
// plastic fragments, within the marine ecosystem.
// The filtering system with five sieves presented by the submarine, manages to collect what is below the surface of the sea, something that until today had not been raised.


//***The pseudocode is designed for a non-functional scale. It is for demonstration purposes.


//For more information about the code visit: https://github.com/Trash-Whale/Bio-Trash-Whale
//=======================================================
//                        CONEXIONES
//=======================================================

//Conexiones PUENTE H
int ENA = 8; //PIN que Controla Velocidad de Giro Motor Derecho
int IN1 = 9; //Controla Motores Derechos
int IN2 = 10; //Controla Motores Derechos


//CONEXIONES GIROSCOPIO
int Scl = A5;
int Sda = A4;
int Int = 0;

//CONEXION SERVOS
//servoDERECHA = 5
//servoIZQUIERDA = 6
//servoDORSAL = 7

//CONEXIONES GPS
int TX_GPS = 4;
int RX_GPS = 3;

//CONEXIONES SENSOR DE CORRIENTE
int OUT_TV = A0;


//=======================================================
//                       SERVOS
//=======================================================
#include <Servo.h>
int pos = 0;
Servo servoDERECHA;
Servo servoIZQUIERDA;
Servo servoDORSAL;

//PUENTE H
int TIEMPO_HELICE = 10000;
int TRANSICION = 500;


//=======================================================
//                         GPS
//=======================================================
#include <SoftwareSerial.h>//incluimos SoftwareSerial
#include <TinyGPS.h>//incluimos TinyGPS

TinyGPS gps;//Declaramos el objeto gps
SoftwareSerial serialgps(4,3);//Declaramos el pin 4 Tx y 3 Rx

//Variables GPS
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;



//                         GIROSCOPIO
//=======================================================



//Incluimos la librería para I2C
#include <Wire.h>
 
//Definimos la direccion I2C del MPU
#define MPU 0x68
 
//Definimos los ratios de conversión
#define A_R 16384.0
#define G_R 131.0
 
//Definimos la conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//El MPU da los valores en enteros de 16 bits
//Declaramos memorias para los valores brutos
int16_t AcX; 
int16_t AcY; 
int16_t AcZ;
int16_t GyX; 
int16_t GyY;
int16_t GyZ;
 
//Declaramos cadena de caracteres para los ángulos
float Acc[2];
float Gy[2];
float Angle[2];


//SENSOR DE CORRIENTE
float Sensibilidad=0.1; //sensibilidad en Voltios/Amperio para sensor de 20 Amperios


//=======================================================
//                   VOID SET UP
//=======================================================
void setup()
{
 

//                         GPS
  //Serial.begin(115200);//Iniciamos el puerto serie
  serialgps.begin(9600);//Iniciamos el puerto serie del gps
  //Imprimimos:
  Serial.println("");
  Serial.println("GPS GY-GPS6MV2 Leantec");
  Serial.println(" ---Buscando señal--- ");
  Serial.println("");



//                         GIROSCOPIO
  //Inicializamos el I2C y el puerto serie
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);


//                       SERVOS
servoDERECHA.attach(5, 600, 1500);
servoIZQUIERDA.attach(6, 600, 1500);
servoDORSAL.attach(7, 600, 1500);

//                       PUENTE H
//DEfino el tipo de salida que es cada pin del Puente h
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(ENA, OUTPUT);

}

//=======================================================
//                  VOOIP LOOP
//=======================================================
void loop()
{
  GPS();
  CORRIENTE();
  ESTABILIDAD();
  ADELANTE();
  delay(TRANSICION);
  DOBLAR_DERECHA();
  delay(TRANSICION);
  ADELANTE();
  delay(TRANSICION);
  DOBLAR_IZQUIERDA();
  delay(TRANSICION);
}

//=======================================================
//                   ESTABILIDAD
//=======================================================
void ESTABILIDAD()
{
  ESTABLEMX();
  ESTABLEmX();
  ESTABLEMY();
  ESTABLEmY();
  }

void ESTABLEMX()
{
  if((Angle[0])>3)
  DERECHA_ABAJO;
  IZQUIERDA_ARRIBA;
    }

void ESTABLEmX()
{
  if((Angle[0])<-3)
  DERECHA_ARRIBA;
  IZQUIERDA_ABAJO;
    }
    
void ESTABLEMY()
{
  if((Angle[1])>3)
  DERECHA_ABAJO;
  IZQUIERDA_ABAJO;
    }

void ESTABLEmY()
{
  if((Angle[1])<-3)
  DERECHA_ARRIBA;
  IZQUIERDA_ARRIBA;
    }
  
//=======================================================
//                   RUTINA
//=======================================================

void ADELANTE()
{
    servoDERECHA.write(90); 
    servoIZQUIERDA.write(90); 
    servoDORSAL.write(90); 
    HELICE(); 
    ESTABILIDAD();
   }


void DOBLAR_DERECHA()
{
    servoDERECHA.write(90); 
    servoIZQUIERDA.write(90); 
    servoDORSAL.write(45); 
    HELICE();
  }


void DOBLAR_IZQUIERDA()
{
    servoDERECHA.write(90); 
    servoIZQUIERDA.write(90); 
    servoDORSAL.write(135);
    HELICE(); 
    }

void EMERGER()
{
   DERECHA_ARRIBA();
   IZQUIERDA_ARRIBA();
    servoDORSAL.write(90); 
    HELICE();
    }

void SUEMERGIR()
{
   DERECHA_ABAJO();
   IZQUIERDA_ABAJO();
    servoDORSAL.write(90); 
    HELICE();
    }
    
void IZQUIERDA_ARRIBA()
{
    servoIZQUIERDA.write(45); //ANGULO DE 45 GRADOS ARRIBA
   }


void DERECHA_ARRIBA()
{
    servoDERECHA.write(135); //ANGULO DE 45 GRADOS ARRIBA
   }

void IZQUIERDA_ABAJO()
{
    servoIZQUIERDA.write(135); //ANGULO DE 45 GRADOS ABAJO
   }


void DERECHA_ABAJO()
{
    servoDERECHA.write(45); //ANGULO DE 45 GRADOS ABAJO
   }

void HELICE()
{
    digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);
  delay(TIEMPO_HELICE);
   }

//=======================================================
//                   SENSOR DE CORRIENTE
//=======================================================
void CORRIENTE()
{
  float I=get_corriente(200);//obtenemos la corriente promedio de 500 muestras 
  Serial.print("Corriente: ");
  Serial.println(I,3); 
  delay(100);     
}

float get_corriente(int n_muestras)
{
  float voltajeSensor;
  float corriente=0;
  for(int i=0;i<n_muestras;i++)
  {
    voltajeSensor = analogRead(A0) * (5.0 / 1023.0);////lectura del sensor
    corriente=corriente+(voltajeSensor-2.5)/Sensibilidad; //Ecuación  para obtener la corriente
  }
  corriente=corriente/n_muestras;
  return(corriente);
}



//=======================================================
//                         GIROSCOPIO
//=======================================================
//https://leantec.es/tutorial-arduino-acelerometro-giroscopo-mp/

 void GIROSCOPIO(){
 //Leemos los valores del Acelerometro
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedimos el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   //Calculamos los angulos Y, X respectivamente.
  Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
  Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leemos los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,4,true); //A diferencia del Acelerometro, solo se piden 4 registros
   GyX=Wire.read()<<8|Wire.read();
   GyY=Wire.read()<<8|Wire.read();
 
   //Calculamos el angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
 
   //Aplicamos un Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*0.010) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*0.010) + 0.02*Acc[1];
 
   //Mostramos los valores por el monitor serial
   Serial.print("Angle X: "); Serial.println(Angle[0]);
   Serial.print("Angle Y: "); Serial.println(Angle[1]);
   
  //Esperamos 1 segundo para poder visualizarlo en el monitor serial
   delay(1000);
}



 
//=======================================================
//                         GPS
//======================================================= 
 void GPS(){
  
  while(serialgps.available()) 
  {
    int c = serialgps.read();
 
    if(gps.encode(c))  
    {
      float latitude, longitude;
      gps.f_get_position(&latitude, &longitude);
      Serial.print("Latitud/Longitud: "); 
      Serial.print(latitude,5); 
      Serial.print(", "); 
      Serial.println(longitude,5);


  gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
      Serial.print("Fecha: "); 
      Serial.print(day, DEC); 
      Serial.print("/"); 
      Serial.print(month, DEC); 
      Serial.print("/"); 
      Serial.print(year);
      Serial.print(" Hora: "); 
      Serial.print(hour, DEC); 
      Serial.print(":"); 
      Serial.print(minute, DEC); 
      Serial.print(":"); 
      Serial.print(second, DEC); 
      Serial.print("."); 
      Serial.println(hundredths, DEC);
      Serial.print("Altitud (metros): ");
      Serial.println(gps.f_altitude()); 
      Serial.print("Rumbo (grados): "); 
      Serial.println(gps.f_course()); 
      Serial.print("Velocidad(kmph): ");
      Serial.println(gps.f_speed_kmph());
      Serial.print("Satelites: "); 
      Serial.println(gps.satellites());
      Serial.println();
      gps.stats(&chars, &sentences, &failed_checksum);  
    }
  }

  
}
