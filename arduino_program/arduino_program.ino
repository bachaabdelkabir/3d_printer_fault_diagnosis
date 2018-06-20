// ****************************************************************************************
// BACHA Abdelkabir - Equipe Optimisation des Systèmes Industriels et Logistiques
// - Laboratoire LRI - ENSEM - UH2C 
// Mesure de température inspiré de https://arduino-info.wikispaces.com/MultipleTemperatureSensorsToLCD
// Mesure de tension inspiré de http://startingelectronics.org/articles/arduino/measuring-voltage-with-arduino/
// Mesure de vitesse de rotation inspiré de http://www.instructables.com/id/Measure-RPM-DIY-Portable-Digital-Tachometer/?ALLSTEPS
// Multiplixage http://playground.arduino.cc/learning/4051
// ****************************************************************************************

//*****************************************************************************************
// Inclusions des bibliothèques

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 7                   // Data wire is plugged into pin 7 on the Arduino

//*****************************************************************************************
// Initialisation de la bibliothèque One Wire
  OneWire oneWire(ONE_WIRE_BUS);         // Setup a oneWire instance to communicate with 
                                         // any OneWire devices (not just Maxim/Dallas 
                                         // temperature ICs)
                                         
//*****************************************************************************************
// Initialisation de la bibliothèque Dallas Temperature
// Pass our oneWire reference to Dallas Temperature.
  DallasTemperature temperture_sensors(&oneWire);   
  
//*****************************************************************************************
// Déclaration des variables

#define NUM_SAMPLES 10
const int ledPin =  13;                  // the number of the LED pin


//--------------------------- 4051 Multiplexer section--------------------------------------
const int A_pin = 2;                         // A pin off 4051
const int B_pin = 3;                         // B pin off 4051
const int C_pin = 4;                         // C pin off 4051

      int r0 = 0;                            //value of select pin at the 4051 (s0)
      int r1 = 0;                            //value of select pin at the 4051 (s1)
      int r2 = 0;                            //value of select pin at the 4051 (s2)
      int count = 0;                         //which y pin we are selecting
      
//---------------------------END of 4051 Multiplexer section--------------------------------

      int courant_mot;
      int mVperAmp = 66;                 // use 100 for 20A Module and 66 for 30A Module
      float RawValue= 0;
      int ACSoffset = 2500; 
      double AmpsVoltage = 0;
      double Amps = 0;
      int  pot_value;

      double sum1 = 0;                      // sum of samples taken for motor current measerment
      double sum2 = 0;                      // sum of samples taken for motor voltage measurment 
      double sum3 = 0;                      // sum of samples taken for generator voltage measurment  
      unsigned char sample_count = 0;         // current sample number
      float Tension_supply = 0.0;              // calculated voltage
      float Tension_Heatbed = 0.0;
      float Tension_Hotend = 0.0;

      DeviceAddress Probe01 = { 0x28, 0xFF, 0xA3, 0x27, 0x68, 0x14, 0x03, 0x76 }; 
      DeviceAddress Probe02 = { 0x28, 0xFF, 0x6C, 0x4C, 0x6D, 0x14, 0x04, 0x89 };
      DeviceAddress Probe03 = { 0x28, 0xFF, 0x74, 0x63, 0x6D, 0x14, 0x04, 0x21 };

      unsigned int  ADCValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      double        Voltage[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      double        Vcc;

      const int D8 =  8;      // the number of the D8 pin
      const int D10 =  10;      // the number of the D10 pin
      const int E_EN =  22;      // the number of the D10 pin
      const int X_EN =  24;      // the number of the D10 pin
      const int Y_EN =  26;      // the number of the D10 pin               
      const int Z_EN =  28;      // the number of the D10 pin 
      
//*****************************************************************************************
// initialisations Générales 
void setup() {

  Serial.begin(9600);                    // initialize serial communication at 9600 bits 
                                         // per second:
//  temperture_sensors.begin();            // Start up the library
                                         // IC Default 9 bit. If you have troubles consider 
                                         // upping it 12. Ups the delay giving the IC more 
                                         // time to process the temperature measurement
                                         // set the resolution to 9 bit (Can be 9 to 12 bits .. lower is faster)
  temperture_sensors.setResolution(Probe01, 9);
  temperture_sensors.setResolution(Probe02, 9);
  temperture_sensors.setResolution(Probe03, 9); 
  
  pinMode(ledPin, OUTPUT);               // initialize the LED pin as an output:
  
//--------------------------- 4051 Multiplexer section--------------------------------------
  pinMode(A_pin, OUTPUT);               // initialize the A pin off 4051 as an output:
  pinMode(B_pin, OUTPUT);               // initialize the B pin off 4051 as an output:
  pinMode(C_pin, OUTPUT);               // initialize the C pin off 4051 as an output:
//---------------------------END of 4051 Multiplexer section--------------------------------

  pinMode(D8, INPUT);               // initialize the D8 pin as an input:
  pinMode(D10, INPUT);              // initialize the D10 pin as an input:
  pinMode(E_EN, INPUT);              // initialize the D10 pin as an input:  
  pinMode(X_EN, INPUT);              // initialize the D10 pin as an input: 
  pinMode(Y_EN, INPUT);              // initialize the D10 pin as an input: 
  pinMode(Z_EN, INPUT);              // initialize the D10 pin as an input:   
  
}
//*****************************************************************************************
// Boocle Principale
void loop() {

//---------------------------Acquisition des Données--------------------------------------
Vcc = readVcc()/1000.0;




//--------------------------- 4051 Multiplexer section--------------------------------------
//  for (count=0; count<=7; count++) {
//
// 
//
//    // select the bit  
//
//    r0 = bitRead(count,0);    // use this with arduino 0013 (and newer versions)     
//    r1 = bitRead(count,1);    // use this with arduino 0013 (and newer versions)     
//    r2 = bitRead(count,2);    // use this with arduino 0013 (and newer versions)     
//
//    //r0 = count & 0x01;      // old version of setting the bits
//    //r1 = (count>>1) & 0x01;      // old version of setting the bits
//    //r2 = (count>>2) & 0x01;      // old version of setting the bits
//
//    digitalWrite(A_pin, r0);
//    digitalWrite(B_pin, r1);
//    digitalWrite(C_pin, r2);
//
// 
//
//    //read the multiplexed pin here
//
//// take a number of analog samples and add them up
//    while (sample_count < NUM_SAMPLES) {
//        sum1 += analogRead(A0);
//        sample_count++;
////       delay(1);
//    }
//    sample_count = 0;
//
//
//    ADCValue[count] = sum1;
//    Voltage[count] = (ADCValue[count] / (NUM_SAMPLES*1024.0)) * Vcc; //For accurate measurements, divide the ADC value by the ADC maximum value and multiply by the resultant ADV value.
//
//    
//    }

// Read PIN 0
    digitalWrite(A_pin, 0);
    digitalWrite(B_pin, 0);
    digitalWrite(C_pin, 0);

    ADCValue[0] = analogRead(A0);
    Voltage[0] = (ADCValue[0] / (1024.0)) * 5.0; //For accurate measurements, divide the ADC value by the ADC maximum value and multiply by the resultant ADV value.

// Read PIN 1
    digitalWrite(A_pin, 0);
    digitalWrite(B_pin, 1);
    digitalWrite(C_pin, 0);

    ADCValue[1] = analogRead(A0);
    Voltage[1] = (ADCValue[1] / (1024.0)) * 5.0; //For accurate measurements, divide the ADC value by the ADC maximum value and multiply by the resultant ADV value.

// Read PIN 2
    digitalWrite(A_pin, 0);
    digitalWrite(B_pin, 1);
    digitalWrite(C_pin, 0);

    ADCValue[2] = analogRead(A0);
    Voltage[2] = (ADCValue[2] / (1024.0)) * 5.0; //For accurate measurements, divide the ADC value by the ADC maximum value and multiply by the resultant ADV value.

// Read PIN 3
    digitalWrite(A_pin, 1);
    digitalWrite(B_pin, 1);
    digitalWrite(C_pin, 0);

    ADCValue[3] = analogRead(A0);
    Voltage[3] = (ADCValue[3] / (1024.0)) * 5.0; //For accurate measurements, divide the ADC value by the ADC maximum value and multiply by the resultant ADV value.

// Read PIN 4
    digitalWrite(A_pin, 0);
    digitalWrite(B_pin, 0);
    digitalWrite(C_pin, 1);

    ADCValue[4] = analogRead(A0);
    Voltage[4] = (ADCValue[4] / (1024.0)) * 5.0; //For accurate measurements, divide the ADC value by the ADC maximum value and multiply by the resultant ADV value.

// Read PIN 5
    digitalWrite(A_pin, 1);
    digitalWrite(B_pin, 0);
    digitalWrite(C_pin, 1);

    ADCValue[5] = analogRead(A0);
    Voltage[5] = (ADCValue[5] / (1024.0)) * 5.0; //For accurate measurements, divide the ADC value by the ADC maximum value and multiply by the resultant ADV value.

// Read PIN 6
    digitalWrite(A_pin, 0);
    digitalWrite(B_pin, 1);
    digitalWrite(C_pin, 1);

    ADCValue[6] = analogRead(A0);
    Voltage[6] = (ADCValue[6] / (1024.0)) * 5.0; //For accurate measurements, divide the ADC value by the ADC maximum value and multiply by the resultant ADV value.

// Read PIN 7
    digitalWrite(A_pin, 1);
    digitalWrite(B_pin, 1);
    digitalWrite(C_pin, 1);

    ADCValue[7] = analogRead(A0);
    Voltage[7] = (ADCValue[7] / (1024.0)) * 5.0; //For accurate measurements, divide the ADC value by the ADC maximum value and multiply by the resultant ADV value.
    
//---------------------------END of 4051 Multiplexer section--------------------------------
//





//---------- Mesure du cournant ------------------------------
//  int courant_mot = analogRead(A0);      // read the input on analog pin 0:
 
////courant_mot=map(courant_mot,);
//
//    while (sample_count < NUM_SAMPLES) {
//        sum1 += analogRead(A0);
//        sample_count++;
////        delay(1);
//    }
////   courant_mot = (float(sum1) / float(NUM_SAMPLES) );
// 
//   sample_count = 0;
 //---------- Fin Mesure du cournant ------------------------------
   
//---------- Mesure de la tension Power supply ------------------------------   
// Mesure de la tension
// take a number of analog samples and add them up
    while (sample_count < NUM_SAMPLES) {
        sum2 += analogRead(A1);
        sample_count++;
//       delay(1);
    }
    sample_count = 0;
//---------- Fin Mesure de la tension du moteur ------------------------------  

//---------- Mesure de la tension du Heatbed ------------------------------   
// Mesure de la tension
// take a number of analog samples and add them up
    while (sample_count < NUM_SAMPLES) {
        sum3 += analogRead(A2);
        sample_count++;
//       delay(1);
    }
    sample_count = 0;
//---------- Fin Mesure de la tension la génératrice ------------------------------  


//---------- Mesure de la tension de  la BUSE ------------------------------   
// Mesure de la tension
// take a number of analog samples and add them up
    while (sample_count < NUM_SAMPLES) {
        sum1 += analogRead(A3);
        sample_count++;
//       delay(1);
    }
    sample_count = 0;
//---------- Fin Mesure de la tension de la BUSE ------------------------------  


 //---------- Mesure de température ------------------------------
  
  temperture_sensors.requestTemperatures(); // Send the command to get temperatures

 //---------- Fin Mesure de température ------------------------------




  
//---------------------------Calibrages et décalages--------------------------------------



//  float(RawValue) = sum1 / NUM_SAMPLES ;
// AmpsVoltage = ((float)RawValue / (float)1023) * (float)4.5; // Gets you mV
// Amps = (((float)AmpsVoltage - (float)ACSoffset) / (float)mVperAmp);
//  Amps = 0.074 * (RawValue - 512 ); // http://embedded-lab.com/blog/?p=4529
//     sum1 = 0; 
//    // calculate the voltage
//    // use 5.0 for a 5.0V ADC reference voltage
//    // 5.015V is the calibrated reference voltage
    Tension_supply = ((float)sum2 / (float)NUM_SAMPLES * 5.000) / 1023.0;
    // voltage multiplied by 11.0 when using voltage divider that
    // divides by 11.0
    // 11.0 is the calibrated voltage dividevalue
    Tension_supply *=11.0;
    sum2 = 0;
//    
    Tension_Heatbed = ((float)sum3 / (float)NUM_SAMPLES * 5.000) / 1023.0;
    // voltage multiplied by 4.3 when using voltage divider that
    // divides by 4.3  
    // 4.450 is the calibrated voltage dividevalue
    Tension_Heatbed *=11.0;
    sum3 = 0;


    Tension_Hotend = ((float)sum1 / (float)NUM_SAMPLES * 5.000) / 1023.0;
    // voltage multiplied by 4.3 when using voltage divider that
    // divides by 4.3  
    // 4.450 is the calibrated voltage dividevalue
    Tension_Hotend *=11.0;
    sum1 = 0;




//---------------------Trasmission des Données Via port série--------------------------------


//  Serial.print(" T_int "); 
  Serial.print(temperture_sensors.getTempC(Probe01));
  Serial.print("\t");
//  Serial.print(" T_ext "); 
  Serial.print(temperture_sensors.getTempC(Probe02));
  Serial.print("\t");
//  Serial.print(" T_H_bg "); 
  Serial.print(temperture_sensors.getTempC(Probe03));
  Serial.print("\t");

//for (count=0; count<=7; count++) {
//    Serial.print("\t");
//    Serial.print(Voltage[count],2);
//}

//  Serial.print(Vcc);
//  Serial.print("\t"); 
//  Serial.print(ADCValue[0]);
//  Serial.print("\t");

  Serial.print(Voltage[0],2);
  Serial.print("\t");
  Serial.print(Voltage[1],2);
  Serial.print("\t");
  Serial.print(Voltage[2],2);
  Serial.print("\t");
  Serial.print(Voltage[3],2);
  Serial.print("\t");
  Serial.print(Voltage[4],2);
  Serial.print("\t");
  Serial.print(Voltage[5],2);
  Serial.print("\t");
  Serial.print(Voltage[6],2);
  Serial.print("\t");
  Serial.print(Voltage[7],2);

  Serial.print("\t"); 
//  Serial.print(" V_moteur ");  
  Serial.print(Tension_supply,2);
  
  Serial.print("\t"); 
//  Serial.print(" V_moteur ");  
  Serial.print(Tension_Heatbed,2);

  Serial.print("\t"); 
//  Serial.print(" V_moteur ");  
  Serial.print(Tension_Hotend,2);

  Serial.print("\t"); 
  Serial.print(digitalRead(D8));

  Serial.print("\t"); 
  Serial.print(digitalRead(D10));

  Serial.print("\t"); 
  Serial.print(digitalRead(E_EN));

  Serial.print("\t"); 
  Serial.print(digitalRead(X_EN));

  Serial.print("\t"); 
  Serial.print(digitalRead(Y_EN));
  
  Serial.println(" "); 
  
  
  delay(50);        // delay in between reads for stability

  
}
//*****************************************************************************************
// Fin Boocle Principale


long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV --- 1024*1.1*1000 = 1126400
  return result;
}
