#include <INA3221.h>

#include <Wire.h>

enum { INA_260 , INA_3221 }; 
enum {MCP_9600, MCP_9601};
int INAxxx = INA_3221;    
int MCPxxxx = MCP_9601; 

long TestTime=0;

#include <INA3221.h>
#define R_SHUNT 10
#include "Seeed_MCP9600.h"
// Set I2C address to 0x41 (A0 pin -> VCC)
INA3221 ina_0(INA3221_ADDR40_GND);
INA3221 ina_1(INA3221_ADDR41_VCC);
INA3221 ina_2(INA3221_ADDR42_SDA);
INA3221 ina_3(INA3221_ADDR43_SCL);

#define SERIAL_SPEED     115200
#define DEFAUT_TIME_MEASURE 500

enum {PIN_LB = A3,
     PIN_HB = A5,
     PIN_DRL = A4,
     PIN_PL = A2,
     PIN_TI = A6,
     PIN_KL1 = A0,
     PIN_KL2 = A1,
     }; 
     
int tabRelais[7];
bool tiOn =false; 
int tiMode = 0 ; //0 = Blinking ; 1 = Continuous
int tiPeriode = 333; //330 EU ; 500 US  
int tiState = 0; 


int  CapteurTempAdr[8];
int  CapteurVoltAdr[12];



MCP9600 TempSensor[8];



float TempValue[8];

int nDevicesVolt = 0 ;
int nDevicesTemp = 0 ;
String ListDevices = "";

String Commande = "";
bool measure = false ;
long tMeas = DEFAUT_TIME_MEASURE;
long lastTime ;
long lastTimeTI ;
long lastTimeTemp ;


void setup() {

  Wire.begin();
  
tabRelais[0]= PIN_KL1;
tabRelais[1]= PIN_KL2;
tabRelais[2]= PIN_LB;
tabRelais[3]= PIN_HB;
tabRelais[4]= PIN_DRL;
tabRelais[5]= PIN_PL;
tabRelais[6]= PIN_TI;

defineINA();
defineMCP();


 
 
  ScanI2c();
  
pinMode(A0, OUTPUT);
pinMode(A1, OUTPUT);
pinMode(A2, OUTPUT);
pinMode(A3, OUTPUT);
pinMode(A4, OUTPUT);
pinMode(A5, OUTPUT);
pinMode(A6, OUTPUT);


  ListDevices = String(";") + nDevicesTemp + String("T;") + nDevicesVolt + String("V;")  ;
  Serial.begin(SERIAL_SPEED);
  Serial.setTimeout(50);

}

void loop() {

  if (Serial.available()) {
    Commande = Serial.readString();
    if (!Commande.equals(""))
    {
      if (Commande.indexOf("IDN") != -1)
      {
        Serial.print("ARDUINO");
        Serial.print(ListDevices);
        Serial.println("DATALOGGER");
      }
      else if (Commande.indexOf("RL_END") != -1)
      {
        String val = Commande.substring(Commande.indexOf("RL_BEGIN")+8,Commande.indexOf("RL_END"));

        for (int j =0; j<6;j++){
 
            if (val[j]=='0'){
                digitalWrite(tabRelais[j+1],LOW);
                if (j==5){
                    tiOn=false; 
                }
            }
            else{
                digitalWrite(tabRelais[j+1],HIGH);
                if (j==5){
                    tiOn=true; 
                }
            }
        }
        if (val[0]=='0'){
                digitalWrite(tabRelais[0],LOW);
            }
            else{
                digitalWrite(tabRelais[0],HIGH);
            }
        }
      else if (Commande.indexOf("STOP") != -1)
      {
        measure = false ;
        Serial.println("OK");
      }
      else if (Commande.indexOf("START") != -1)
      {
        measure = true ;
        lastTime = micros();
        TestTime = 0;
      }
      else if (Commande.indexOf("20ms") != -1)
      {
        tMeas = 20000;
        Serial.println("OK");
      }
      else if (Commande.indexOf("40ms") != -1)
      {
        tMeas = 40000;
        Serial.println("OK");
      }
      else if (Commande.indexOf("60ms") != -1)
      {
        tMeas = 60000;
        Serial.println("OK");
      }
      else if (Commande.indexOf("1s") != -1)
      {
        tMeas = 1000000;
        Serial.println("OK");
      }
      else if (Commande.indexOf("5s") != -1)
      {
        tMeas = 5000000;
        Serial.println("OK");
      }
      else if (Commande.indexOf("2s") != -1)
      {
        tMeas = 2000000;
        Serial.println("OK");
      }
      else if (Commande.indexOf("10s") != -1)
      {
        tMeas = 10000000;
        Serial.println("OK");
      }
      else if(Commande.indexOf("READ") != -1)
      {
        //ReadInfo();
      }
      else if(Commande.indexOf("TI_EU") != -1)
      {
        tiPeriode=333;
        tiMode = 0;

      }
      else if(Commande.indexOf("TI_US") != -1)
      {
        tiPeriode=500;
        tiMode = 0;

      }
      else if(Commande.indexOf("TI_CONTINUOUS") != -1)
      {
        tiMode = 1;
        if(tiOn)
        {
             digitalWrite(tabRelais[PIN_TI],HIGH);
        }
      }
      
      Commande = "";
    }
  }
  if (measure) {
    
    
    if (micros() - tMeas> lastTime )
    {
      TestTime+=(tMeas/1000);
      lastTime = micros();
      Serial.print(TestTime);
    Serial.print(";");
        for (int i = 0; i < nDevicesTemp; i++) {
        Serial.print(TempValue[i], 3);
        Serial.print(";");
      }
     
                Serial.print(ina_0.getCurrent(INA3221_CH1) * 1000, 3);
    Serial.print(";");
        Serial.print(ina_0.getVoltage(INA3221_CH2), 3); 
    Serial.print(";");
        Serial.print(ina_0.getCurrent(INA3221_CH2) * 1000, 3);
    Serial.print(";");
        Serial.print(ina_0.getVoltage(INA3221_CH3), 3);
    Serial.print(";");
        Serial.print(ina_0.getCurrent(INA3221_CH3) * 1000, 3);
    Serial.print(";");
    Serial.print(ina_1.getVoltage(INA3221_CH1), 3);
    Serial.print(";");
    Serial.print(ina_1.getCurrent(INA3221_CH1) * 1000, 3);
    Serial.print(";");
        Serial.print(ina_1.getVoltage(INA3221_CH2), 3);
    Serial.print(";");
        Serial.print(ina_1.getCurrent(INA3221_CH2) * 1000, 3);
    Serial.print(";");
        Serial.print(ina_1.getVoltage(INA3221_CH3), 3);
    Serial.print(";");
        Serial.print(ina_1.getCurrent(INA3221_CH3) * 1000, 3);
    Serial.print(";");
        Serial.print(ina_2.getVoltage(INA3221_CH1), 3);
    Serial.print(";");
        Serial.print(ina_2.getCurrent(INA3221_CH1) * 1000, 3);
    Serial.print(";");
        Serial.print(ina_2.getVoltage(INA3221_CH2), 3);
    Serial.print(";");
    Serial.print(ina_2.getCurrent(INA3221_CH2) * 1000, 3);
    Serial.print(";");
        Serial.print(ina_2.getVoltage(INA3221_CH3), 3);
    Serial.print(";");
        Serial.print(ina_2.getCurrent(INA3221_CH3) * 1000, 3);
    Serial.print(";");
    Serial.print(ina_3.getVoltage(INA3221_CH1), 3);
    Serial.print(";");
        Serial.print(ina_3.getCurrent(INA3221_CH1) * 1000, 3);
    Serial.print(";");
        Serial.print(ina_3.getVoltage(INA3221_CH2), 3);
    Serial.print(";");
    Serial.print(ina_3.getCurrent(INA3221_CH2) * 1000, 3);
    Serial.print(";");
        Serial.print(ina_3.getVoltage(INA3221_CH3), 3);
    Serial.print(";");
        Serial.print(ina_3.getCurrent(INA3221_CH3) * 1000, 3);
    Serial.println(";");
        
        
        
    }
    if (micros() > lastTimeTemp + 1000000) {
      lastTimeTemp = micros();
      
      for (int i = 0; i < nDevicesTemp; i++) {
        get_temperature(&TempValue[i], TempSensor[i]);
      }
    }
  }

    if (tiOn & !tiMode){
        if (tiState==1) 
        {
            if (millis() > lastTimeTI + tiPeriode)
            {
              lastTimeTI = millis();
              digitalWrite(tabRelais[6],LOW);
              tiState=0;
              
            }
        }
        else{
             if (millis() > lastTimeTI + (666-tiPeriode))
            {
              lastTimeTI = millis();
              digitalWrite(tabRelais[6],HIGH);
              tiState=1;
            }
        }
        
    }

}

    


void ScanI2c () {
  byte error, address;

  //SCAN VOLTAGE/CURRENT SENSOR
  
  for (address = 64; address < 80; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    delay(50);
    if (error == 0 || error == 4 )
    {
      Serial.println(address);      
      
      CapteurVoltAdr[nDevicesVolt] = address;
      nDevicesVolt++;
    }
  }
  nDevicesVolt=nDevicesVolt*3;

  //SCAN TEMPERATURE SENSOR
  #if MCPxxxx == MCP_9600
  for (address = 96; address < 104; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    delay(50);
    if (error == 0)
    {
        Serial.println(address);
      CapteurTempAdr[nDevicesTemp] = address;
      TempSensor[nDevicesTemp].init(THER_TYPE_K);
      TempValue[nDevicesTemp] = (0);
      nDevicesTemp++;
    }
  }
  #else
  for (address = 96; address < 104; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    delay(50);
    if (error == 0)
    {
        Serial.println(address);
      CapteurTempAdr[nDevicesTemp] = address;
      TempSensor[nDevicesTemp].setThermocoupleType(MCP9600_TYPE_K);
      TempSensor[nDevicesTemp].setFilterCoefficient(3);
      TempSensor[nDevicesTemp].setADCresolution(MCP9600_ADCRESOLUTION_18);
      TempValue[nDevicesTemp] = (0);
      nDevicesTemp++;
    }
  }
  #endif 
  for (int i = 0 ; i < nDevicesVolt; i++)
  {
    //Serial.print("Voltage Sensor address : ");
    //Serial.println(CapteurVoltAdr[i]);
  }
  for (int i = 0 ; i < nDevicesTemp; i++)
  {
    //Serial.print("Temp Sensor address : ");
    //Serial.println(CapteurTempAdr[i]);
  }
}


/*void InitINA() {
  for (int i = 0 ; i < nDevicesVolt; i++)
  {
    VoltageSensor[i].setAveragingCount(INA260_AveragingCount(INA260_COUNT_128));
    VoltageSensor[i].setCurrentConversionTime(INA260_ConversionTime(INA260_TIME_140_us));
    VoltageSensor[i].setVoltageConversionTime(INA260_ConversionTime(INA260_TIME_140_us));
  } 
}*/


err_t sensor_basic_config(MCP9600 sensor) {
  err_t ret = NO_ERROR;
  CHECK_RESULT(ret, sensor.set_filt_coefficients(FILT_MID));
  CHECK_RESULT(ret, sensor.set_cold_junc_resolution(COLD_JUNC_RESOLUTION_0_625));
  CHECK_RESULT(ret, sensor.set_ADC_meas_resolution(ADC_18BIT_RESOLUTION));
  CHECK_RESULT(ret, sensor.set_burst_mode_samp(BURST_32_SAMPLE));
  CHECK_RESULT(ret, sensor.set_sensor_mode(NORMAL_OPERATION));
  return ret;
}

float get_temperature(float* value, MCP9600 sensor) {
  err_t ret = NO_ERROR;
  float hot_junc = 0;
  float junc_delta = 0;
  float cold_junc = 0;
  CHECK_RESULT(ret, sensor.read_hot_junc(&hot_junc));
  CHECK_RESULT(ret, sensor.read_junc_temp_delta(&junc_delta));

  CHECK_RESULT(ret, sensor.read_cold_junc(&cold_junc));
  *value = hot_junc;



  return *value;
}


void defineINA(void){
    ina_0.begin(&Wire);
    ina_0.reset();
    ina_0.setShuntRes(R_SHUNT, R_SHUNT, R_SHUNT);
    ina_1.begin(&Wire);
    ina_1.reset();
    ina_1.setShuntRes(R_SHUNT, R_SHUNT, R_SHUNT);
    ina_2.begin(&Wire);
    ina_2.reset();
    ina_2.setShuntRes(R_SHUNT, R_SHUNT, R_SHUNT);
    ina_3.begin(&Wire);
    ina_3.reset();
    ina_3.setShuntRes(R_SHUNT, R_SHUNT, R_SHUNT);  

}

void defineMCP(void){
    MCP9600 mcp9600_1;
    MCP9600 mcp9600_2;
    MCP9600 mcp9600_3;
    MCP9600 mcp9600_4;
    MCP9600 mcp9600_5;
    MCP9600 mcp9600_6;
    MCP9600 mcp9600_7;
    MCP9600 mcp9600_8;

    TempSensor[0] = mcp9600_1;
    TempSensor[1] = mcp9600_2;
    TempSensor[2] = mcp9600_3;
    TempSensor[3] = mcp9600_4;
    TempSensor[4] = mcp9600_5;
    TempSensor[5] = mcp9600_6;
    TempSensor[6] = mcp9600_7;
    TempSensor[7] = mcp9600_8;

    
  
}
