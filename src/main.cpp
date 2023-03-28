#include <Arduino.h>
#include <FreeRTOS.h>
#include "BluetoothSerial.h"
#include <math.h>
#include <fis_header.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;
static TaskHandle_t task_3 = NULL;
static TaskHandle_t task_4 = NULL;
static TaskHandle_t task_5 = NULL;
static TaskHandle_t task_6 = NULL;




#define SOUND_SPEED 0.034
unsigned long actualTime=0;
unsigned long lastTime=0;
unsigned long TimeDiffer=0;
float R=0.062/2.0;
float distancePerCount=(2.0*PI*R)/1920.0; //dystans na jeden impuls
int deltaLeftCounter=0;
int deltaRightCounter=0;
float deltaDistance=0.0;
float deltaX=0.0;
float deltaY=0.0;
float deltaAngle=0.0;
int OldDeltaLeftCounter=0;
int OldDeltaRightCounter=0;
float posX=0.0;
float posY=0.0;
float posAngle=0.0; //-PI //0
int leftCounter=0;
int rightCounter=0;
volatile int interruptCounter;
int totalInterruptCounter;
volatile int interruptCounter1;
int totalInterruptCounter1;
volatile int interruptCounter2;
int totalInterruptCounter2;
volatile int interruptCounter3;
int totalInterruptCounter3;
int speedMOTOR;
int speedMOTOR_L;
int speedMOTOR_R;
float Xg=0.0;
float Yg=0.0;
float z;
float psi;
float VelL;
float VelR;
float P=0.0;
char posXString[8];
char posYString[8];
int curr_button;
int prev_button;
int menuPos=0;

const uint8_t Button = 26;
const uint8_t A1A = 12;//silnik A
const uint8_t A1B = 13;//-.-
const uint8_t B1A = 2;//silnikB
const uint8_t B1B = 4;//-.-
const uint8_t ECHO = 14;
const uint8_t TRIG = 21;
const uint8_t TRIG2 = 22;
const uint8_t ECHO2 = 27;
const uint8_t TRIG3 = 34;
const uint8_t ECHO3 = 15;
const uint8_t LH_ENCODER_A = 25;
const uint8_t LH_ENCODER_B = 33;
const uint8_t RH_ENCODER_A = 35;
const uint8_t RH_ENCODER_B = 32;

char Direction;
long time_HC04;
float distanceHC_CM;
long time_HC04_R;
float distanceHC_CM_R;
long time_HC04_L;
float distanceHC_CM_L;
volatile signed long counterL=0;
volatile signed long counterR=0;

U8G2_PCD8544_84X48_1_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 18, /* data=*/ 23, /* cs=*/ 5, /* dc=*/ 19, /* reset=*/ 17);	// Nokia 5110 Display
//U8G2_PCD8544_84X48_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 5, /* dc=*/ 19, /* reset=*/ 17);	
BluetoothSerial SerialBT;


//  SSID/Password for WiFi
//const char* ssid = "Xiaomi_188E";
//const char* password = "WPISAC";// CREDENTIALS

const char* ssid = "PozdrawiamOsoby";
const char* password = "WPISAC";// CREDENTIALS


// MQTT Broker IP address
//const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_server = "broker.mqttdashboard.com";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[20];
int value = 0;

 // MODUL WIFI
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32-ROBO","WPISAC","WPISAC")) { // CREDENTIALS
      Serial.println("connected");
    } else {
      delay(800); // default 1000
    }
  }
}



// FIS WSTEPNE 
const int fis_gcI = 3;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 2;
// Number of rules to the fuzzy inference system
const int fis_gcR = 8;
char Mode;
FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    if (t1<0)
    t1=0;
    return (FIS_TYPE) t1;
}

FIS_TYPE fis_prod(FIS_TYPE a, FIS_TYPE b)
{
    return (a * b);
}

FIS_TYPE fis_probor(FIS_TYPE a, FIS_TYPE b)
{
    return (a + b - (a * b));
}

FIS_TYPE fis_sum(FIS_TYPE a, FIS_TYPE b)
{
    return (a + b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;
    if (size == 0) return ret;
    if (size == 1) return array[0];
    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 2, 2, 2 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 3, 3 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 0, 0, 33 };
FIS_TYPE fis_gMFI0Coeff2[] = { 4, 380, 2000 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2 };
FIS_TYPE fis_gMFI1Coeff1[] = { 0, 0, 33 };
FIS_TYPE fis_gMFI1Coeff2[] = { 4, 380, 2000 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2 };
FIS_TYPE fis_gMFI2Coeff1[] = { 0, 0, 33 };
FIS_TYPE fis_gMFI2Coeff2[] = { 4, 380, 2000 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0, 0, -10 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0, 0, 0, 3 };
FIS_TYPE fis_gMFO0Coeff3[] = { 0, 0, 0, 10 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3 };
FIS_TYPE fis_gMFO1Coeff1[] = { 0, 0, 0, -10 };
FIS_TYPE fis_gMFO1Coeff2[] = { 0, 0, 0, 3 };
FIS_TYPE fis_gMFO1Coeff3[] = { 0, 0, 0, 10 };
FIS_TYPE* fis_gMFO1Coeff[] = { fis_gMFO1Coeff1, fis_gMFO1Coeff2, fis_gMFO1Coeff3 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff, fis_gMFO1Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0 };
int fis_gMFI1[] = { 0, 0 };
int fis_gMFI2[] = { 0, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2};

// Output membership function set

int* fis_gMFO[] = {};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 2, 2, 2 };
int fis_gRI1[] = { 2, 2, 1 };
int fis_gRI2[] = { 2, 1, 2 };
int fis_gRI3[] = { 2, 1, 1 };
int fis_gRI4[] = { 1, 2, 2 };
int fis_gRI5[] = { 1, 2, 1 };
int fis_gRI6[] = { 1, 1, 2 };
int fis_gRI7[] = { 1, 1, 1 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7 };

// Rule Outputs
int fis_gRO0[] = { 3, 3 };
int fis_gRO1[] = { 1, 3 };
int fis_gRO2[] = { 1, 3 };
int fis_gRO3[] = { 1, 3 };
int fis_gRO4[] = { 3, 1 };
int fis_gRO5[] = { 3, 3 };
int fis_gRO6[] = { 3, 1 };
int fis_gRO7[] = { 1, 3 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 400, 400, 400 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0, 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 1, 1 };

//***********************************************************************
// Fuzzy Inference System  //OMIJANIE PRZESZKOD                                      
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyOutput1[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, fuzzyOutput1, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = 1;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = 0;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            FIS_TYPE sWI = 0.0;
            for (j = 0; j < fis_gOMFCount[o]; ++j)
            {
                fuzzyOutput[o][j] = fis_gMFOCoeff[o][j][fis_gcI];
                for (i = 0; i < fis_gcI; ++i)
                {
                    fuzzyOutput[o][j] += g_fisInput[i] * fis_gMFOCoeff[o][j][i];
                }
            }

            for (r = 0; r < fis_gcR; ++r)
            {
                index = fis_gRO[r][o] - 1;
                sWI += fuzzyFires[r] * fuzzyOutput[o][index];
            }

            g_fisOutput[o] = sWI / sW;
        }
    }
}


// Number of inputs to the fuzzy inference system
const int fis_gcI_G = 2;
// Number of outputs to the fuzzy inference system
const int fis_gcO_G= 2;
// Number of rules to the fuzzy inference system
const int fis_gcR_G = 6;

FIS_TYPE g_fisInput_G[fis_gcI];
FIS_TYPE g_fisOutput_G[fis_gcO];

// Setup routine runs once when you press reset:

// Loop routine runs over and over again forever:



// Gaussian Member Function
FIS_TYPE fis_gaussmf_G(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE s = p[0], c = p[1];
    FIS_TYPE t = (x - c) / s;
    return exp(-(t * t) / 2);
}

// Pointers to the implementations of member functions
_FIS_MF fis_gMF_G[] =
{
    fis_gaussmf_G
};

// Count of member function for each Input
int fis_gIMFCount_G[] = { 2, 3 };

// Count of member function for each Output
int fis_gOMFCount_G[] = { 3, 3 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1_G[] = { 0.098063459, -0.0165 };
FIS_TYPE fis_gMFI0Coeff2_G[] = { 2.0864227464, 5.02 };
FIS_TYPE* fis_gMFI0Coeff_G[] = { fis_gMFI0Coeff1_G, fis_gMFI0Coeff2_G };
FIS_TYPE fis_gMFI1Coeff1_G[] = { 2.36, -4.0 };
FIS_TYPE fis_gMFI1Coeff2_G[] = { 2.36, 4.0 };
FIS_TYPE fis_gMFI1Coeff3_G[] = { 0.0988, 0.0212 }; //0.0629432835570066 //0.2644 // 1.2031
FIS_TYPE* fis_gMFI1Coeff_G[] = { fis_gMFI1Coeff1_G, fis_gMFI1Coeff2_G, fis_gMFI1Coeff3_G };
FIS_TYPE** fis_gMFICoeff_G[] = { fis_gMFI0Coeff_G, fis_gMFI1Coeff_G };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1_G[] = { 0, 0, -10 }; //-10
FIS_TYPE fis_gMFO0Coeff2_G[] = { 0, 0, 10 }; // +10
FIS_TYPE fis_gMFO0Coeff3_G[] = { 0, 0, 0 };
FIS_TYPE* fis_gMFO0Coeff_G[] = { fis_gMFO0Coeff1_G, fis_gMFO0Coeff2_G, fis_gMFO0Coeff3_G };
FIS_TYPE fis_gMFO1Coeff1_G[] = { 0, 0, -8.5 }; // - 10 //6.5
FIS_TYPE fis_gMFO1Coeff2_G[] = { 0, 0, 10 }; //+ 10
FIS_TYPE fis_gMFO1Coeff3_G[] = { 0, 0, 0 };
FIS_TYPE* fis_gMFO1Coeff_G[] = { fis_gMFO1Coeff1_G, fis_gMFO1Coeff2_G, fis_gMFO1Coeff3_G };
FIS_TYPE** fis_gMFOCoeff_G[] = { fis_gMFO0Coeff_G, fis_gMFO1Coeff_G };

// Input membership function set
int fis_gMFI0_G[] = { 0, 0 };
int fis_gMFI1_G[] = { 0, 0, 0 };
int* fis_gMFI_G[] = { fis_gMFI0_G, fis_gMFI1_G};

// Output membership function set

int* fis_gMFO_G[] = {};

// Rule Weights
FIS_TYPE fis_gRWeight_G[] = { 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType_G[] = { 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0_G[] = { 1, 1 };
int fis_gRI1_G[] = { 1, 3 };
int fis_gRI2_G[] = { 1, 2 };
int fis_gRI3_G[] = { 2, 1 };
int fis_gRI4_G[] = { 2, 3 };
int fis_gRI5_G[] = { 2, 2 };
int* fis_gRI_G[] = { fis_gRI0_G, fis_gRI1_G, fis_gRI2_G, fis_gRI3_G, fis_gRI4_G, fis_gRI5_G };

// Rule Outputs
int fis_gRO0_G[] = { 1, 2 };
int fis_gRO1_G[] = { 3, 3 };
int fis_gRO2_G[] = { 2, 1 };
int fis_gRO3_G[] = { 1, 2 };
int fis_gRO4_G[] = { 2, 2 };
int fis_gRO5_G[] = { 2, 1 };
int* fis_gRO_G[] = { fis_gRO0_G, fis_gRO1_G, fis_gRO2_G, fis_gRO3_G, fis_gRO4_G, fis_gRO5_G };

// Input range Min
FIS_TYPE fis_gIMin_G[] = { 0, -4 };

// Input range Max
FIS_TYPE fis_gIMax_G[] = { 5, 4 };

// Output range Min
FIS_TYPE fis_gOMin_G[] = { 0, 0 };

// Output range Max
FIS_TYPE fis_gOMax_G[] = { 1, 1 };


//***********************************************************************
// Fuzzy Inference System/// DO CELU FIS 
//***********************************************************************
void fis_evaluate_G()
{
    FIS_TYPE fuzzyInput0_G[] = { 0, 0 };
    FIS_TYPE fuzzyInput1_G[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput_G[fis_gcI] = { fuzzyInput0_G, fuzzyInput1_G, };
    FIS_TYPE fuzzyOutput0_G[] = { 0, 0, 0 };
    FIS_TYPE fuzzyOutput1_G[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyOutput_G[fis_gcO_G] = { fuzzyOutput0_G, fuzzyOutput1_G, };
    FIS_TYPE fuzzyRules_G[fis_gcR_G] = { 0 };
    FIS_TYPE fuzzyFires_G[fis_gcR_G] = { 0 };
    FIS_TYPE* fuzzyRuleSet_G[] = { fuzzyRules_G, fuzzyFires_G };
    FIS_TYPE sW_G = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI_G; ++i)
    {
        for (j = 0; j < fis_gIMFCount_G[i]; ++j)
        {
            fuzzyInput_G[i][j] =
                (fis_gMF_G[fis_gMFI_G[i][j]])(g_fisInput_G[i], fis_gMFICoeff_G[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR_G; ++r)
    {
        if (fis_gRType_G[r] == 1)
        {
            fuzzyFires_G[r] = 1;
            for (i = 0; i < fis_gcI_G; ++i)
            {
                index = fis_gRI_G[r][i];
                if (index > 0)
                    fuzzyFires_G[r] = fis_prod(fuzzyFires_G[r], fuzzyInput_G[i][index - 1]);
                else if (index < 0)
                    fuzzyFires_G[r] = fis_prod(fuzzyFires_G[r], 1 - fuzzyInput_G[i][-index - 1]);
                else
                    fuzzyFires_G[r] = fis_prod(fuzzyFires_G[r], 1);
            }
        }
        else
        {
            fuzzyFires_G[r] = 0;
            for (i = 0; i < fis_gcI_G; ++i)
            {
                index = fis_gRI_G[r][i];
                if (index > 0)
                    fuzzyFires_G[r] = fis_probor(fuzzyFires_G[r], fuzzyInput_G[i][index - 1]);
                else if (index < 0)
                    fuzzyFires_G[r] = fis_probor(fuzzyFires_G[r], 1 - fuzzyInput_G[i][-index - 1]);
                else
                    fuzzyFires_G[r] = fis_probor(fuzzyFires_G[r], 0);
            }
        }

        fuzzyFires_G[r] = fis_gRWeight_G[r] * fuzzyFires_G[r];
        sW_G += fuzzyFires_G[r];
    }

    if (sW_G == 0)
    {
        for (o = 0; o < fis_gcO_G; ++o)
        {
            g_fisOutput_G[o] = ((fis_gOMax_G[o] + fis_gOMin_G[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO_G; ++o)
        {
            FIS_TYPE sWI_G = 0.0;
            for (j = 0; j < fis_gOMFCount_G[o]; ++j)
            {
                fuzzyOutput_G[o][j] = fis_gMFOCoeff_G[o][j][fis_gcI_G];
                for (i = 0; i < fis_gcI_G; ++i)
                {
                    fuzzyOutput_G[o][j] += g_fisInput_G[i] * fis_gMFOCoeff_G[o][j][i];
                }
            }

            for (r = 0; r < fis_gcR_G; ++r)
            {
                index = fis_gRO_G[r][o] - 1;
                sWI_G += fuzzyFires_G[r] * fuzzyOutput_G[o][index];
            }

            g_fisOutput_G[o] = sWI_G / sW_G;
        }
    }
}

float minimum(float a, float b, float c){
  float min = a;                                 //// było  50 i a
  if (b<=min)
  min=b;
  if (c<=min)
  min=c;

  return min;
}


//ENKODERY
//LEFT
void pulseL() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      counterL++; //
    } else {
      counterL--;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      counterL--; //--
    } else {
      counterL++;
    }
  }
}
//RIGHT
void pulseR() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      counterR--;
    } else {
      counterR++;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      counterR++;
    } else {
      counterR--;
    }
  }
}

///STEROWANIE///
void MoveForward1(int speedM_L,int speedM_R){
      if (speedM_L <=100)
      speedM_L=0;
      if (speedM_R <=100)
      speedM_R=0;
    ledcWrite(0, 0);
    ledcWrite(1, speedM_L);
    ledcWrite(2, 0);
    ledcWrite(3, speedM_R);
}

void MoveBackward1(int speedM){

  if (speedM <=100)
  speedM=0;
    ledcWrite(0, speedM);
    ledcWrite(1, 0);
    ledcWrite(2, speedM);
    ledcWrite(3, 0);
}

void MoveRight1(int speedM_L,int speedM_R){

      if (speedM_L <=100)
      speedM_L=0;
      if (speedM_R <=100)
      speedM_R=0;
      ledcWrite(0, 0);
      ledcWrite(1, speedM_L);
      ledcWrite(2, speedM_R);
      ledcWrite(3, 0);
    }       

void MoveLeft1(int speedM_L,int speedM_R){

      if (speedM_L <=100)
      speedM_L=0;
      if (speedM_R <=100)
      speedM_R=0;
      ledcWrite(0, speedM_L);
      ledcWrite(1, 0);
      ledcWrite(2, 0);
      ledcWrite(3, speedM_R);
    }       

char Position[5];
byte pos=0;

void Blutacz(void *parameters){
  while(1){
  if (SerialBT.available()){  
    Position[pos] =SerialBT.read();
    if(Position[pos]=='#'){
      Position[pos]=0;
      pos=0;
    }
    else{
      if (pos<4) 
      pos++;
    }
    Direction=Position[0];
  }
  vTaskDelay(30/portTICK_PERIOD_MS);
  }
}    


///ODOMETRIA///
void Odometry(){
leftCounter=counterL;
rightCounter=counterR;
deltaLeftCounter=counterL-OldDeltaLeftCounter;
deltaRightCounter=counterR-OldDeltaRightCounter;
deltaDistance=((deltaLeftCounter+deltaRightCounter)/2)*distancePerCount;
deltaX = deltaDistance * (float)cos(posAngle);
deltaY = deltaDistance * (float)sin(posAngle);
//if (abs(deltaRightCounter-deltaLeftCounter)>50)
deltaAngle = ((deltaRightCounter-deltaLeftCounter)*(0.00081812308)); 
//deltaAngle = ((deltaRightCounter*distancePerCount-deltaLeftCounter*distancePerCount)/145);
//0.0175 Radiany
posX += deltaX;
posY += deltaY;
posAngle += deltaAngle;

if (posAngle > 2*PI)
    posAngle -= 2*PI;
else if (posAngle <= (-2*PI))
    posAngle += 2*PI;

OldDeltaRightCounter=rightCounter;
OldDeltaLeftCounter=leftCounter;
}


// POZYCJA ROBOTA
void Pozycja(void *parameters){
  while(1){
    Odometry();
    z = sqrt(pow((Xg-posX),2) + pow((Yg-posY),2));
    psi = posAngle - atan2(Yg-posY,Xg-posX); // moze byc na odwrot odejmowanie posAngle ?
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// ODCZYT CZUJNIKOW ULTRASONIC
void UltrasonicL(void *parameter){
  while(1){
  digitalWrite(TRIG3, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG3, LOW);
  time_HC04_L = pulseIn(ECHO3, HIGH);
  distanceHC_CM_L = time_HC04_L / 58.0;
 
  digitalWrite(TRIG2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);
  time_HC04_R = pulseIn(ECHO2, HIGH);
  distanceHC_CM_R = time_HC04_R / 58.0;
  
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  time_HC04 = pulseIn(ECHO, HIGH);
  distanceHC_CM = time_HC04 / 58.0;

  if (distanceHC_CM_L > 420)
distanceHC_CM_L =420;
if (distanceHC_CM > 420)
distanceHC_CM =420;
if (distanceHC_CM_R > 420)
distanceHC_CM_R =420;

  g_fisInput[0] = distanceHC_CM_L;
  g_fisInput[1] = distanceHC_CM;
  g_fisInput[2] = distanceHC_CM_R;

    g_fisOutput[0] = 0;
    g_fisOutput[1] = 0;
    fis_evaluate();

  vTaskDelay(15/portTICK_PERIOD_MS);
  }
}



//////// WIFI

void WiFi_con(void *parameter){
while(1){
  /*
if (!client.connected()) {
    reconnect();
  }                                             //WIFI
  client.loop();
  */

  vTaskDelay(70/portTICK_PERIOD_MS);
}
}
int iter=0;

void UploadToDatabase(void *parameter){
  while(1){
    
    dtostrf(posX, 1, 2, posXString);
    client.publish("esp32/posX", posXString);       //BAZA DANYCH, publikowanie do bazy po mqtt na tematach
    dtostrf(posY, 1, 2, posYString);
    Serial.print("Xg: ");
    Serial.println(Xg);
    Serial.print("Yg: ");
    Serial.println(Yg);
    client.publish("esp32/posY", posYString);
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
}

void DisplayMenu(void *parameter){
  while(1){
    curr_button=digitalRead(Button);


if (!curr_button&&prev_button==0){
  
  if (menuPos==2)
  menuPos=0;
  else
  menuPos++;
  prev_button=1;					
}
else if (curr_button)
{

 prev_button=0;

}

switch (menuPos)
{
case 0:
  u8g2.setFont(u8g2_font_5x8_tr); 
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 8);
    u8g2.print(F("STAN BATERII"));
    u8g2.setCursor(0, 16);
    u8g2.print(F("xx %"));
    u8g2.setCursor(4, 24);
    u8g2.print(F("--------------"));
    u8g2.setCursor(0, 32);
    u8g2.print(F("POBOR PRADU"));
    u8g2.setCursor(0, 40);
    u8g2.print(F("xx.xx A"));
  } while ( u8g2.nextPage() );
  break;

case 1:
  u8g2.setFont(u8g2_font_5x8_tr); 
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 8);
    u8g2.print(F("TRYB JAZDY"));
    u8g2.setCursor(0, 16);
    u8g2.print((Direction));
    u8g2.setCursor(4, 24);
    u8g2.print(F("--------------"));
    u8g2.setCursor(0, 32);
    u8g2.print(F("PRZEJECHANA TRASA"));
    u8g2.setCursor(0, 40);
    u8g2.print(((counterL+counterR)/2*distancePerCount));
  } while ( u8g2.nextPage() );
  break;

  case 2:
  u8g2.setFont(u8g2_font_5x8_tr); 
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 8);
    u8g2.print(F("3 EKRAN TEST"));
    u8g2.setCursor(0, 16);
    u8g2.print(F("TEST"));
    u8g2.setCursor(4, 24);
    u8g2.print(F("--------------"));
    u8g2.setCursor(0, 32);
    u8g2.print(F(" EKRAN 3 "));
    u8g2.setCursor(0, 40);
    u8g2.print(F("TEST"));
  } while ( u8g2.nextPage() );
  break;

default:
  break;
}

/*u8g2.setFont(u8g2_font_5x8_tr); 
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 8);
    u8g2.print(F("WCISNIETY"));
    u8g2.setCursor(0, 16);
    u8g2.print((posY));
  } while ( u8g2.nextPage() );*/

if (!curr_button&&prev_button==0){
  
  if (menuPos==2)
  menuPos=0;
  else
  menuPos++;
  prev_button=1;					
}
else if (curr_button)
{

prev_button=0;

}
 vTaskDelay(20/portTICK_PERIOD_MS);
  }
}


// SETUP
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_ROBO"); //Bluetooth device name

  pinMode(B1A,OUTPUT);// define pin as output
  pinMode(B1B,OUTPUT);
  pinMode(A1A,OUTPUT);
  pinMode(A1B,OUTPUT);   
  pinMode(TRIG, OUTPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(ECHO3, INPUT);
  pinMode(LH_ENCODER_A,INPUT);
  pinMode(LH_ENCODER_B,INPUT);
  pinMode(RH_ENCODER_A,INPUT);
  pinMode(RH_ENCODER_B,INPUT);
  pinMode(Button,INPUT_PULLUP);

  u8g2.begin();
  u8g2.setContrast(145);
  u8g2.setColorIndex(1);
  
  ledcSetup(0,22000,8);
  ledcSetup(1,22000,8);
  ledcSetup(2,22000,8);
  ledcSetup(3,22000,8);
  ledcAttachPin(B1A,0); 
  ledcAttachPin(B1B,1);
  ledcAttachPin(A1A,2); 
  ledcAttachPin(A1B,3);
  attachInterrupt(digitalPinToInterrupt(25),pulseL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(35),pulseR,CHANGE);

  //setup_wifi();
  client.setServer(mqtt_server, 1883);


  xTaskCreate(
    UltrasonicL,    // Function that should be called
    "UltraSonicC",   // Name of the task (for debugging)
    2048,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    &task_1            // Task handle
  );

  xTaskCreate(
    Blutacz,     
    "Blutacz",    
    2048,             
    NULL,            
    1,               
    &task_2           
  );

  xTaskCreate(
    Pozycja,     
    "Pozycja",    
    2048,             
    NULL,            
    1,               
    &task_3         
  );

   xTaskCreate(
    WiFi_con,    
    "WiFi",   
    2048,            
    NULL,            
    1,              
    &task_4             
  );                                        //BAZA DANYCH WIFI

  xTaskCreate(
    UploadToDatabase, 
    "Database",  
    2048,      
    NULL,       
    1,         
    &task_5     
  );
  
  xTaskCreate(
    DisplayMenu, 
    "Display",  
    2048,      
    NULL,       
    1,         
    &task_6     
  );

}

void loop() {
 
if (Direction == 'N') // RESET DO CELU
{
  counterL=0;
  counterR=0;
  posX=0;
  posY=0;

leftCounter=0;
rightCounter=0;
deltaLeftCounter=0;
deltaRightCounter=0;
deltaDistance=0;
deltaX = 0;
deltaY = 0;
deltaAngle = 0;
posAngle =0; //-PI;
OldDeltaRightCounter=0;
OldDeltaLeftCounter=0;
}



else if (Direction == 'G') //  Ustawienie współrzędnych
    {
    Xg=Position[1]-'0'; // zamiana char na float
    Yg=Position[2]-'0'; // zamiana char na float
    }

else if (Direction == 'M'){ // DO CELU


//actualTime = millis();

    g_fisInput_G[0] = z;
    g_fisInput_G[1] = psi;

    g_fisOutput_G[0] = 0;
    g_fisOutput_G[1] = 0;

    fis_evaluate_G();
//lastTime = actualTime;


    P = minimum(distanceHC_CM,distanceHC_CM_L,distanceHC_CM_R)/420.0;

    if (P>1)
    P=1.0;
    if (P<-1)
    P=-1.0;


    VelL= (0.7-P)*g_fisOutput[0] + (P+0.3)*g_fisOutput_G[0];
    VelR= (0.7-P)*g_fisOutput[1] + (P+0.3)*g_fisOutput_G[1];

  

if(actualTime-lastTime >= 100UL){

 

//Serial.println(P);
//Serial.println(posY);   
lastTime=actualTime;
}

if (VelL >0 && VelR > 0)
    {
      speedMOTOR_L=map(VelL,0,10,140,220);
      speedMOTOR_R=map(VelR,0,10,160,250);
      MoveForward1(speedMOTOR_L,speedMOTOR_R);
      //Serial.println(speedMOTOR);
      //Serial.println(g_fisOutput[0]);
    }
if (VelL<0 && VelR > 0)
    {
      speedMOTOR_L=map(VelL,-10,0,120,250);
      speedMOTOR_R=map(VelR,0,10,120,250);
      MoveLeft1(speedMOTOR_L,speedMOTOR_R);
    }
if (VelL>0 && VelR < 0)
    {
      speedMOTOR_L=map(VelL,0,10,100,220);
      speedMOTOR_R=map(VelR,-10,0,120,250);
      MoveRight1(speedMOTOR_L,speedMOTOR_R);
    }
if (VelL <0 && VelR < 0)
{
      speedMOTOR_L=map(VelL,-10,0,120,250);
      speedMOTOR_R=map(VelR,-10,0,120,250);
      MoveBackward1(speedMOTOR_L);
}

    
if (z<0.08){ //Osiagniecie celu

      //Direction='S';
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      ledcWrite(2, 0);
      ledcWrite(3, 0);
      delayMicroseconds(100);
}
}

else if(Direction == 'U')
    {
      
    ledcWrite(0, 0);
    ledcWrite(1, 210);
    ledcWrite(2, 0);
    ledcWrite(3, 230);
    delayMicroseconds(100);
    }

else if (Direction == 'R')
    {
      //RIGHT
     
      ledcWrite(0, 0);
      ledcWrite(1, 250);
      ledcWrite(2, 250);
      ledcWrite(3, 0);
      delayMicroseconds(100);
    }
else if (Direction == 'L')
    {
      //LEFT
      
      ledcWrite(0, 250);            
      ledcWrite(1, 0);
      ledcWrite(2, 0);
      ledcWrite(3, 250);
      delayMicroseconds(100);
    }
else if (Direction == 'D')
    {
      //BACK (D - Down)
      
      ledcWrite(0, 250);
      ledcWrite(1, 0);
      ledcWrite(2, 250);
      ledcWrite(3, 0);
      delayMicroseconds(100);
    }

else if (Direction == 'S')
    {
      //STOP
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      ledcWrite(2, 0);
      ledcWrite(3, 0);
      delayMicroseconds(100);
    }

/*
u8g2.setFont(u8g2_font_5x8_tr); 
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 8);
    u8g2.print(F("Hello World!"));
  } while ( u8g2.nextPage() );
*/


 



else if (Direction=='A')
{
  while(Direction=='A'){
    
    
  
if (g_fisOutput[0] >0 && g_fisOutput[1] > 0)
    {
      speedMOTOR_L=map(g_fisOutput[0],1,10,141,250);
      speedMOTOR_R=map(g_fisOutput[1],1,10,141,250);
      MoveForward1(speedMOTOR_L,speedMOTOR_R);

    }
    else if (g_fisOutput[0]<0 && g_fisOutput[1] > 0)
    {
      speedMOTOR_L=map(g_fisOutput[0],-10,0,141,250);
      speedMOTOR_R=map(g_fisOutput[1],0,10,141,250);
      MoveLeft1(speedMOTOR_L,speedMOTOR_R);

    }
    else if (g_fisOutput[0]>0 && g_fisOutput[1] < 0)
    {
      speedMOTOR_L=map(g_fisOutput[0],0,10,141,250);
      speedMOTOR_R=map(g_fisOutput[1],-10,0,141,250);
      MoveRight1(speedMOTOR_L,speedMOTOR_R);
      //Serial.println(speedMOTOR);
      //Serial.println(g_fisOutput[0]); //debug

    }
  }
 
}



}