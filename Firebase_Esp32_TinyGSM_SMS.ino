#include <WiFi.h>
#include "FirebaseESP32.h"


#define FIREBASE_HOST "bonikapp.firebaseio.com" //Change to your Firebase RTDB project ID e.g. Your_Project_ID.firebaseio.com
#define FIREBASE_AUTH "5lcU1MxMwbZLSjsFprv7MiBhWDK0t5TvsHN7CwvS" //Change to your Firebase RTDB secret password
#define WIFI_SSID "rochi"
#define WIFI_PASSWORD "dsl@5678"


// Please select the corresponding model

#define SIM800L_IP5306_VERSION_20190610
// #define SIM800L_AXP192_VERSION_20200327
// #define SIM800C_AXP192_VERSION_20200609
// #define SIM800L_IP5306_VERSION_20200811

// Define the serial console for debug prints, if needed
// #define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG          SerialMon

#include "utilities.h"

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800          // Modem is SIM800
#define TINY_GSM_RX_BUFFER      1024   // Set RX buffer to 1Kb

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */

bool reply = false;

TinyGsmClient client(modem);


void setupModem()
{
#ifdef MODEM_RST
    // Keep reset high
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, HIGH);
#endif

    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);

    // Turn on the Modem power first
    digitalWrite(MODEM_POWER_ON, HIGH);

    // Pull down PWRKEY for more than 1 second according to manual requirements
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);

    // Initialize the indicator as an output
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LED_OFF);
}

//Define Firebase Data objects
FirebaseData firebaseData1;
FirebaseData firebaseData2;
FirebaseData firebaseData3;

const int ledPin =  19; //GPIO19 for LED
const int swPin =  18; //GPIO18 for Switch
bool swState = false;
String path = "/Nodes";
String nodeID = "Node2"; //This is this node ID to receive control
String otherNodeID = "Node1"; //This is other node ID to control
String smsNodeID = "SMS";

void streamCallback(StreamData data)
{

  if (data.dataType() == "boolean") {
    if (data.boolData())
      Serial.println("Set " + nodeID + " to High");
    else
      Serial.println("Set " + nodeID + " to Low");
    digitalWrite(ledPin, data.boolData());
  }
}


void streamTimeoutCallback(bool timeout)
{
  if (timeout)
  {
    Serial.println();
    Serial.println("Stream timeout, resume streaming...");
    Serial.println();
  }
}

void setup()
{
    // Set console baud rate
    SerialMon.begin(115200);

    pinMode(ledPin, OUTPUT);
    pinMode(swPin, INPUT);

    Serial.println();
    Serial.println();

    delay(10);

    // Start power management
    if (setupPMU() == false) {
        Serial.println("Setting power error");
    }

    // Some start operations
    setupModem();

    // Set GSM module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);


    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.reconnectWiFi(true);



    if (!Firebase.beginStream(firebaseData1, path + "/" + nodeID))
    {
        Serial.println("Could not begin stream");
        Serial.println("REASON: " + firebaseData1.errorReason());
        Serial.println();
    }

    Firebase.setStreamCallback(firebaseData1, streamCallback, streamTimeoutCallback);

}

void loop()
{
    bool ret = modem.restart();
    if (ret == false) {
        Serial.println("restart ..");
        return;
    }

    Serial.println("Set  SIM800X keep active on");


    // 
    if (digitalRead(swPin) != swState) {

    bool _swState = swState;
    swState = digitalRead(swPin);

    if (Firebase.setBool(firebaseData2, path + "/" + otherNodeID, swState)) {
      if (swState)
        Serial.println("Set " + otherNodeID + " to High");
      else
        Serial.println("Set " + otherNodeID + " to Low");
    } else {
      swState = _swState;
      Serial.println("Could not set " + otherNodeID);
    }

    }

    // Send SMS
//    String SMS_TARGET = Firebase.getString(firebaseData3, path + "/" + smsNodeID);
//    res = modem.sendSMS(SMS_TARGET, String("Hello from ") + imei);
//    DBG("SMS:", res ? "OK" : "fail");


    // Set IO25 to sleep hold, so that when ESP32 sleeps, SIM800X will keep power and running
    gpio_hold_en(GPIO_NUM_25);  //MODEM_POWER_ON

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    esp_deep_sleep_start();
}
