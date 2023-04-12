/**
 * Mower Passage Mower Sketch v1.1
 * find out more: blog.altholtmann.com
 * by Timo Altholtmann
 * 
 * ***********************    General Notes    *****************************
 * - Uncomment means removing the 2 // in front of #define.
 * 
 **/
// --------------------------------------------------------------------------------------------------------------------
// START General Settings
// --------------------------------------------------------------------------------------------------------------------
// Debugging. If enabled, the sketch will print DEBUGGING states to the serial monitor
// with 115200 Baudrate
// Default: commented
// #define DEBUGGING // Uncomment to output DEBUGGING messages

// Mac-Adress of the mower. This is the most important part and has to match the defined Mac-Adress in the Passage ESP32 sketch,
// otherwise a communication is not possible. Only needs to be changed, if there are other ESP32 Mowers in the neighborhood
// Every number can be changed to a value between 0-9. For example {0, 5, 0, 9, 0, 1}. That should be enough possible
// combinations to find a unique one in the neighborhood.
// Default: {0,0,0,0,0,1}
#define MOWER_MAC_ADDRESS \
    {                     \
        0, 0, 0, 0, 0, 1  \
    }

// --------------------------------------------------------------------------------------------------------------------
// END General Settings
// --------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------
// END Configuration - No need to change anything below - unless you know what you are doing!!
// --------------------------------------------------------------------------------------------------------------------

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

const uint16_t stateCheckInterval_ms = 1000;
const uint8_t stateCheckSamples = 5;

const byte sensorVccPin = 25;
const byte sensorGndPin = 26;
const byte sensorPin = 33;
const byte ledPin = LED_BUILTIN;
float filterResult = 0.0;

// programm vars
uint32_t lastStateCheck_MS = 0;
uint32_t lastHallCheck_MS = 0;
uint8_t newMACAddress[] = MOWER_MAC_ADDRESS;

typedef struct DATA_TO_MOVER
{
    uint8_t checkSum;
} DATA_TO_MOVER;

typedef struct DATA_TO_PASSAGE
{
    uint8_t moverState;
    uint8_t checkSum;
} DATA_TO_PASSAGE;

// Mover State: 0:undefined 1:mowing 2:searching
DATA_TO_PASSAGE dataToPassage = {0, 0};
DATA_TO_MOVER dataToMover;
esp_now_peer_info_t peerInfo;

char *getMacStrFromAddress(uint8_t *address)
{
    static char macStr[18];
    // Copies the sender mac address to a string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", address[0], address[1], address[2], address[3], address[4], address[5]);
    return macStr;
}

void addPeerIfNotExists(uint8_t *address)
{
    // Return if the peer already exists
    if (esp_now_is_peer_exist(address))
    {
        return;
    }

#ifdef DEBUGGING
    Serial.print("Adding peer: ");
    Serial.println(getMacStrFromAddress(address));
    Serial.println(" to the peer list");
    Serial.println();
#endif

    // register peer
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    memcpy(peerInfo.peer_addr, address, 6);

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
#ifdef DEBUGGING
        Serial.print("Failed to add peer: ");
        Serial.println(getMacStrFromAddress(address));
#endif

        return;
    }
}

void sendState(uint8_t *address, uint8_t checkSum)
{
    // set the right checksum to the message
    dataToPassage.checkSum = checkSum;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(address, (uint8_t *)&dataToPassage, sizeof(dataToPassage));

    if (result != ESP_OK)
    {
#ifdef DEBUGGING
        Serial.print("Failed sending mover state to: ");
        Serial.println(getMacStrFromAddress(address));
        Serial.println();
#endif
    }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *address, const uint8_t *incomingData, int len)
{
    memcpy(&dataToMover, incomingData, sizeof(dataToMover));
    addPeerIfNotExists((uint8_t *)address);
    // Answer with the current state
    sendState((uint8_t *)address, dataToMover.checkSum);
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
#ifdef DEBUGGING
    Serial.print("Packet to: ");
    Serial.print(getMacStrFromAddress((uint8_t *)mac_addr));
    Serial.print(" send status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        digitalWrite(ledPin, HIGH);
        delay(70);
        digitalWrite(ledPin, LOW);
    }
#endif
}

void setUpWifi()
{
    WiFi.mode(WIFI_STA);

    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

    if (esp_now_init() != ESP_OK)
    {
#ifdef DEBUGGING
        Serial.println("Error initializing ESP-NOW. Things wont work");
#endif
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
}

// Funktion Filtern()                                            
// Bildet einen Tiefpassfilter (RC-Glied) nach.           
// FF = Filterfaktor;  Tau = FF / Aufruffrequenz            
// FiltVal der gefilterte Wert, NewVal der neue gelesene Wert; FF Filterfaktor    
void Filtern(float &FiltVal, float NewVal, int FF)
{
  FiltVal= ((FiltVal * FF) + NewVal) / (FF +1);  
}

void checkSensorState()
{
    // Check the interval
    if (millis() - lastStateCheck_MS > stateCheckInterval_ms)
    {
        uint32_t sum = 0;
        // read sensor pin
        for (uint8_t i = 0; i < stateCheckSamples; i++)
        {
            sum += analogRead(sensorPin);
            delay(5);
        }
        // uint32_t result = sum / stateCheckSamples;
        float result = ((float)sum / (float)stateCheckSamples) / (float)(4040 / 15.7);

#ifdef DEBUGGING
        Serial.print("current Voltage: ");
        Serial.print(result);
        Serial.print(" V - ");
        Serial.print("\t");
#endif
        Filtern(filterResult, result, 8);

#ifdef DEBUGGING
        Serial.print("filtered Voltage: ");
        Serial.print(filterResult);
        Serial.print(" V - ");
        Serial.print("\t");
#endif
        
        // Set state according to the voltage
        if (filterResult > 1)
        {
            // Mower is mowing
            dataToPassage.moverState = 1;
#ifdef DEBUGGING
            Serial.println("Mower is mowing");
#endif
        }
        else
        {
            // Mover does not mow at the moment
            dataToPassage.moverState = 2;
#ifdef DEBUGGING
            Serial.println("Mower is not mowing");
#endif
        }

        lastStateCheck_MS = millis();
    }
}

void setup()
{
#ifdef DEBUGGING
    Serial.begin(115200);
#endif

    pinMode(sensorPin, INPUT);
    pinMode(sensorVccPin, OUTPUT);
    pinMode(sensorGndPin, OUTPUT);
    pinMode(ledPin, OUTPUT);

    digitalWrite(ledPin, LOW);
    digitalWrite(sensorGndPin, LOW);
    digitalWrite(sensorVccPin, HIGH);

    setUpWifi();
}

void loop()
{
    checkSensorState();
}
