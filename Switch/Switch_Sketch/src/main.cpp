/**
 * Mower Passage Relay Station Sketch v1.0
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

// Uncomment if a display is used to show the state. A push button on IO14, connected to GND will activate the display
// Default: Uncommented
#define USE_DISPLAY // Uncomment to use a 128x64 OLED display

// Time in seconds the display stays on
// Default: 60
#define DISPLAY_ON_TIME_S 60

// Interval in seconds the esp32 wakes up and asks the mower for the actual state
// Default: 10
#define SLEEP_TIME_S 10
// Interval in seconds the esp32 wakes up, if communication with the mower failed.
// That means typically that the mower is not in range and the esp32 can sleep longer to save battery
// Default: 60
#define SLEEP_TIME_FAILURE_S 30

// Mac-Adress of the mower. This is the most important part and has to match the defined Mac-Adress in the Mower ESP32 sketch
// otherwise a communication is not possible. Only needs to be changed, if there are other ESP32 Mowers in the neighborhood
// Every number can be changed to a value between 0-9. For example {0, 5, 0, 9, 0, 1}. That should be enough possible
// combinations to find a unique one in the neighborhood.
// Default: {0,0,0,0,0,1}
#define MOWER_MAC_ADDRESS \
    {                     \
        0, 0, 0, 0, 0, 1  \
    }

// Comment if the ESP32 should not go to sleep to save power. This is only recommended if the ESP32 is wire-powered by a
// USB-Cable with USB-Charger for example
// Default: Uncommented
#define USE_SLEEP

// Which ESP32 Board is used. Pin Outs for the boards are different!
// Choices: Firebeetle32, devkit
#define FIREBEETLE32
// #define DEVKIT_V1

// --------------------------------------------------------------------------------------------------------------------
// END General Settings
// --------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------
// END Configuration - No need to change anything below - unless you know what you are doing!!
// --------------------------------------------------------------------------------------------------------------------

#ifdef USE_DISPLAY
// U8g2 library by Oliver -> https://github.com/olikraus/u8g2
#include <U8g2lib.h>
#include <Wire.h>

// Interval the display refreshes in milliseconds
// Default: 500
#define DISPLAY_REFRESH_INTERVAL_MS 500

// We need to choose the right class for the display here.
// Uncomment only one of them. If you have another display,
// take a look at the U8g2 documentation.
// 128x64 Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* clock=*/SCL, /* data=*/SDA, /* reset=*/U8X8_PIN_NONE);
#endif

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#ifdef FIREBEETLE32
// Firebeetle only input pins: 34,35,36,39
// firebeetle available pins: 25(D2), 26(D3), 27(D4), 9(D5), 10(D6), 13(D7), 5(D8)
const byte relaisPin = 26;
const byte solarManagerPin = 25;
const byte displayVccPin = 14;
const byte displayGndPin = 12;
// Interrupt pin and the showInfoPin should be the same. That way only one push button is needed to
// start the display either in sleep or when esp32 is awake
const gpio_num_t interruptPin = GPIO_NUM_15;
const byte showInfoPin = 15;
#endif

#ifdef DEVKIT_V1
// only input pins: 34,35,36,39
const byte relaisPin = 26;
const byte solarManagerPin = 25;
const byte displayVccPin = 19;
const byte displayGndPin = 23;
// Interrupt pin and the showInfoPin should be the same. That way only one push button is needed to
// start the display either in sleep or when esp32 is awake
const gpio_num_t interruptPin = GPIO_NUM_15;
const byte showInfoPin = 15;
#endif



typedef struct DATA_TO_MOVER
{
    uint8_t checkSum;
} DATA_TO_MOVER;

// Mover State: 0:undefined 1:mowing 2:searching
typedef struct DATA_TO_PASSAGE
{
    uint8_t moverState;
    uint8_t checkSum;
} DATA_TO_PASSAGE;

// Struct for outgoing data
DATA_TO_PASSAGE dataToPassage;
// Struct for the incoming data
DATA_TO_MOVER dataToMover = {0};

// Programm vars
// Mac address of the mover
uint8_t moverMacAddress[] = MOWER_MAC_ADDRESS;
esp_now_peer_info_t peerInfo;
uint32_t timeMessageSend_ms = 0;
uint32_t lastMessageToMover = 0;
RTC_DATA_ATTR bool relaisState = false;
RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR uint16_t failedMessages = 0;
RTC_DATA_ATTR uint16_t successfulMessages = 0;
RTC_DATA_ATTR uint16_t failedMessagesSuccessively = 0;
RTC_DATA_ATTR uint16_t checkSumErrorCount = 0;
RTC_DATA_ATTR uint16_t checkSumErrorTotal = 0;

#ifdef USE_DISPLAY
bool displayState = false;
uint32_t displayStartTime = 0;
uint32_t lastDisplayRefresh = 0;
#endif

#ifdef DEBUGGING
uint32_t awakeTime = 0;
#endif

char *getMacStrFromAddress(uint8_t *address)
{
    static char macStr[18];
    // Copies the sender mac address to a string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", address[0], address[1], address[2], address[3], address[4], address[5]);
    return macStr;
}

void goToSleep(uint16_t intveral_s = SLEEP_TIME_S)
{
#ifdef USE_SLEEP
    // Only sleep if display is off
    if (!displayState)
    {

#ifdef DEBUGGING
        Serial.print("Going to sleep/delay " + String(intveral_s) + "s now, time awake: ");
        Serial.print(millis() - awakeTime);
        Serial.println(" ms");
        Serial.println("");
        Serial.flush();
#endif
        // Use external pin for wakeup
        esp_sleep_enable_ext0_wakeup(interruptPin, 0);
        esp_wifi_stop();
        // esp_sleep_enable_timer_wakeup(intveral_s * 1000000);
        esp_deep_sleep(intveral_s * 1000000);
    }
#endif
}

void powerPulseRelay()
{
    digitalWrite(solarManagerPin, HIGH);
    delay(10); // The relay need at leat 5ms to switch the state
    digitalWrite(solarManagerPin, LOW);
}

void switchRelayOff()
{
    if (relaisState == false)
    {
        return;
    }
    relaisState = false;
    Serial.println("Relais OFF");
    digitalWrite(relaisPin, LOW);
    powerPulseRelay();
}

void switchRelayOn()
{
    if (relaisState == true)
    {
        return;
    }
    relaisState = true;
    Serial.println("Relais ON");
    digitalWrite(relaisPin, HIGH);
    powerPulseRelay();
}

void sendMessageToMover()
{
    lastMessageToMover = millis();

    // Generate random number that we verify when the mower sends data back.
    dataToMover.checkSum = random(0, 254);
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(moverMacAddress, (uint8_t *)&dataToMover, sizeof(dataToMover));
    if (result != ESP_OK)
    {
#ifdef DEBUGGING
        Serial.println("Error sending the data - Things wont work");
#endif
        goToSleep(SLEEP_TIME_FAILURE_S);
    }
}

// True: Passage open; False: Passage close
void setPassage(bool state)
{
#ifdef DEBUGGING
    if (state)
        Serial.println("opening passage");
    else
        Serial.println("closing passage");
#endif
    if (state)
        switchRelayOn();
    else
        switchRelayOff();
}

void OnDataRecv(const uint8_t *address, const uint8_t *incomingData, int len)
{
    failedMessagesSuccessively = 0;
    successfulMessages++;

    memcpy(&dataToPassage, incomingData, sizeof(dataToPassage));

    // If check sums do not match something went wrong during communication
    if (dataToPassage.checkSum != dataToMover.checkSum)
    {
        checkSumErrorCount++;
        checkSumErrorTotal++;

#ifdef DEBUGGING
        Serial.print("Checksum does not match, failures in a row: " + (String) checkSumErrorCount);
        Serial.println( "Total failures: " + (String) checkSumErrorTotal);
#endif
        
        if (checkSumErrorCount < 3)
        {
            // Send message again and hope for better result
            sendMessageToMover();
        }
        else
        {
            // Go sleeping and try communication in the next interval
            goToSleep();
        }
        return;
    }
    else
    {
        // Reset error count if communication was successful
        checkSumErrorCount = 0;
    }

    if (dataToPassage.moverState == 1)
    {
        setPassage(false);
    }
    else if (dataToPassage.moverState == 2)
    {
        setPassage(true);
    }

#ifdef DEBUGGING
    if (dataToPassage.moverState == 1)
    {
        Serial.println("Got message from Mower: He is mowing");
    }
    else if (dataToPassage.moverState == 2)
    {
        Serial.println("Got message from Mower: He is not mowing");
    }
#endif
    goToSleep();
}

void OnDataSent(const uint8_t *address, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        timeMessageSend_ms = millis();
    }
    else
    {
#ifdef DEBUGGING
        Serial.println("Message send failure - Mower probably to far away");
#endif
        failedMessages++;
        failedMessagesSuccessively++;
        if (failedMessagesSuccessively > 3)
        {
            // Set higher sleep time, since the mower is not in range for a while
            // we can sleep longer - the mower is not that fast :-)
            goToSleep(SLEEP_TIME_FAILURE_S);
        }
        else
        {
            goToSleep();
        }
    }
}

void displayStart()
{
#ifdef DEBUGGING
    Serial.println("Display Start");
#endif
    displayStartTime = millis();
    digitalWrite(displayVccPin, HIGH);
    delay(100);
    display.begin();
    displayState = true;
}

void updateDisplay()
{
    if (displayState && millis() - lastDisplayRefresh > DISPLAY_REFRESH_INTERVAL_MS)
    {
        display.clearBuffer();
        display.setFont(u8g2_font_ncenB12_tr);
        display.drawStr(0, 12, "Passage:");
        if (relaisState)
            display.drawStr(80, 12, "Open");
        else
            display.drawStr(80, 12, "Close");

        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(0, 26, "Failed Msg:");
        display.setCursor(80, 26);
        display.print(failedMessagesSuccessively);

        display.drawStr(0, 40, "ok/fail:");
        display.setCursor(65, 40);
        display.print((String) successfulMessages + "/" + (String) failedMessages);

        display.drawStr(0, 54, "CKSum err:");
        display.setCursor(80, 54);
        display.print(checkSumErrorTotal);

        display.sendBuffer();

        lastDisplayRefresh = millis();
    }
    if (displayState && millis() - displayStartTime > DISPLAY_ON_TIME_S * 1000)
    {
        displayState = false;
        digitalWrite(displayVccPin, LOW);
#ifdef DEBUGGING
        Serial.println("Display off");
#endif
    }
}

void setupWifi()
{
    WiFi.enableLongRange(true);
    WiFi.mode(WIFI_STA);

    //Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
#ifdef DEBUGGING
        Serial.println("Error initializing ESP-NOW. Things wont work");
#endif
        goToSleep(SLEEP_TIME_FAILURE_S);
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);

    // Register mower esp32 as peer
    memcpy(peerInfo.peer_addr, moverMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
#ifdef DEBUGGING
        Serial.println("Failed to add peer. Things wont work");
#endif
        goToSleep(SLEEP_TIME_FAILURE_S);
    }
}

void debugRelay()
{
    switchRelayOn();
    delay(1000);
    switchRelayOff();
    delay(1000);
    switchRelayOn();
    delay(1000);
    switchRelayOff();
    delay(1000);
    switchRelayOn();
    delay(1000);
    switchRelayOff();
    delay(1000);
    switchRelayOn();
    delay(1000);
    switchRelayOff();
    delay(1000);
}

void checkWakeUpReason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
    {
        displayStart();
    }
}

void setup()
{
    esp_deep_sleep_disable_rom_logging();
    bootCount++;
#ifdef DEBUGGING
    Serial.begin(115200);
    awakeTime = millis();
#endif

    pinMode(relaisPin, OUTPUT);
    pinMode(solarManagerPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(showInfoPin, INPUT_PULLUP);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(solarManagerPin, LOW);
    digitalWrite(relaisPin, relaisState);

#ifdef USE_DISPLAY
    pinMode(displayVccPin, OUTPUT);
    pinMode(displayGndPin, OUTPUT);
    digitalWrite(displayVccPin, LOW);
    digitalWrite(displayGndPin, LOW);
#endif

    checkWakeUpReason();

    // Force setting the relay every 10 boot times, to have the right state for sure
    if (bootCount % 10 == 1)
    {
#ifdef DEBUGGING
        Serial.print("Force reset relais, it should be: ");
        Serial.println(relaisState);
#endif
        relaisState = !relaisState;
        setPassage(!relaisState);
    }

#ifdef DEBUGGING
    Serial.print("BootCount: ");
    Serial.println(bootCount);
    Serial.print("RelaisState: ");
    Serial.println(relaisState);
#endif

    // debugRelay();
    setupWifi();
    sendMessageToMover();
}

void loop()
{
#ifdef USE_DISPLAY
    updateDisplay();
#endif

    // This code is only reached if the display is ON or the sleep mode is OFF, otherwise the esp32 goes to sleep before
    uint32_t interval = SLEEP_TIME_S * 1000;
    // Increase message send interval time to see changes on the display
    if (displayState)
        interval = interval / 4;
    if (millis() - lastMessageToMover > interval)
    {
        sendMessageToMover();
    }

#ifdef USE_DISPLAY
    // Start the display if the showInfoPin is triggered
    if (digitalRead(showInfoPin) == LOW && !displayState)
    {
        displayStart();
    }
#endif

    delay(10);
}
