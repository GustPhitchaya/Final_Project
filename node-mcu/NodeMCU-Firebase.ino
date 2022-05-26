#include <Arduino.h>
#include <DigitalTube.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <SoftwareSerial.h>
#include <WiFiUdp.h>
// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "yun318801 2G"
#define WIFI_PASSWORD "08032545"

// Insert Firebase project API Key
// #define API_KEY "REPLACE_WITH_YOUR_PROJECT_API_KEY"
#define API_KEY "AIzaSyCqlh-xAnuPivTtfW3L9FPC4fvJqTnrm0I"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "kanin49144@gmail.com"
#define USER_PASSWORD "12345678"

// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL "https://door-locking-a1764-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Database main path (to be updated in setup with the user UID)
String databasePath;
// Database child nodes

String timePath = "/timestamp";
String enteredPinPath = "/entered pin";
// Parent Node (to be updated in every loop)
String parentPath;
String passwordPath = "/UsersData";
FirebaseJson json, passjson;

// Define NTP Client to get time
WiFiUDP ntpUDP;

// Variable to save current epoch time
int timestamp;

SoftwareSerial NodeMCU(D7, D8); // RX | TX
DigitalTube dis(D3, D2, D1);
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 1000;

int val = 130000;
char showarr[4];
int idx = 3;
int status = 4;

// Initialize WiFi
void initWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
    Serial.println();
}

void postEnteredPin(int entered) {
    sendDataPrevMillis = millis();
    // Get current timestamp
    timestamp = getTime();
    Serial.print("time: ");
    Serial.println(timestamp);

    parentPath = databasePath + "/" + String(timestamp);
    json.set(timePath, String(timestamp));
    json.set(enteredPinPath, entered);
    Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
}

int getKey() {
    if (Firebase.RTDB.getInt(&fbdo, "/Password/key")) {
        if (fbdo.dataType() == "int") {
            int key = fbdo.intData();
            Serial.println(key);
            return key;
        }
    } else {
        Serial.println(fbdo.errorReason());
    }
    return 0;
}
// Function that gets current epoch time
unsigned long getTime() {
    timeClient.update();
    unsigned long now = timeClient.getEpochTime();
    return now;
}

void setup() {
    pinMode(D7, INPUT);
    pinMode(D8, OUTPUT);
    Serial.begin(9600);
    NodeMCU.begin(9600);
    dis.begin();
    initWiFi();
    timeClient.begin();

    // Assign the api key (required)
    config.api_key = API_KEY;

    // Assign the user sign in credentials
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    // Assign the RTDB URL (required)
    config.database_url = DATABASE_URL;

    Firebase.reconnectWiFi(true);
    fbdo.setResponseSize(4096);

    // Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

    // Assign the maximum retry of token generation
    config.max_token_generation_retry = 5;

    // Initialize the library with the Firebase authen and config
    Firebase.begin(&config, &auth);

    // Getting the user UID might take a few seconds
    Serial.println("Getting User UID");
    while ((auth.token.uid) == "") {
        Serial.print('.');
        delay(1000);
    }
    // Print user UID
    uid = auth.token.uid.c_str();
    Serial.print("User UID: ");
    Serial.println(uid);

    // Update database path
    databasePath = "/UsersData/" + uid + "/log";
}

void loop() {
    if (Firebase.ready() && NodeMCU.available()) {
        int tmp = NodeMCU.parseInt();
        status = tmp / 100000;
        Serial.println(tmp);
        switch (status) {
        case 1:
            val = tmp;
            Serial.println("OFF!");
            NodeMCU.print("9\n\r");
            break;
        case 2:
            val = tmp;
            Serial.println("ON!");
            int data = val % 10000;
            postEnteredPin(data);
            // post data log
            // request key pin from firebase
            int key = getKey();
            Serial.println(key);
            // paring between data and real pin
            int isCorrect = (data == key);
            // send back status if it's true send 1 else send 0
            if (isCorrect) {
                NodeMCU.print("1\n\r");
            } else {
                NodeMCU.print("0\n\r");
            }
            break;
        case 3:
            val = tmp;
            Serial.println("Change!");
            int data = val % 10000;
            passjson.set("key", data);
            Firebase.RTDB.updateNode(&fbdo, "/Password", &passjson);
        default:
            break;
        }
    }
    Serial.println(status);
    if (status == 4) {
        dis.show('-', '-', '-', '-');
        delay(500);
    } else {
        idx = (val / 10000) % 10;
        // Serial.println(idx);
        showarr[0] = (val / 1000) % 10;
        showarr[1] = (val / 100) % 10;
        showarr[2] = (val / 10) % 10;
        showarr[3] = (val / 1) % 10;
        dis.show(showarr[0], showarr[1], showarr[2], showarr[3]);
        delay(250);
        char savestate = showarr[idx];
        showarr[idx] = ' ';
        dis.show(showarr[0], showarr[1], showarr[2], showarr[3]);
        showarr[idx] = savestate;
        delay(250);
    }
}
