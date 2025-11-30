#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define PIN_SCK 26
#define PIN_SDA 27
#define PIN_LEFT 32
#define PIN_RIGHT 33

// NOTE : REPLACE with the MAC address shown in Player 1 console
uint8_t p1Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ****************************************
    * NOTE : Ensure following values and definitions are consistent with Player 1 code
    * BEGIN
**************************************** */

static constexpr int playerWidth = 30;
static constexpr int playerHeight = 2;
static constexpr float playerSpeed = 3;

static constexpr int ballRadius = 4;

// (p1y and p2y will be swapped in Player 1 code)
static constexpr int p1y = 5;
static constexpr int p2y = SCREEN_HEIGHT - playerHeight - 5;

typedef struct _P2State {
    int p2x;
    bool p2Served;
} P2State;

typedef enum _Serve {
    NONE,
    PLAYER1,
    PLAYER2
} Serve;

typedef struct _GameState {
    int p1x;
    int p1Points;
    int p2Points;
    int ballX;
    int ballY;
    Serve serve;
} GameState;

P2State p2State;
Serve serve;
GameState gameState;
esp_now_peer_info_t peerInfo;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const unsigned long timeoutMs = 2000;

/* ****************************************
    * END
**************************************** */

int p1x = (SCREEN_WIDTH - playerWidth) / 2;
int p1Points;

int p2x = (SCREEN_WIDTH - playerWidth) / 2;
int p2Dir;
int p2Points;
bool p2Served;

float ballX;
float ballY;

bool gamePaused = true;
unsigned long lastPacketTime;

void drawCenteredText(const char* s, int16_t y = -1);

void readMacAddress() {
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    if (ret == ESP_OK) {
        Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                      baseMac[0], baseMac[1], baseMac[2],
                      baseMac[3], baseMac[4], baseMac[5]);
    } else {
        Serial.println("Failed to read MAC address");
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Data delivery successful" : "Data delivery failed");
}

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&gameState, incomingData, sizeof(gameState));
    // Serial.print("Bytes received: ");
    // Serial.println(len);
 
    p1x = gameState.p1x;
    p1Points = gameState.p1Points;
    p2Points = gameState.p2Points;
    ballX = gameState.ballX;
    // Invert ball's y pos on P2 screen
    ballY = SCREEN_HEIGHT - gameState.ballY;
    serve = gameState.serve;

    gamePaused = false;
    lastPacketTime = millis();
}

void blink(void *pvParameters) {
    while (1) {
        digitalWrite(2, HIGH);
        delay(500);
        digitalWrite(2, LOW);
        delay(500);
    } 
}

void setup() {
    Serial.begin(9600);

    pinMode(PIN_LEFT, INPUT_PULLUP);
    pinMode(PIN_RIGHT, INPUT_PULLUP);
    pinMode(2, OUTPUT);

    Wire.setPins(PIN_SDA, PIN_SCK);

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        return;
    }

    Serial.println("Display loaded");
    display.clearDisplay();
    display.fillRect(p2x, p2y, playerWidth, playerHeight, WHITE);
    drawCenteredText("Connecting to P1...");
    display.display();

    TaskHandle_t blinkHandle = NULL;
    xTaskCreatePinnedToCore(blink, "blink", 1000, NULL, 1, &blinkHandle, 0);

    WiFi.mode(WIFI_STA);

    Serial.println("Hello, world from Player 2!");
    Serial.print("My MAC address is : ");
    readMacAddress();

    Serial.println("Connecting to Player 1...");

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(esp_now_send_cb_t(onDataSent));

    memcpy(peerInfo.peer_addr, p1Address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));

    p2State.p2x = p2x;
    p2State.p2Served = p2Served;
    esp_now_send(p1Address, (uint8_t *)&p2State, sizeof(p2State));

    while (gamePaused) {
        delay(100);
    }
    
    Serial.println("Connected to Player 1 successfully!");

    vTaskDelete(blinkHandle);

    digitalWrite(2, HIGH);

    display.clearDisplay();
    display.fillRect(p1x, p1y, playerWidth, playerHeight, WHITE);
    display.fillCircle(ballX, ballY, ballRadius, WHITE);
    display.display();
}

unsigned long lastSendTime;
static constexpr unsigned long sendInterval = 50;
void loop() {
    if (millis() - lastPacketTime > timeoutMs) {
        gamePaused = true;
    }

    display.clearDisplay();

    if (gamePaused) {
        Serial.printf("Connection timed out (%f)\n", (millis() - lastPacketTime) / 1000.0f);
        drawCenteredText("P1 disconnected");
        digitalWrite(2, LOW);
    } else {
        digitalWrite(2, HIGH);

        if (!digitalRead(PIN_LEFT)) {
            p2Dir = -1;
        } else if (!digitalRead(PIN_RIGHT)) {
            p2Dir = 1;
        } else {
            p2Dir = 0;
        }

        if (serve != NONE) {
            p2x = (SCREEN_WIDTH - playerWidth) / 2;
            p2Dir = 0;
        } else {
            p2Served = false;
        }
        if (serve == PLAYER2) {
            drawCenteredText("Your serve", 20);
            if (!digitalRead(PIN_LEFT) || !digitalRead(PIN_RIGHT)) {
                p2Served = true;
            }
        } else if (serve == PLAYER1) {
            drawCenteredText("P1's serve", 20);
        }

        p2x += p2Dir * playerSpeed;
        if (p2x <= 0) {
            p2x = 0;
        } else if (p2x + playerWidth > SCREEN_WIDTH) {
            p2x = SCREEN_WIDTH - playerWidth;
        }

        display.fillRect(p1x, p1y, playerWidth, playerHeight, WHITE);
        display.fillRect(p2x, p2y, playerWidth, playerHeight, WHITE);
        display.fillCircle(ballX, ballY, ballRadius, WHITE);
        char s[8];
        sprintf(s, "%d", p2Points);
        drawCenteredText(s, 48);
        sprintf(s, "%d", p1Points);
        drawCenteredText(s, 10);
    }

    display.display();

    if (millis() - lastSendTime >= sendInterval) {
        p2State.p2x = p2x;
        p2State.p2Served = p2Served;
        esp_err_t result = esp_now_send(p1Address, (uint8_t *)&p2State, sizeof(p2State));
        lastSendTime = millis();
    }
}

void drawCenteredText(const char* s, int16_t y) {
    int16_t x1, y1;
    uint16_t w, h;

    // Get s bounds
    display.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);

    // Center coordinates
    int16_t x = (SCREEN_WIDTH - w) / 2;
    if (y == -1) 
        y = (SCREEN_HEIGHT - h) / 2;

    display.setCursor(x, y);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.print(s);
}