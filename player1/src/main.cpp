#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define PIN_SCK 26
#define PIN_SDA 27
#define PIN_LEFT 32
#define PIN_RIGHT 33

// NOTE : REPLACE with the MAC Address shown in Player 2 console 
uint8_t p2Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ****************************************
    * NOTE : Ensure following values and definitions are consistent with Player 2 code
    * BEGIN
**************************************** */

static constexpr int playerWidth = 30;
static constexpr int playerHeight = 2;
static constexpr float playerSpeed = 3;

static constexpr int ballSpeed = 3;
static constexpr int ballRadius = 4;

// (p1y and p2y will be swapped in Player 2 code)
static constexpr int p1y = SCREEN_HEIGHT - playerHeight - 5;
static constexpr int p2y = 5;

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
Serve serve = PLAYER1;
GameState gameState;
esp_now_peer_info_t peerInfo;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const unsigned long timeoutMs = 2000;

/* ****************************************
    * END
**************************************** */

int p1x = (SCREEN_WIDTH - playerWidth) / 2;
int p1Dir;
int p1Points;

int p2x = (SCREEN_WIDTH - playerWidth) / 2;
int p2Points;
bool p2Served;

float ballX = SCREEN_WIDTH / 2;
float ballY = SCREEN_HEIGHT / 2;
float ballDirX;
float ballDirY;

bool gamePaused = true;
unsigned long lastPacketTime;

void normalise(float &x, float &y);
bool intersect(float cx, float cy, float radius, float rx, float ry, float rw, float rh);
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
    memcpy(&p2State, incomingData, sizeof(p2State));
    // Serial.print("Bytes received: ");
    // Serial.println(len);
 
    p2x = p2State.p2x;
    p2Served = p2State.p2Served;

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

    Serial.println("Hello, Player 1");

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
    display.fillRect(p1x, p1y, playerWidth, playerHeight, WHITE);
    drawCenteredText("Connecting to P2...");
    display.display();

    TaskHandle_t blinkHandle = NULL;
    xTaskCreatePinnedToCore(blink, "blink", 1000, NULL, 1, &blinkHandle, 0);

    WiFi.mode(WIFI_STA);

    Serial.print("Player 1 MAC address is : ");
    readMacAddress();

    Serial.println("Connecting to Player 2...");

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(esp_now_send_cb_t(onDataSent));

    memcpy(peerInfo.peer_addr, p2Address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));

    // Send initial handshake message to Player 2 and wait until handshake from Player 2 is received before starting
    gameState.p1x = p1x;
    gameState.p1Points = p1Points;
    gameState.p2Points = p2Points;
    gameState.ballX = ballX;
    gameState.ballY = ballY;
    gameState.serve = serve;
    esp_now_send(p2Address, (uint8_t *)&gameState, sizeof(gameState));

    while (gamePaused) {
        delay(100);
    }
    
    Serial.println("Connected to Player 2 successfully!");

    vTaskDelete(blinkHandle);

    digitalWrite(2, HIGH);

    display.clearDisplay();
    display.fillRect(p1x, p1y, playerWidth, playerHeight, WHITE);
    display.fillRect(p2x, p2y, playerWidth, playerHeight, WHITE);
    display.fillCircle(ballX, ballY, ballRadius, WHITE);
    display.display();
}

unsigned long moveTimer;
unsigned long lastSend;
static constexpr unsigned long sendInterval = 50;
void loop() {
    if (millis() - lastPacketTime > timeoutMs) {
        gamePaused = true;
    }
    
    display.clearDisplay();
    if (gamePaused) {
        Serial.printf("Connection timed out (%f)\n", (millis() - lastPacketTime) / 1000.0f);
        drawCenteredText("P2 disconnected");
        digitalWrite(2, LOW);
    } else {
        digitalWrite(2, HIGH);
        // account for player 2's serve delay so that serves are equal (prevents ball from going past paddle & self-scoring)
        if (millis() - moveTimer >= sendInterval) {
            if (!digitalRead(PIN_LEFT)) {
                p1Dir = -1;
            } else if (!digitalRead(PIN_RIGHT)) {
                p1Dir = 1;
            } else {
                p1Dir = 0;
            }
        }

        if (serve != NONE) {
            p1x = (SCREEN_WIDTH - playerWidth) / 2;
            p1Dir = 0;
            ballX = SCREEN_WIDTH / 2;
            ballY = SCREEN_HEIGHT / 2;
            ballDirX = 0;
            ballDirY = 0;
        }
        if (serve == PLAYER1) {
            drawCenteredText("Your serve", 20);
            if (!digitalRead(PIN_LEFT) || !digitalRead(PIN_RIGHT)) {
                ballDirY = 1;
                serve = NONE;
                moveTimer = millis();
            }
        } else if (serve == PLAYER2) {
            drawCenteredText("P2's serve", 20);
            if (p2Served) {
                ballDirY = -1;
                serve = NONE;
            }
        }

        if (ballY + ballRadius <= 0) {
            serve = PLAYER1;
            p1Points++;
            delay(500);
        } else if (ballY - ballRadius >= SCREEN_HEIGHT) {
            serve = PLAYER2;
            p2Points++;
            delay(500);
        }

        if (ballX - ballRadius <= 0) {
            ballX = ballRadius;
            ballDirX = 1;
            normalise(ballDirX, ballDirY);
        } else if (ballX + ballRadius >= SCREEN_WIDTH) {
            ballX = SCREEN_WIDTH - ballRadius;
            ballDirX = -1;
            normalise(ballDirX, ballDirY);
        }
        if (intersect(ballX, ballY, ballRadius, p1x, p1y, playerWidth, playerHeight)) {
            ballDirY = -1;
            //                   change ballDirX magnitude based on paddle position
            //                   middle = 0, left corner = -1, right corner = 1
            //                    vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv 
            ballDirX = constrain((ballX - (p1x + playerWidth / 2.0f)) / (playerWidth / 2.0f), -1, 1);
            normalise(ballDirX, ballDirY);
        } else if (intersect(ballX, ballY, ballRadius, p2x, p2y, playerWidth, playerHeight)) {
            ballDirY = 1;
            ballDirX = constrain((ballX - (p2x + playerWidth / 2.0f)) / (playerWidth / 2.0f), -1, 1);
            normalise(ballDirX, ballDirY);
        }

        p1x += p1Dir * playerSpeed;
        ballX += ballDirX * ballSpeed;
        ballY += ballDirY * ballSpeed;

        if (p1x <= 0) {
            p1x = 0;
        }
        else if (p1x + playerWidth > SCREEN_WIDTH) {
            p1x = SCREEN_WIDTH - playerWidth;
        }

        display.fillRect(p1x, p1y, playerWidth, playerHeight, WHITE);
        display.fillRect(p2x, p2y, playerWidth, playerHeight, WHITE);
        display.fillCircle(ballX, ballY, ballRadius, WHITE);
        char s[8];
        sprintf(s, "%d", p1Points);
        drawCenteredText(s, 48);
        sprintf(s, "%d", p2Points);
        drawCenteredText(s, 10);
    }

    display.display();

    if (millis() - lastSend >= sendInterval) {
        gameState.p1x = p1x;
        gameState.p1Points = p1Points;
        gameState.p2Points = p2Points;
        gameState.ballX = ballX;
        gameState.ballY = ballY;
        gameState.serve = serve;

        esp_err_t result = esp_now_send(p2Address, (uint8_t *)&gameState, sizeof(gameState));
        
        lastSend = millis();
    }
}

void normalise(float &x, float &y) {
    float len = sqrt(x * x + y * y);
    x /= len;
    y /= len;
}

bool intersect(float cx, float cy, float radius, float rx, float ry, float rw, float rh) {
    int testX = cx;
    int testY = cy;

    // check distance from left edge
    if (cx < rx)
        testX = rx;
    // check distance from right edge
    else if (cx > rx + rw)
        testX = rx + rw;
    // check distance from top edge
    if (cy < ry)
        testY = ry;
    // check distance from bottom edge
    else if (cy > ry + rh)
        testY = ry + rh;

    float distX = cx - testX;
    float distY = cy - testY;
    float distance = sqrt((distX * distX) + (distY * distY));

    if (distance <= radius)
        return true;
    return false;
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