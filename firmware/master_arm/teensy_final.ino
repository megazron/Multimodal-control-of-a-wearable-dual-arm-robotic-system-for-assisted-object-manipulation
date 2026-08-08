#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

static constexpr int      JOINTS           = 7;
static constexpr float    DEG_MAX          = 360.0f;
static constexpr int      ALPHA_FP         = 64;
static constexpr float    CHANGE_THRESHOLD = 0.5f;
static constexpr uint32_t LOOP_MS          = 10;
static constexpr int      ADC_MIN_VAL      = 50;
static constexpr int      ADC_MAX_VAL      = 4045;

// ============================================================================
// PIN MAP -- fixed to avoid the I2C conflict.
//
// Teensy 4.1's default I2C bus (Wire), used here for BOTH MPU6050 IMUs,
// runs on pins 18/19 -- which are exactly A4 and A5. The previous version
// of this file put A4/A5 inside potPinsK1, meaning two pot channels were
// physically sharing pins with the IMU data bus. Fixed by excluding A4/A5
// entirely from analog use.
//
// Teensy 4.1 has 18 analog-capable pins (A0-A17). Minus the 2 reserved
// for I2C (A4, A5) leaves exactly 16 -- matching the 14 pots + 2 FSR
// channels needed, with zero pins shared or reused.
//
// Wire1 (SDA1/SCL1 = pins 16/17 = A2/A3) and Wire2 (SDA2/SCL2 = pins
// 24/25 = A10/A11) are NOT used anywhere in this code (only "Wire" is
// called), so A2/A3/A10/A11 are genuinely free/safe as plain analog pins.
//
// Physical LEFT/RIGHT board distribution: verify against the pin numbers
// silkscreened on your actual board, or the official PJRC pinout card
// (https://www.pjrc.com/teensy/card11a_rev4_web.pdf) -- grouped below by
// function (K1 / K2 / FSR) so you can route each group to whichever
// physical side makes sense once you check your board.
// ============================================================================
const int potPinsK1[JOINTS] =
{
 A0, A1, A2, A3, A6, A7, A8      // K1 (left arm) -- 7 channels, no I2C pins
};
const int potPinsK2[JOINTS] =
{
 A9, A10, A11, A12, A13, A16, A17  // K2 (right arm) -- 7 channels, no I2C pins
};

// -------- Pressure sensors (FSR) --------
const int FSR1_PIN = A14;
const int FSR2_PIN = A15;

// -------- Push buttons (toggle) --------
// 2-wire wiring: signal pin -> GND when pressed. INPUT_PULLUP means the
// pin reads HIGH when NOT pressed, LOW when pressed -- no external
// resistor needed. Digital pins 2/3 are separate from the A0-A17 analog
// pins entirely (Teensy 4.1: analog pins start at physical pin 14).
const int BUTTON1_PIN = 2;
const int BUTTON2_PIN = 3;
static constexpr uint32_t DEBOUNCE_MS = 40;

int  btn1State = 0, btn2State = 0;        // toggled 0/1 output value
int  btn1LastRaw = HIGH, btn2LastRaw = HIGH;
uint32_t btn1LastChangeMs = 0, btn2LastChangeMs = 0;

int32_t emaK1[JOINTS];
int32_t emaK2[JOINTS];
float lastDegK1[JOINTS];
float lastDegK2[JOINTS];

// -------- MPU6050 (uses Wire / pins 18,19 / A4,A5 -- now conflict-free) --------
MPU6050 imuK1(0x68);
MPU6050 imuK2(0x69);
int16_t ax1,ay1,az1,gx1,gy1,gz1;
int16_t ax2,ay2,az2,gx2,gy2,gz2;

inline int updateButtonToggle(
int pin,
int& state,
int& lastRaw,
uint32_t& lastChangeMs)
{
    int raw = digitalRead(pin);
    uint32_t now = millis();
    if(raw != lastRaw && (now - lastChangeMs) > DEBOUNCE_MS)
    {
        lastChangeMs = now;
        lastRaw = raw;
        if(raw == LOW)
        {
            state = state ? 0 : 1;
        }
    }
    return state;
}

inline int updateEMA(int pin, int32_t& emaFP)
{
    int raw = analogRead(pin);
    emaFP =
    ((int32_t)ALPHA_FP * raw +
    (256 - ALPHA_FP) * emaFP) >> 8;
    return (int)emaFP;
}

inline float adcToDeg(int adc)
{
    float t = constrain(
    (float)(adc - ADC_MIN_VAL) /
    (float)(ADC_MAX_VAL - ADC_MIN_VAL),
    0.0f,
    1.0f);
    return t * DEG_MAX;
}

void readArm(
const int pins[],
int32_t emaFP[],
float curDeg[])
{
    for(int j=0;j<JOINTS;j++)
    {
        int adc = updateEMA(
        pins[j],
        emaFP[j]);
        curDeg[j] = adcToDeg(adc);
    }
}

bool buildFrame(
float degK1[],
float degK2[],
int fsr1,
int fsr2,
int btn1,
int btn2,
char* buf,
size_t bufLen)
{
    char* p = buf;
    char* end = buf + bufLen - 1;
    for(int j=0;j<JOINTS;j++)
    {
        p += snprintf(p, end-p, "k1j%d:%.1f,", j+1, degK1[j]);
    }
    for(int j=0;j<JOINTS;j++)
    {
        p += snprintf(p, end-p, "k2j%d:%.1f,", j+1, degK2[j]);
    }
    p += snprintf(p, end-p, "K1IMU:%d,%d,%d,%d,%d,%d,", ax1,ay1,az1,gx1,gy1,gz1);
    p += snprintf(p, end-p, "K2IMU:%d,%d,%d,%d,%d,%d,", ax2,ay2,az2,gx2,gy2,gz2);
    p += snprintf(p, end-p, "fsr1:%d,fsr2:%d,", fsr1, fsr2);
    p += snprintf(p, end-p, "btn1:%d,btn2:%d", btn1, btn2);
    *p='\0';
    memcpy(lastDegK1, degK1, JOINTS*sizeof(float));
    memcpy(lastDegK2, degK2, JOINTS*sizeof(float));
    return true;
}

void setup()
{
    Serial.begin(115200);
    while(!Serial && millis()<3000){}
    analogReadResolution(12);
    analogReadAveraging(1);
    for(int j=0;j<JOINTS;j++)
    {
        emaK1[j] = (int32_t)analogRead(potPinsK1[j])<<8;
        emaK2[j] = (int32_t)analogRead(potPinsK2[j])<<8;
        lastDegK1[j]=-999;
        lastDegK2[j]=-999;
    }
    Wire.begin();
    Wire.setClock(400000);
    imuK1.initialize();
    imuK2.initialize();
    Serial.println("SYSTEM READY");
    if(imuK1.testConnection())
        Serial.println("K1 MPU OK");
    else
        Serial.println("K1 MPU FAILED");
    if(imuK2.testConnection())
        Serial.println("K2 MPU OK");
    else
        Serial.println("K2 MPU FAILED");
    Serial.println("FSR1 + FSR2 Pressure Sensors Ready");

    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    btn1LastRaw = digitalRead(BUTTON1_PIN);
    btn2LastRaw = digitalRead(BUTTON2_PIN);
    Serial.println("Buttons 1 + 2 Ready (toggle mode)");
}

void loop()
{
    static uint32_t lastMs=0;
    uint32_t now=millis();
    if(now-lastMs < LOOP_MS)
        return;
    lastMs=now;

    float curDegK1[JOINTS];
    float curDegK2[JOINTS];
    readArm(potPinsK1, emaK1, curDegK1);
    readArm(potPinsK2, emaK2, curDegK2);

    imuK1.getMotion6(&ax1,&ay1,&az1,&gx1,&gy1,&gz1);
    imuK2.getMotion6(&ax2,&ay2,&az2,&gx2,&gy2,&gz2);

    int fsr1 = analogRead(FSR1_PIN);
    int fsr2 = analogRead(FSR2_PIN);

    int btn1 = updateButtonToggle(BUTTON1_PIN, btn1State, btn1LastRaw, btn1LastChangeMs);
    int btn2 = updateButtonToggle(BUTTON2_PIN, btn2State, btn2LastRaw, btn2LastChangeMs);

    char frame[460];
    buildFrame(curDegK1, curDegK2, fsr1, fsr2, btn1, btn2, frame, sizeof(frame));
    Serial.println(frame);
}
