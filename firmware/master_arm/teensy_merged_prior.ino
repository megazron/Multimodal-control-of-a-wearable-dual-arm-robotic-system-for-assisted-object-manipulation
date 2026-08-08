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

const int potPinsK1[JOINTS] =
{
 A0,A1,A2,A3,A4,A5,A6
};
const int potPinsK2[JOINTS] =
{
 A7,A8,A9,A10,A11,A12,A13
};

// -------- Pressure sensors (FSR) --------
// FSR1 was the original single-sensor pin; FSR2 added on the next free
// analog pin. Confirm A15 is valid/free on your specific Teensy board
// (Teensy 4.1 goes up to A17; Teensy 4.0 has fewer analog pins -- if
// you're on a 4.0, swap FSR2_PIN for whatever analog pin is actually free).
const int FSR1_PIN = A14;
const int FSR2_PIN = A15;

// -------- Push buttons (toggle) --------
// 2-wire wiring: signal pin -> GND when pressed. INPUT_PULLUP means the
// pin reads HIGH when NOT pressed, LOW when pressed -- no external
// resistor needed. Separate physical pins from the A0-A15 analog pins
// used above (Teensy 4.x digital pins 0-13 are distinct from A0-A13).
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

// -------- MPU6050 --------
MPU6050 imuK1(0x68);
MPU6050 imuK2(0x69);
int16_t ax1,ay1,az1,gx1,gy1,gz1;
int16_t ax2,ay2,az2,gx2,gy2,gz2;

// Debounced toggle: returns the (possibly just-flipped) state for this
// button. Only flips on the HIGH->LOW transition (the moment it's
// pressed), not while held or on release -- matches "press once = flip".
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
        if(raw == LOW)          // HIGH->LOW = the moment it's pressed
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
        p += snprintf(
        p,
        end-p,
        "k1j%d:%.1f,",
        j+1,
        degK1[j]);
    }
    for(int j=0;j<JOINTS;j++)
    {
        p += snprintf(
        p,
        end-p,
        "k2j%d:%.1f,",
        j+1,
        degK2[j]);
    }
    p += snprintf(
    p,
    end-p,
    "K1IMU:%d,%d,%d,%d,%d,%d,",
    ax1,ay1,az1,gx1,gy1,gz1);
    p += snprintf(
    p,
    end-p,
    "K2IMU:%d,%d,%d,%d,%d,%d,",
    ax2,ay2,az2,gx2,gy2,gz2);
    p += snprintf(
    p,
    end-p,
    "fsr1:%d,fsr2:%d,",
    fsr1,
    fsr2);
    p += snprintf(
    p,
    end-p,
    "btn1:%d,btn2:%d",
    btn1,
    btn2);
    *p='\0';
    memcpy(lastDegK1,
    degK1,
    JOINTS*sizeof(float));
    memcpy(lastDegK2,
    degK2,
    JOINTS*sizeof(float));
    return true;
}

void setup()
{
    Serial.begin(115200);
    while(!Serial && millis()<3000){}
    analogReadResolution(12);   // 0-4095, applies to pots AND both FSR pins
    analogReadAveraging(1);
    for(int j=0;j<JOINTS;j++)
    {
        emaK1[j] =
        (int32_t)analogRead(potPinsK1[j])<<8;
        emaK2[j] =
        (int32_t)analogRead(potPinsK2[j])<<8;
        lastDegK1[j]=-999;
        lastDegK2[j]=-999;
    }
    // MPU setup
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
    readArm(
    potPinsK1,
    emaK1,
    curDegK1);
    readArm(
    potPinsK2,
    emaK2,
    curDegK2);

    // Read IMUs
    imuK1.getMotion6(
    &ax1,&ay1,&az1,
    &gx1,&gy1,&gz1);
    imuK2.getMotion6(
    &ax2,&ay2,&az2,
    &gx2,&gy2,&gz2);

    // Read both pressure sensors
    int fsr1 = analogRead(FSR1_PIN);
    int fsr2 = analogRead(FSR2_PIN);

    // Update both button toggles (debounced, flips on press)
    int btn1 = updateButtonToggle(
    BUTTON1_PIN, btn1State, btn1LastRaw, btn1LastChangeMs);
    int btn2 = updateButtonToggle(
    BUTTON2_PIN, btn2State, btn2LastRaw, btn2LastChangeMs);

    char frame[460];
    buildFrame(
    curDegK1,
    curDegK2,
    fsr1,
    fsr2,
    btn1,
    btn2,
    frame,
    sizeof(frame));
    Serial.println(frame);
}
