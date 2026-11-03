/*
  Jump-bar motor controller with Bluetooth (BLE) link and DS3231 RTC
  ESP32 DEV

  TEST BUILD: original firmware plus diagnostic testing tools.

  Existing BLE connection, motor operation, website messages, sensor
  thresholds and commands remain unchanged.

  Additional test commands:
    test
    test on
    test off
    test status
    test config

    test ir
    test ir stream on
    test ir stream off
    test ir rate on
    test ir rate off
    test ir reset

    test us
    test us stream on
    test us stream off
    test us stats
    test us stats on
    test us stats off
    test us reset

    test move on
    test move off
    test move status

    test ble ping
    test ble ping TEXT
    test ble burst
    test ble burst N

  DOG_CLOCK is printed whenever a dog is counted.

  Needs:
    ESP32 Arduino core (Espressif)
    AccelStepper library
    RTClib library (Adafruit)

  Board:
    ESP32 Dev Module
*/

#include <AccelStepper.h>
#include <Preferences.h>
#include <Wire.h>
#include <RTClib.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// =====================================================================
// BLUETOOTH CONFIGURATION
// =====================================================================

// These UUIDs must match src/lib/esp32-ble.ts in the web application.
#define BLE_DEVICE_NAME "JumpBar-ESP32"

#define NUS_SERVICE_UUID \
  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"

#define NUS_RX_UUID \
  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

#define NUS_TX_UUID \
  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


// =====================================================================
// MOTOR CONFIGURATION
// =====================================================================

#define STEP_PIN 26
#define DIR_PIN  27

AccelStepper motor(
  AccelStepper::DRIVER,
  STEP_PIN,
  DIR_PIN
);

Preferences preferences;

// Motor speed settings. Change them HERE: setup() uses them, and the
// website reads them back with the "config" command, so the speed it shows
// always matches the firmware.
const float MOTOR_MAX_SPEED_STEPS_PER_S = 1000.0;
const float MOTOR_ACCELERATION_STEPS_PER_S2 = 500.0;


// =====================================================================
// RTC CONFIGURATION
// =====================================================================

const uint8_t RTC_SDA_PIN = 21;
const uint8_t RTC_SCL_PIN = 22;

RTC_DS3231 rtc;

bool rtcAvailable = false;


// =====================================================================
// BLUETOOTH STATE
// =====================================================================

BLEServer *bleServer = nullptr;
BLECharacteristic *txCharacteristic = nullptr;

volatile bool bleClientConnected = false;

bool bleWasConnected = false;


// Outgoing BLE text buffer.
const size_t TX_BUFFER_SIZE = 4096;

char txBuffer[TX_BUFFER_SIZE];

size_t txHead = 0;
size_t txTail = 0;

unsigned long lastBleSendMs = 0;


// Incoming BLE command queue.
const int RX_QUEUE_SIZE = 4;

String rxQueue[RX_QUEUE_SIZE];

volatile int rxHead = 0;
volatile int rxTail = 0;

String rxPartial = "";


// =====================================================================
// DUAL SERIAL AND BLE OUTPUT
// =====================================================================

class DualOutput : public Print
{
public:
  size_t write(uint8_t c) override
  {
    Serial.write(c);

    if (bleClientConnected)
    {
      size_t next = (txHead + 1) % TX_BUFFER_SIZE;

      if (next != txTail)
      {
        txBuffer[txHead] = (char)c;
        txHead = next;
      }
    }

    return 1;
  }
};

DualOutput out;


// =====================================================================
// FORWARD DECLARATIONS
// =====================================================================

void processCommand(String command);
void printCommands();

void testCommand(const String &command);
void testOnFrame(const bool *arrived);
void testAlways();
void testMonitor();

void testMovementRequested(
  long startSteps,
  long targetSteps
);


// =====================================================================
// BLUETOOTH CALLBACKS
// =====================================================================

class ServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *server) override
  {
    bleClientConnected = true;
  }

  void onDisconnect(BLEServer *server) override
  {
    bleClientConnected = false;
  }
};


class RxCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *characteristic) override
  {
    auto value = characteristic->getValue();

    for (size_t i = 0; i < value.length(); i++)
    {
      char c = value[i];

      if (c == '\n' || c == '\r')
      {
        if (rxPartial.length() > 0)
        {
          int next = (rxHead + 1) % RX_QUEUE_SIZE;

          if (next != rxTail)
          {
            rxQueue[rxHead] = rxPartial;
            rxHead = next;
          }

          rxPartial = "";
        }
      }
      else if (rxPartial.length() < 64)
      {
        rxPartial += c;
      }
    }
  }
};


// =====================================================================
// BLUETOOTH SETUP AND SERVICE
// =====================================================================

void setupBle()
{
  BLEDevice::init(BLE_DEVICE_NAME);

  bleServer = BLEDevice::createServer();

  bleServer->setCallbacks(
    new ServerCallbacks()
  );

  BLEService *service =
    bleServer->createService(
      NUS_SERVICE_UUID
    );

  txCharacteristic =
    service->createCharacteristic(
      NUS_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
    );

  txCharacteristic->addDescriptor(
    new BLE2902()
  );

  BLECharacteristic *rxCharacteristic =
    service->createCharacteristic(
      NUS_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR
    );

  rxCharacteristic->setCallbacks(
    new RxCallbacks()
  );

  service->start();

  BLEAdvertising *advertising =
    BLEDevice::getAdvertising();

  advertising->addServiceUUID(
    NUS_SERVICE_UUID
  );

  advertising->setScanResponse(true);

  BLEDevice::startAdvertising();
}


void pumpBleOutput()
{
  if (!bleClientConnected)
  {
    txHead = 0;
    txTail = 0;

    return;
  }

  if (txHead == txTail)
  {
    return;
  }

  if ((uint32_t)(millis() - lastBleSendMs) < 10)
  {
    return;
  }

  uint8_t chunk[20];

  size_t n = 0;

  while (
    n < sizeof(chunk) &&
    txTail != txHead
  )
  {
    chunk[n++] =
      (uint8_t)txBuffer[txTail];

    txTail =
      (txTail + 1) % TX_BUFFER_SIZE;
  }

  txCharacteristic->setValue(
    chunk,
    n
  );

  txCharacteristic->notify();

  lastBleSendMs = millis();
}


void serviceBle()
{
  if (
    !bleClientConnected &&
    bleWasConnected
  )
  {
    BLEDevice::startAdvertising();
  }

  bleWasConnected =
    bleClientConnected;

  while (rxTail != rxHead)
  {
    String command =
      rxQueue[rxTail];

    rxTail =
      (rxTail + 1) % RX_QUEUE_SIZE;

    processCommand(command);
  }

  pumpBleOutput();
}


// =====================================================================
// MOTOR CALIBRATION
// =====================================================================

const long STEPS_FOR_5CM = 4968;

const float STEPS_PER_CM =
  STEPS_FOR_5CM / 5.0;


// =====================================================================
// IR TIMING GATE
// =====================================================================

const uint8_t IR_RX_PIN = 15;

const uint8_t BEAM_COUNT = 4;

const uint8_t LED_COUNT = 3;

const uint8_t LED_PINS[LED_COUNT] =
{
  5,
  18,
  19
};

const uint8_t BEAMS_NEEDED = 2;

const uint32_t DOG_COOLDOWN_MS =
  1000;

const uint32_t SYNC_MIN_US =
  1500;

const uint32_t SYNC_MAX_US =
  3500;

const uint32_t SLOT_GAP_US =
  600;

const uint32_t SLOT_PITCH_US =
  1200;

const uint32_t SAMPLE_OFFSET_US =
  300;

const uint32_t MAX_LATE_US =
  500;

const uint8_t BLOCK_AFTER_MISSES =
  3;

const uint32_t NO_SYNC_TIMEOUT_US =
  100000;


volatile uint32_t fallTime = 0;
volatile uint32_t syncEndTime = 0;
volatile bool syncFlag = false;


// Diagnostic only.
volatile uint32_t lastSyncWidthUs = 0;


bool beamBlocked[BEAM_COUNT] =
{
  false,
  false,
  false,
  false
};

uint8_t missCount[BEAM_COUNT] =
{
  0,
  0,
  0,
  0
};

bool noSignal = false;
bool hasHadSync = false;

bool gateWasBlocked = false;

uint32_t dogCount = 0;

uint32_t framesReceived = 0;
uint32_t framesSkipped = 0;

uint32_t lastFrameTime = 0;
uint32_t lastDogTime = 0;


// =====================================================================
// RUN STATE
// =====================================================================

bool runActive = false;

uint32_t runStartUs = 0;

bool crossingReported = false;

uint32_t lastReportedDogCount = 0;

bool lastReportedBlocked = false;


// =====================================================================
// SHARED ULTRASONIC STATE
// =====================================================================

volatile float leftDistanceCm =
  9999.0;

volatile float rightDistanceCm =
  9999.0;

volatile uint32_t ultrasonicCycle =
  0;

uint32_t lastReportedCycle = 0;


// =====================================================================
// IR INTERRUPT
// =====================================================================

void IRAM_ATTR receiverISR()
{
  uint32_t now = micros();

  if (digitalRead(IR_RX_PIN) == LOW)
  {
    fallTime = now;
  }
  else
  {
    uint32_t width =
      now - fallTime;

    if (
      width >= SYNC_MIN_US &&
      width <= SYNC_MAX_US
    )
    {
      syncEndTime = now;
      syncFlag = true;

      lastSyncWidthUs = width;
    }
  }
}


// =====================================================================
// IR RECEIVER FUNCTIONS
// =====================================================================

void setLeds(bool on)
{
  for (
    uint8_t i = 0;
    i < LED_COUNT;
    i++
  )
  {
    digitalWrite(
      LED_PINS[i],
      on ? HIGH : LOW
    );
  }
}


void processFrame(uint32_t syncEnd)
{
  bool arrived[BEAM_COUNT];

  for (
    uint8_t i = 0;
    i < BEAM_COUNT;
    i++
  )
  {
    uint32_t target =
      syncEnd +
      SLOT_GAP_US +
      (uint32_t)i * SLOT_PITCH_US +
      SAMPLE_OFFSET_US;

    while (
      (int32_t)(micros() - target) < 0
    )
    {
      // Wait for the middle of the slot.
    }

    arrived[i] =
      digitalRead(IR_RX_PIN) == LOW;
  }

  framesReceived++;

  lastFrameTime = micros();

  noSignal = false;
  hasHadSync = true;

  for (
    uint8_t i = 0;
    i < BEAM_COUNT;
    i++
  )
  {
    if (arrived[i])
    {
      missCount[i] = 0;
      beamBlocked[i] = false;
    }
    else
    {
      if (missCount[i] < 255)
      {
        missCount[i]++;
      }

      if (
        missCount[i] >=
        BLOCK_AFTER_MISSES
      )
      {
        beamBlocked[i] = true;
      }
    }
  }

  // Diagnostic data collection only.
  testOnFrame(arrived);
}


void updateReceiver()
{
  if (syncFlag)
  {
    uint32_t syncEnd =
      syncEndTime;

    syncFlag = false;

    if (
      (uint32_t)(micros() - syncEnd) >
      MAX_LATE_US
    )
    {
      framesSkipped++;
    }
    else
    {
      processFrame(syncEnd);
    }
  }

  if (
    hasHadSync &&
    !noSignal &&
    (uint32_t)(
      micros() - lastFrameTime
    ) > NO_SYNC_TIMEOUT_US
  )
  {
    noSignal = true;

    for (
      uint8_t i = 0;
      i < BEAM_COUNT;
      i++
    )
    {
      missCount[i] =
        BLOCK_AFTER_MISSES;

      beamBlocked[i] = true;
    }
  }
}


uint8_t countBlockedBeams()
{
  uint8_t count = 0;

  for (
    uint8_t i = 0;
    i < BEAM_COUNT;
    i++
  )
  {
    if (beamBlocked[i])
    {
      count++;
    }
  }

  return count;
}


void updateGate()
{
  bool gateBlocked =
    countBlockedBeams() >=
    BEAMS_NEEDED;

  if (gateBlocked != gateWasBlocked)
  {
    setLeds(gateBlocked);

    if (gateBlocked)
    {
      if (
        (uint32_t)(
          millis() - lastDogTime
        ) >= DOG_COOLDOWN_MS ||
        dogCount == 0
      )
      {
        dogCount++;

        lastDogTime = millis();

        Serial.print(
          "DOG PASSED | Count: "
        );

        Serial.println(dogCount);
      }
    }

    gateWasBlocked = gateBlocked;
  }
}


void printStatus()
{
  Serial.println();
  Serial.println(
    "----- GATE STATUS -----"
  );

  Serial.print("Sync: ");

  if (!hasHadSync)
  {
    Serial.println(
      "waiting for transmitter"
    );
  }
  else
  {
    Serial.println(
      noSignal ? "LOST" : "OK"
    );
  }

  for (
    uint8_t i = 0;
    i < BEAM_COUNT;
    i++
  )
  {
    Serial.print("Beam ");
    Serial.print(i + 1);
    Serial.print(": ");

    Serial.println(
      beamBlocked[i]
        ? "BLOCKED"
        : "CLEAR"
    );
  }

  Serial.print("Dogs counted: ");
  Serial.println(dogCount);

  Serial.print("Frames received: ");
  Serial.println(framesReceived);

  Serial.print("Frames skipped: ");
  Serial.println(framesSkipped);

  Serial.println(
    "-----------------------"
  );
}


// =====================================================================
// ULTRASONIC SENSOR CONFIGURATION
// =====================================================================

const int TRIG_LEFT = 2;
const int ECHO_LEFT = 4;

const int TRIG_RIGHT = 16;
const int ECHO_RIGHT = 17;

const float DETECTION_THRESHOLD_CM =
  10.0;


float readDistanceCM(
  int trigPin,
  int echoPin
)
{
  digitalWrite(trigPin, LOW);

  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration =
    pulseIn(
      echoPin,
      HIGH,
      30000
    );

  if (duration == 0)
  {
    return 9999.0;
  }

  return duration * 0.0343 / 2.0;
}


bool detected(float distanceCM)
{
  return (
    distanceCM <
    DETECTION_THRESHOLD_CM
  );
}


bool leftWasDetected = false;
bool rightWasDetected = false;


void ultrasonicLoop()
{
  float leftDistance =
    readDistanceCM(
      TRIG_LEFT,
      ECHO_LEFT
    );

  delay(50);

  float rightDistance =
    readDistanceCM(
      TRIG_RIGHT,
      ECHO_RIGHT
    );

  leftDistanceCm =
    leftDistance;

  rightDistanceCm =
    rightDistance;

  ultrasonicCycle =
    ultrasonicCycle + 1;

  bool leftDetected =
    detected(leftDistance);

  bool rightDetected =
    detected(rightDistance);

  if (
    leftDetected !=
    leftWasDetected
  )
  {
    leftWasDetected =
      leftDetected;

    Serial.print("LEFT: ");

    Serial.println(
      leftDetected
        ? "DETECTED"
        : "CLEAR"
    );
  }

  if (
    rightDetected !=
    rightWasDetected
  )
  {
    rightWasDetected =
      rightDetected;

    Serial.print("RIGHT: ");

    Serial.println(
      rightDetected
        ? "DETECTED"
        : "CLEAR"
    );
  }

  delay(100);
}


void ultrasonicTask(void *parameter)
{
  for (;;)
  {
    ultrasonicLoop();
  }
}


// =====================================================================
// WEBSITE RUN REPORTING
// =====================================================================

uint32_t runTimeUs()
{
  return micros() - runStartUs;
}


uint8_t blockedBeamMask()
{
  uint8_t mask = 0;

  for (
    uint8_t i = 0;
    i < BEAM_COUNT;
    i++
  )
  {
    if (beamBlocked[i])
    {
      mask |= (1 << i);
    }
  }

  return mask;
}


int distanceForReport(
  float distanceCM
)
{
  if (distanceCM >= 9999.0)
  {
    return -1;
  }

  return (int)(
    distanceCM + 0.5
  );
}


void reportGateEvents()
{
  if (!runActive)
  {
    lastReportedDogCount =
      dogCount;

    lastReportedBlocked =
      gateWasBlocked;

    return;
  }

  if (
    dogCount !=
    lastReportedDogCount
  )
  {
    lastReportedDogCount =
      dogCount;

    if (dogCount > 0)
    {
      out.print("GATE ");
      out.print(runTimeUs());
      out.print(" ");
      out.println(
        blockedBeamMask()
      );

      crossingReported = true;
    }
  }

  if (
    gateWasBlocked !=
    lastReportedBlocked
  )
  {
    lastReportedBlocked =
      gateWasBlocked;

    if (
      !gateWasBlocked &&
      crossingReported
    )
    {
      out.print("GATE_CLEAR ");
      out.println(runTimeUs());

      crossingReported = false;
    }
  }
}


void reportUltrasonic()
{
  if (!runActive)
  {
    return;
  }

  uint32_t cycle =
    ultrasonicCycle;

  if (
    cycle ==
    lastReportedCycle
  )
  {
    return;
  }

  lastReportedCycle =
    cycle;

  float left =
    leftDistanceCm;

  float right =
    rightDistanceCm;

  uint8_t flags =
    (detected(left) ? 1 : 0) |
    (detected(right) ? 2 : 0);

  out.print("US ");

  out.print(
    runTimeUs() / 1000
  );

  out.print(" ");

  out.print(
    distanceForReport(left)
  );

  out.print(" ");

  out.print(
    distanceForReport(right)
  );

  out.print(" ");

  out.println(flags);
}


// =====================================================================
// SENSOR REPORTS
// =====================================================================

void reportSensor(
  const String &id
)
{
  bool known = true;
  bool active = false;

  if (
    id.equalsIgnoreCase(
      "GATE-L"
    )
  )
  {
    active =
      detected(leftDistanceCm);
  }
  else if (
    id.equalsIgnoreCase(
      "GATE-R"
    )
  )
  {
    active =
      detected(rightDistanceCm);
  }
  else if (
    id.equalsIgnoreCase(
      "GATE-IR"
    )
  )
  {
    // Active while the IR beams are blocked (a dog is in the gate).
    active =
      gateWasBlocked;
  }
  else
  {
    known = false;
  }

  out.print("SENSOR ");
  out.print(id);

  out.println(
    known
      ? (
          active
            ? " active"
            : " inactive"
        )
      : " unconfigured"
  );
}


void reportAllSensors()
{
  out.println();

  out.println(
    "Reporting 3 configured sensors:"
  );

  reportSensor("GATE-L");
  reportSensor("GATE-R");
  reportSensor("GATE-IR");
}


// =====================================================================
// DS3231 RTC
// =====================================================================

const char *DAY_NAMES[7] =
{
  "Sunday",
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday"
};


void formatDateTime(
  const DateTime &dt,
  char *buffer,
  size_t size
)
{
  snprintf(
    buffer,
    size,
    "%04d-%02d-%02d %02d:%02d:%02d",
    dt.year(),
    dt.month(),
    dt.day(),
    dt.hour(),
    dt.minute(),
    dt.second()
  );
}


void setupRtc()
{
  Wire.begin(
    RTC_SDA_PIN,
    RTC_SCL_PIN
  );

  Wire.setClock(400000);

  rtcAvailable = rtc.begin();

  if (!rtcAvailable)
  {
    out.println(
      "WARNING: DS3231 RTC not found. Check SDA (21) / SCL (22) wiring."
    );

    return;
  }

  out.println(
    "DS3231 RTC found."
  );

  if (rtc.lostPower())
  {
    out.println(
      "WARNING: RTC lost power (battery flat or missing)."
    );

    out.println(
      "The time is not valid. Set it with: time set YYYY-MM-DD HH:MM:SS"
    );
  }
}


void showTime()
{
  if (!rtcAvailable)
  {
    out.println(
      "ERROR: RTC not available."
    );

    return;
  }

  DateTime now =
    rtc.now();

  char text[24];

  formatDateTime(
    now,
    text,
    sizeof(text)
  );

  out.println();

  out.print("RTC time: ");
  out.print(text);
  out.print(" (");

  out.print(
    DAY_NAMES[
      now.dayOfTheWeek()
    ]
  );

  out.println(")");

  out.print(
    "RTC temperature: "
  );

  out.print(
    rtc.getTemperature(),
    1
  );

  out.println(" C");

  if (rtc.lostPower())
  {
    out.println(
      "WARNING: RTC lost power, this time may be wrong."
    );
  }
}


void setTimeFromText(
  const String &text
)
{
  if (!rtcAvailable)
  {
    out.println(
      "ERROR: RTC not available."
    );

    return;
  }

  int y;
  int mo;
  int d;
  int h;
  int mi;
  int s;

  bool valid =
    sscanf(
      text.c_str(),
      "%d-%d-%d %d:%d:%d",
      &y,
      &mo,
      &d,
      &h,
      &mi,
      &s
    ) == 6;

  valid =
    valid &&
    y >= 2000 &&
    y <= 2099 &&
    mo >= 1 &&
    mo <= 12 &&
    d >= 1 &&
    d <= 31 &&
    h >= 0 &&
    h <= 23 &&
    mi >= 0 &&
    mi <= 59 &&
    s >= 0 &&
    s <= 59;

  if (!valid)
  {
    out.println(
      "ERROR: Invalid date/time."
    );

    out.println(
      "Example: time set 2026-10-26 14:30:00"
    );

    return;
  }

  rtc.adjust(
    DateTime(
      y,
      mo,
      d,
      h,
      mi,
      s
    )
  );

  out.println(
    "RTC time set."
  );

  showTime();
}


void setTimeFromBuild()
{
  if (!rtcAvailable)
  {
    out.println(
      "ERROR: RTC not available."
    );

    return;
  }

  rtc.adjust(
    DateTime(
      F(__DATE__),
      F(__TIME__)
    )
  );

  out.println(
    "RTC set to the sketch compile time."
  );

  showTime();
}


void reportRunClock(
  const char *label
)
{
  if (!rtcAvailable)
  {
    return;
  }

  char text[24];

  formatDateTime(
    rtc.now(),
    text,
    sizeof(text)
  );

  out.print("RUN_CLOCK ");
  out.print(label);
  out.print(" ");
  out.println(text);
}


// =====================================================================
// RUN CONTROL
// =====================================================================

void startRun()
{
  if (runActive)
  {
    out.println(
      "ERROR: A run is already active."
    );

    return;
  }

  if (motor.distanceToGo() != 0)
  {
    out.println(
      "ERROR: Wait for the bar to stop moving before starting a run."
    );

    return;
  }

  dogCount = 0;
  lastDogTime = 0;

  lastReportedDogCount = 0;

  lastReportedBlocked =
    gateWasBlocked;

  crossingReported = false;

  lastReportedCycle =
    ultrasonicCycle;

  runStartUs = micros();

  runActive = true;

  out.println("RUN started");

  reportRunClock("start");
}


void endRun()
{
  if (!runActive)
  {
    out.println(
      "ERROR: No run is active."
    );

    return;
  }

  uint32_t totalUs =
    runTimeUs();

  runActive = false;

  out.print("RUN ended ");

  out.println(
    totalUs / 1000
  );

  reportRunClock("end");
}


// =====================================================================
// MOTOR POSITION
// =====================================================================

const float MIN_POSITION_CM =
  15.0;

const float MAX_POSITION_CM =
  60.0;

const long MAX_POSITION_STEPS =
  lround(
    (
      MAX_POSITION_CM -
      MIN_POSITION_CM
    ) * STEPS_PER_CM
  );

bool motorWasMoving = false;

long lastSavedPositionSteps =
  -1;

bool restoredFromMemory =
  false;


long cmToSteps(
  float positionCm
)
{
  float distanceFromMinimum =
    positionCm -
    MIN_POSITION_CM;

  return lround(
    distanceFromMinimum *
    STEPS_PER_CM
  );
}


float stepsToCm(
  long stepPosition
)
{
  return
    MIN_POSITION_CM +
    (
      (float)stepPosition /
      STEPS_PER_CM
    );
}


bool positionWithinBounds(
  long stepPosition
)
{
  return
    stepPosition >= 0 &&
    stepPosition <=
      MAX_POSITION_STEPS;
}


void saveCurrentPosition()
{
  long currentSteps =
    motor.currentPosition();

  if (
    !positionWithinBounds(
      currentSteps
    )
  )
  {
    out.println(
      "ERROR: Current position is outside the valid range."
    );

    out.println(
      "Position was not saved."
    );

    return;
  }

  preferences.begin(
    "motor",
    false
  );

  preferences.putLong(
    "position",
    currentSteps
  );

  preferences.putBool(
    "valid",
    true
  );

  preferences.end();

  lastSavedPositionSteps =
    currentSteps;

  out.print("Position saved: ");

  out.print(
    stepsToCm(currentSteps),
    2
  );

  out.print(" cm, ");

  out.print(currentSteps);

  out.println(" steps.");
}


bool restoreSavedPosition()
{
  preferences.begin(
    "motor",
    true
  );

  bool validPosition =
    preferences.getBool(
      "valid",
      false
    );

  long savedSteps =
    preferences.getLong(
      "position",
      0
    );

  preferences.end();

  if (!validPosition)
  {
    return false;
  }

  if (
    !positionWithinBounds(
      savedSteps
    )
  )
  {
    out.println(
      "WARNING: Stored position is out of bounds."
    );

    out.println(
      "Stored position will not be used."
    );

    return false;
  }

  motor.setCurrentPosition(
    savedSteps
  );

  lastSavedPositionSteps =
    savedSteps;

  return true;
}


void forgetSavedPosition()
{
  if (motor.isRunning())
  {
    out.println(
      "ERROR: Stop the motor before clearing its position."
    );

    return;
  }

  preferences.begin(
    "motor",
    false
  );

  preferences.clear();
  preferences.end();

  restoredFromMemory = false;

  lastSavedPositionSteps = -1;

  out.println();

  out.println(
    "Saved motor position cleared."
  );

  out.println(
    "Physically move the motor to 15 cm, then enter 'zero'."
  );
}


void goToPosition(
  float requestedPositionCm
)
{
  if (
    requestedPositionCm <
      MIN_POSITION_CM ||
    requestedPositionCm >
      MAX_POSITION_CM
  )
  {
    out.println();

    out.println(
      "ERROR: Requested position is out of bounds."
    );

    out.print(
      "Requested position: "
    );

    out.print(
      requestedPositionCm,
      2
    );

    out.println(" cm");

    out.print("Allowed range: ");

    out.print(
      MIN_POSITION_CM,
      2
    );

    out.print(" cm to ");

    out.print(
      MAX_POSITION_CM,
      2
    );

    out.println(" cm");

    return;
  }

  long targetSteps =
    cmToSteps(
      requestedPositionCm
    );

  out.println();

  out.print(
    "Current calculated position: "
  );

  out.print(
    stepsToCm(
      motor.currentPosition()
    ),
    2
  );

  out.println(" cm");

  out.print("Moving to: ");

  out.print(
    requestedPositionCm,
    2
  );

  out.println(" cm");

  out.print(
    "Target step position: "
  );

  out.println(targetSteps);

  // Diagnostic timing hook.
  // This does not change the motor target or movement.
  testMovementRequested(
    motor.currentPosition(),
    targetSteps
  );

  motor.moveTo(targetSteps);

  motorWasMoving = true;
}


void setKnownPosition(
  float knownPositionCm
)
{
  if (motor.isRunning())
  {
    out.println();

    out.println(
      "ERROR: Cannot set position while the motor is moving."
    );

    out.println(
      "Send 'stop' and wait until the motor has stopped."
    );

    return;
  }

  if (
    knownPositionCm <
      MIN_POSITION_CM ||
    knownPositionCm >
      MAX_POSITION_CM
  )
  {
    out.println();

    out.println(
      "ERROR: Known position is out of bounds."
    );

    out.print(
      "Entered position: "
    );

    out.print(
      knownPositionCm,
      2
    );

    out.println(" cm");

    out.print(
      "Allowed range: "
    );

    out.print(
      MIN_POSITION_CM,
      2
    );

    out.print(" cm to ");

    out.print(
      MAX_POSITION_CM,
      2
    );

    out.println(" cm");

    return;
  }

  long knownPositionSteps =
    cmToSteps(
      knownPositionCm
    );

  motor.setCurrentPosition(
    knownPositionSteps
  );

  restoredFromMemory = false;

  motorWasMoving = false;

  out.println();

  out.println(
    "Current position reference updated."
  );

  out.print(
    "Physical position defined as: "
  );

  out.print(
    knownPositionCm,
    2
  );

  out.println(" cm");

  out.print(
    "Internal step position: "
  );

  out.println(
    knownPositionSteps
  );

  saveCurrentPosition();
}


void stopMotor()
{
  if (motor.distanceToGo() == 0)
  {
    out.println(
      "Motor is already stationary."
    );

    return;
  }

  motor.stop();

  out.println();

  out.println(
    "STOP command received."
  );

  out.println(
    "Motor is decelerating to a stop."
  );
}


void showPosition()
{
  long currentSteps =
    motor.currentPosition();

  long targetSteps =
    motor.targetPosition();

  out.println();

  out.print(
    "Current calculated position: "
  );

  out.print(
    stepsToCm(currentSteps),
    2
  );

  out.println(" cm");

  out.print(
    "Current internal position: "
  );

  out.print(currentSteps);

  out.println(" steps");

  out.print("Target position: ");

  out.print(
    stepsToCm(targetSteps),
    2
  );

  out.println(" cm");

  out.print(
    "Target internal position: "
  );

  out.print(targetSteps);

  out.println(" steps");

  if (motor.distanceToGo() == 0)
  {
    out.println(
      "Motor status: stationary"
    );
  }
  else
  {
    out.println(
      "Motor status: moving"
    );
  }

  if (restoredFromMemory)
  {
    out.println(
      "Startup reference: restored from memory"
    );
  }
  else
  {
    out.println(
      "Startup reference: manually zeroed/default"
    );
  }
}


void zeroMotorAtMinimum()
{
  if (motor.isRunning())
  {
    out.println();

    out.println(
      "ERROR: Cannot zero while the motor is moving."
    );

    out.println(
      "Send 'stop' and wait until it stops."
    );

    return;
  }

  motor.setCurrentPosition(0);

  restoredFromMemory = false;

  out.println();

  out.println(
    "Current step position set to zero."
  );

  out.println(
    "Physical position is now defined as 15 cm."
  );

  saveCurrentPosition();
}


bool isValidPositionNumber(
  String text
)
{
  if (text.length() == 0)
  {
    return false;
  }

  bool decimalFound = false;

  for (
    unsigned int i = 0;
    i < text.length();
    i++
  )
  {
    char character =
      text.charAt(i);

    if (
      character == '.' &&
      !decimalFound
    )
    {
      decimalFound = true;
    }
    else if (!isDigit(character))
    {
      return false;
    }
  }

  return true;
}


// Website: motor settings, so the app shows the real speed.
// Example: CONFIG speed_steps_s 1000 accel_steps_s2 500 steps_per_cm 993.600 min_cm 15.00 max_cm 60.00
void printMotorConfig()
{
  out.print("CONFIG speed_steps_s ");
  out.print(MOTOR_MAX_SPEED_STEPS_PER_S, 0);

  out.print(" accel_steps_s2 ");
  out.print(MOTOR_ACCELERATION_STEPS_PER_S2, 0);

  out.print(" steps_per_cm ");
  out.print(STEPS_PER_CM, 3);

  out.print(" min_cm ");
  out.print(MIN_POSITION_CM, 2);

  out.print(" max_cm ");
  out.println(MAX_POSITION_CM, 2);
}


// =====================================================================
// TESTING SYSTEM STATE
// =====================================================================

bool testEnabled = false;

bool testIrStreamEnabled = false;
bool testIrRateEnabled = false;

bool testUsStreamEnabled = false;
bool testUsStatsEnabled = false;

bool testMovementStreamEnabled = false;


uint32_t testIrStreamIntervalMs =
  250;

uint32_t testIrRateIntervalMs =
  1000;

uint32_t testUsStreamIntervalMs =
  250;

uint32_t testUsStatsIntervalMs =
  5000;

uint32_t testMovementIntervalMs =
  250;


// IR diagnostic counters.
uint8_t testLatestArrivalMask = 0;

uint32_t testRawFrameCount = 0;

uint32_t testBeamArrivalCount[
  BEAM_COUNT
] =
{
  0,
  0,
  0,
  0
};

uint32_t testLastIrStreamMs = 0;
uint32_t testLastIrRateMs = 0;

uint32_t testPreviousRawFrameCount =
  0;

uint32_t testPreviousBeamArrivalCount[
  BEAM_COUNT
] =
{
  0,
  0,
  0,
  0
};


// Ultrasonic statistics.
uint32_t testLastUsStreamMs = 0;
uint32_t testLastUsStatsMs = 0;
uint32_t testLastUsStatsCycle = 0;

uint32_t testUsSamples = 0;

uint32_t testUsLeftValid = 0;
uint32_t testUsRightValid = 0;

uint32_t testUsLeftMissing = 0;
uint32_t testUsRightMissing = 0;

double testUsLeftSum = 0.0;
double testUsRightSum = 0.0;

double testUsLeftSumSquares = 0.0;
double testUsRightSumSquares = 0.0;

float testUsLeftMinimum = 9999.0;
float testUsLeftMaximum = 0.0;

float testUsRightMinimum = 9999.0;
float testUsRightMaximum = 0.0;


// Movement diagnostics.
bool testMovementPending = false;
bool testMovementActive = false;

uint32_t testMovementCommandUs = 0;
uint32_t testMovementStartUs = 0;

uint32_t testLastMovementReportMs = 0;

long testMovementStartSteps = 0;
long testMovementTargetSteps = 0;


// DOG_CLOCK tracking.
uint32_t testClockedDogCount = 0;


// BLE test sequence.
uint32_t testBleBurstSequence = 0;


// =====================================================================
// TEST HELPERS
// =====================================================================

uint8_t testArrivalMask(
  const bool *arrived
)
{
  uint8_t mask = 0;

  for (
    uint8_t i = 0;
    i < BEAM_COUNT;
    i++
  )
  {
    if (arrived[i])
    {
      mask |= (1 << i);
    }
  }

  return mask;
}


void testPrintDistance(
  float value
)
{
  if (value >= 9999.0)
  {
    out.print("-1");
  }
  else
  {
    out.print(value, 2);
  }
}


void testResetIrCounters()
{
  testRawFrameCount = 0;

  testLatestArrivalMask = 0;

  for (
    uint8_t i = 0;
    i < BEAM_COUNT;
    i++
  )
  {
    testBeamArrivalCount[i] = 0;

    testPreviousBeamArrivalCount[i] =
      0;
  }

  testPreviousRawFrameCount = 0;

  testLastIrRateMs = millis();
}


void testResetUsStatistics()
{
  testUsSamples = 0;

  testUsLeftValid = 0;
  testUsRightValid = 0;

  testUsLeftMissing = 0;
  testUsRightMissing = 0;

  testUsLeftSum = 0.0;
  testUsRightSum = 0.0;

  testUsLeftSumSquares = 0.0;
  testUsRightSumSquares = 0.0;

  testUsLeftMinimum = 9999.0;
  testUsLeftMaximum = 0.0;

  testUsRightMinimum = 9999.0;
  testUsRightMaximum = 0.0;

  testLastUsStatsCycle =
    ultrasonicCycle;

  testLastUsStatsMs =
    millis();
}


float testStandardDeviation(
  double sum,
  double sumSquares,
  uint32_t count
)
{
  if (count < 2)
  {
    return 0.0;
  }

  double mean =
    sum / count;

  double variance =
    (
      sumSquares / count
    ) -
    (
      mean * mean
    );

  if (variance < 0.0)
  {
    variance = 0.0;
  }

  return sqrt(variance);
}


// =====================================================================
// IR TEST DATA COLLECTION
// =====================================================================

void testOnFrame(
  const bool *arrived
)
{
  uint8_t mask =
    testArrivalMask(arrived);

  testLatestArrivalMask =
    mask;

  testRawFrameCount++;

  for (
    uint8_t i = 0;
    i < BEAM_COUNT;
    i++
  )
  {
    if (arrived[i])
    {
      testBeamArrivalCount[i]++;
    }
  }
}


// =====================================================================
// DOG CLOCK
// =====================================================================

void testPrintDogClock(
  uint32_t count
)
{
  out.print("DOG_CLOCK ");

  out.print(count);

  out.print(" ");

  if (rtcAvailable)
  {
    char text[24];

    formatDateTime(
      rtc.now(),
      text,
      sizeof(text)
    );

    out.print(text);
  }
  else
  {
    out.print(
      "RTC_UNAVAILABLE"
    );
  }

  out.print(" uptime_ms ");

  out.print(millis());

  out.print(" run_ms ");

  if (runActive)
  {
    out.println(
      runTimeUs() / 1000
    );
  }
  else
  {
    out.println("-1");
  }
}


// =====================================================================
// MOVEMENT TESTING
// =====================================================================

void testMovementRequested(
  long startSteps,
  long targetSteps
)
{
  testMovementPending = true;

  testMovementActive = false;

  testMovementCommandUs =
    micros();

  testMovementStartUs = 0;

  testMovementStartSteps =
    startSteps;

  testMovementTargetSteps =
    targetSteps;

  testLastMovementReportMs =
    0;
}


void testUpdateMovement()
{
  if (
    testMovementPending &&
    motor.distanceToGo() != 0
  )
  {
    testMovementPending = false;

    testMovementActive = true;

    testMovementStartUs =
      micros();

    if (
      testEnabled &&
      testMovementStreamEnabled
    )
    {
      out.print(
        "TEST MOVE START start_steps "
      );

      out.print(
        testMovementStartSteps
      );

      out.print(
        " target_steps "
      );

      out.print(
        testMovementTargetSteps
      );

      out.print(" start_cm ");

      out.print(
        stepsToCm(
          testMovementStartSteps
        ),
        2
      );

      out.print(" target_cm ");

      out.println(
        stepsToCm(
          testMovementTargetSteps
        ),
        2
      );
    }
  }

  if (
    testMovementActive &&
    motor.distanceToGo() == 0
  )
  {
    uint32_t finishUs =
      micros();

    uint32_t movementUs =
      finishUs -
      testMovementStartUs;

    uint32_t commandToFinishUs =
      finishUs -
      testMovementCommandUs;

    long travelledSteps =
      motor.currentPosition() -
      testMovementStartSteps;

    float travelledCm =
      fabs(
        (float)travelledSteps /
        STEPS_PER_CM
      );

    float elapsedSeconds =
      movementUs / 1000000.0;

    float averageMmPerSecond =
      0.0;

    if (elapsedSeconds > 0.0)
    {
      averageMmPerSecond =
        (
          travelledCm * 10.0
        ) /
        elapsedSeconds;
    }

    out.print(
      "TEST MOVE END start_steps "
    );

    out.print(
      testMovementStartSteps
    );

    out.print(
      " target_steps "
    );

    out.print(
      testMovementTargetSteps
    );

    out.print(
      " final_steps "
    );

    out.print(
      motor.currentPosition()
    );

    out.print(" elapsed_ms ");

    out.print(
      movementUs / 1000
    );

    out.print(
      " command_to_finish_ms "
    );

    out.print(
      commandToFinishUs / 1000
    );

    out.print(" distance_cm ");

    out.print(
      travelledCm,
      3
    );

    out.print(
      " average_mm_s "
    );

    out.println(
      averageMmPerSecond,
      3
    );

    testMovementActive = false;
  }
}


// =====================================================================
// ALWAYS-ACTIVE TEST FUNCTIONS
// =====================================================================

void testAlways()
{
  if (
    dogCount <
    testClockedDogCount
  )
  {
    testClockedDogCount =
      dogCount;
  }
  else
  {
    while (
      testClockedDogCount <
      dogCount
    )
    {
      testClockedDogCount++;

      testPrintDogClock(
        testClockedDogCount
      );
    }
  }

  testUpdateMovement();
}


// =====================================================================
// IR TEST REPORTS
// =====================================================================

void testPrintIrSnapshot()
{
  out.print(
    "TEST IR uptime_ms "
  );

  out.print(millis());

  out.print(" sync ");

  if (!hasHadSync)
  {
    out.print("WAITING");
  }
  else
  {
    out.print(
      noSignal
        ? "LOST"
        : "OK"
    );
  }

  out.print(
    " sync_width_us "
  );

  out.print(
    lastSyncWidthUs
  );

  out.print(" frame ");

  out.print(
    testRawFrameCount
  );

  out.print(
    " arrived_mask "
  );

  out.print(
    testLatestArrivalMask
  );

  out.print(
    " blocked_mask "
  );

  out.print(
    blockedBeamMask()
  );

  out.print(" misses ");

  for (
    uint8_t i = 0;
    i < BEAM_COUNT;
    i++
  )
  {
    if (i > 0)
    {
      out.print(",");
    }

    out.print(
      missCount[i]
    );
  }

  out.print(" skipped ");

  out.println(
    framesSkipped
  );
}


void testPrintIrRates()
{
  uint32_t now =
    millis();

  uint32_t elapsedMs =
    now -
    testLastIrRateMs;

  if (elapsedMs == 0)
  {
    elapsedMs = 1;
  }

  uint32_t frameCount =
    testRawFrameCount;

  uint32_t frameDelta =
    frameCount -
    testPreviousRawFrameCount;

  float frameRate =
    (
      (float)frameDelta *
      1000.0
    ) /
    elapsedMs;

  out.print(
    "TEST IR_RATE window_ms "
  );

  out.print(elapsedMs);

  out.print(" frames ");

  out.print(frameDelta);

  out.print(" frame_hz ");

  out.print(
    frameRate,
    2
  );

  for (
    uint8_t i = 0;
    i < BEAM_COUNT;
    i++
  )
  {
    uint32_t beamCount =
      testBeamArrivalCount[i];

    uint32_t beamDelta =
      beamCount -
      testPreviousBeamArrivalCount[i];

    float beamRate =
      (
        (float)beamDelta *
        1000.0
      ) /
      elapsedMs;

    out.print(" beam");

    out.print(i + 1);

    out.print("_hz ");

    out.print(
      beamRate,
      2
    );

    testPreviousBeamArrivalCount[i] =
      beamCount;
  }

  out.println();

  testPreviousRawFrameCount =
    frameCount;

  testLastIrRateMs =
    now;
}


// =====================================================================
// ULTRASONIC TEST REPORTS
// =====================================================================

void testPrintUltrasonicSnapshot()
{
  float left =
    leftDistanceCm;

  float right =
    rightDistanceCm;

  uint8_t flags =
    (detected(left) ? 1 : 0) |
    (detected(right) ? 2 : 0);

  out.print(
    "TEST US uptime_ms "
  );

  out.print(millis());

  out.print(" cycle ");

  out.print(
    ultrasonicCycle
  );

  out.print(" left_cm ");

  testPrintDistance(left);

  out.print(" right_cm ");

  testPrintDistance(right);

  out.print(" flags ");

  out.println(flags);
}


void testCollectUltrasonicStatistics()
{
  uint32_t cycle =
    ultrasonicCycle;

  if (
    cycle ==
    testLastUsStatsCycle
  )
  {
    return;
  }

  testLastUsStatsCycle =
    cycle;

  float left =
    leftDistanceCm;

  float right =
    rightDistanceCm;

  testUsSamples++;

  if (left >= 9999.0)
  {
    testUsLeftMissing++;
  }
  else
  {
    testUsLeftValid++;

    testUsLeftSum += left;

    testUsLeftSumSquares +=
      (double)left * left;

    if (
      left <
      testUsLeftMinimum
    )
    {
      testUsLeftMinimum =
        left;
    }

    if (
      left >
      testUsLeftMaximum
    )
    {
      testUsLeftMaximum =
        left;
    }
  }

  if (right >= 9999.0)
  {
    testUsRightMissing++;
  }
  else
  {
    testUsRightValid++;

    testUsRightSum += right;

    testUsRightSumSquares +=
      (double)right * right;

    if (
      right <
      testUsRightMinimum
    )
    {
      testUsRightMinimum =
        right;
    }

    if (
      right >
      testUsRightMaximum
    )
    {
      testUsRightMaximum =
        right;
    }
  }
}


void testPrintUltrasonicStatistics()
{
  float leftMean = 0.0;
  float rightMean = 0.0;

  if (testUsLeftValid > 0)
  {
    leftMean =
      testUsLeftSum /
      testUsLeftValid;
  }

  if (testUsRightValid > 0)
  {
    rightMean =
      testUsRightSum /
      testUsRightValid;
  }

  float leftSd =
    testStandardDeviation(
      testUsLeftSum,
      testUsLeftSumSquares,
      testUsLeftValid
    );

  float rightSd =
    testStandardDeviation(
      testUsRightSum,
      testUsRightSumSquares,
      testUsRightValid
    );

  out.print(
    "TEST US_STATS samples "
  );

  out.print(
    testUsSamples
  );

  out.print(" left_valid ");

  out.print(
    testUsLeftValid
  );

  out.print(
    " left_missing "
  );

  out.print(
    testUsLeftMissing
  );

  out.print(
    " left_mean_cm "
  );

  out.print(
    leftMean,
    3
  );

  out.print(" left_sd_cm ");

  out.print(
    leftSd,
    3
  );

  out.print(
    " left_min_cm "
  );

  if (testUsLeftValid > 0)
  {
    out.print(
      testUsLeftMinimum,
      2
    );
  }
  else
  {
    out.print("-1");
  }

  out.print(
    " left_max_cm "
  );

  if (testUsLeftValid > 0)
  {
    out.print(
      testUsLeftMaximum,
      2
    );
  }
  else
  {
    out.print("-1");
  }

  out.print(
    " right_valid "
  );

  out.print(
    testUsRightValid
  );

  out.print(
    " right_missing "
  );

  out.print(
    testUsRightMissing
  );

  out.print(
    " right_mean_cm "
  );

  out.print(
    rightMean,
    3
  );

  out.print(
    " right_sd_cm "
  );

  out.print(
    rightSd,
    3
  );

  out.print(
    " right_min_cm "
  );

  if (testUsRightValid > 0)
  {
    out.print(
      testUsRightMinimum,
      2
    );
  }
  else
  {
    out.print("-1");
  }

  out.print(
    " right_max_cm "
  );

  if (testUsRightValid > 0)
  {
    out.println(
      testUsRightMaximum,
      2
    );
  }
  else
  {
    out.println("-1");
  }
}


// =====================================================================
// BLE TESTS
// =====================================================================

void testBlePing(
  const String &argument
)
{
  out.print(
    "TEST BLE PONG uptime_ms "
  );

  out.print(millis());

  if (argument.length() > 0)
  {
    out.print(" data ");
    out.print(argument);
  }

  out.println();
}


void testBleBurst(
  int requestedLines
)
{
  if (requestedLines < 1)
  {
    requestedLines = 1;
  }

  if (requestedLines > 40)
  {
    requestedLines = 40;
  }

  testBleBurstSequence++;

  out.print(
    "TEST BLE_BURST START sequence "
  );

  out.print(
    testBleBurstSequence
  );

  out.print(" lines ");

  out.println(
    requestedLines
  );

  for (
    int i = 1;
    i <= requestedLines;
    i++
  )
  {
    out.print(
      "TEST BLE_BURST "
    );

    out.print(
      testBleBurstSequence
    );

    out.print(" ");

    out.print(i);

    out.print("/");

    out.print(
      requestedLines
    );

    out.print(
      " abcdefghijklmnopqrstuvwxyz0123456789"
    );

    out.println();
  }

  out.print(
    "TEST BLE_BURST END sequence "
  );

  out.println(
    testBleBurstSequence
  );
}


// =====================================================================
// TEST CONFIGURATION
// =====================================================================

void testPrintConfiguration()
{
  out.println();

  out.println(
    "----- TEST CONFIGURATION -----"
  );

  out.print(
    "Firmware build date: "
  );

  out.print(__DATE__);

  out.print(" ");

  out.println(__TIME__);

  out.print(
    "BLE device name: "
  );

  out.println(
    BLE_DEVICE_NAME
  );

  out.print(
    "BLE connected: "
  );

  out.println(
    bleClientConnected
      ? "yes"
      : "no"
  );

  out.print(
    "RTC available: "
  );

  out.println(
    rtcAvailable
      ? "yes"
      : "no"
  );

  out.print("RTC SDA pin: ");
  out.println(RTC_SDA_PIN);

  out.print("RTC SCL pin: ");
  out.println(RTC_SCL_PIN);

  out.print(
    "Stepper STEP pin: "
  );

  out.println(STEP_PIN);

  out.print(
    "Stepper DIR pin: "
  );

  out.println(DIR_PIN);

  out.print(
    "Stepper maximum speed: "
  );

  out.print((int)MOTOR_MAX_SPEED_STEPS_PER_S);

  out.println(" steps/s");

  out.print(
    "Stepper acceleration: "
  );

  out.print((int)MOTOR_ACCELERATION_STEPS_PER_S2);

  out.println(" steps/s^2");

  out.print("Steps per cm: ");

  out.println(
    STEPS_PER_CM,
    3
  );

  out.print(
    "Minimum position: "
  );

  out.print(
    MIN_POSITION_CM,
    2
  );

  out.println(" cm");

  out.print(
    "Maximum position: "
  );

  out.print(
    MAX_POSITION_CM,
    2
  );

  out.println(" cm");

  out.print(
    "IR receiver pin: "
  );

  out.println(IR_RX_PIN);

  out.print("IR beams: ");

  out.println(BEAM_COUNT);

  out.print(
    "Beams needed for crossing: "
  );

  out.println(
    BEAMS_NEEDED
  );

  out.print(
    "Dog cooldown: "
  );

  out.print(
    DOG_COOLDOWN_MS
  );

  out.println(" ms");

  out.print(
    "No-sync timeout: "
  );

  out.print(
    NO_SYNC_TIMEOUT_US
  );

  out.println(" us");

  out.print(
    "Block after misses: "
  );

  out.println(
    BLOCK_AFTER_MISSES
  );

  out.print(
    "Ultrasonic threshold: "
  );

  out.print(
    DETECTION_THRESHOLD_CM,
    2
  );

  out.println(" cm");

  out.print(
    "Left ultrasonic TRIG/ECHO: "
  );

  out.print(TRIG_LEFT);
  out.print("/");
  out.println(ECHO_LEFT);

  out.print(
    "Right ultrasonic TRIG/ECHO: "
  );

  out.print(TRIG_RIGHT);
  out.print("/");
  out.println(ECHO_RIGHT);

  out.print("Run active: ");

  out.println(
    runActive
      ? "yes"
      : "no"
  );

  out.print(
    "Current motor position: "
  );

  out.print(
    stepsToCm(
      motor.currentPosition()
    ),
    2
  );

  out.print(" cm, ");

  out.print(
    motor.currentPosition()
  );

  out.println(" steps");

  out.print("Free heap: ");

  out.print(
    ESP.getFreeHeap()
  );

  out.println(" bytes");

  out.println(
    "------------------------------"
  );
}


// =====================================================================
// TEST STATUS AND COMMAND LIST
// =====================================================================

void testPrintStatus()
{
  out.println();

  out.println(
    "----- TEST STATUS -----"
  );

  out.print("Test monitor: ");

  out.println(
    testEnabled
      ? "ON"
      : "OFF"
  );

  out.print(
    "IR raw stream: "
  );

  out.println(
    testIrStreamEnabled
      ? "ON"
      : "OFF"
  );

  out.print(
    "IR rate stream: "
  );

  out.println(
    testIrRateEnabled
      ? "ON"
      : "OFF"
  );

  out.print(
    "Ultrasonic raw stream: "
  );

  out.println(
    testUsStreamEnabled
      ? "ON"
      : "OFF"
  );

  out.print(
    "Ultrasonic statistics: "
  );

  out.println(
    testUsStatsEnabled
      ? "ON"
      : "OFF"
  );

  out.print(
    "Movement stream: "
  );

  out.println(
    testMovementStreamEnabled
      ? "ON"
      : "OFF"
  );

  out.println(
    "DOG_CLOCK: ALWAYS ON"
  );

  out.print(
    "IR frames collected: "
  );

  out.println(
    testRawFrameCount
  );

  out.print(
    "Ultrasonic samples collected: "
  );

  out.println(
    testUsSamples
  );

  out.println(
    "-----------------------"
  );
}


void testPrintCommands()
{
  out.println();

  out.println("Test commands:");

  out.println(
    "  test                    : show this list"
  );

  out.println(
    "  test on                 : enable requested diagnostic streams"
  );

  out.println(
    "  test off                : disable all diagnostic streams"
  );

  out.println(
    "  test status             : show diagnostic settings"
  );

  out.println(
    "  test config             : print firmware and pin configuration"
  );

  out.println();

  out.println(
    "  test ir                 : print one raw IR snapshot"
  );

  out.println(
    "  test ir stream on       : enable raw IR stream"
  );

  out.println(
    "  test ir stream off      : disable raw IR stream"
  );

  out.println(
    "  test ir rate on         : enable per-beam rates"
  );

  out.println(
    "  test ir rate off        : disable per-beam rates"
  );

  out.println(
    "  test ir reset           : reset IR diagnostic counters"
  );

  out.println();

  out.println(
    "  test us                 : print one ultrasonic reading"
  );

  out.println(
    "  test us stream on       : enable raw ultrasonic stream"
  );

  out.println(
    "  test us stream off      : disable raw ultrasonic stream"
  );

  out.println(
    "  test us stats           : print ultrasonic statistics"
  );

  out.println(
    "  test us stats on        : collect and print statistics"
  );

  out.println(
    "  test us stats off       : stop statistics output"
  );

  out.println(
    "  test us reset           : reset ultrasonic statistics"
  );

  out.println();

  out.println(
    "  test move on            : enable movement progress"
  );

  out.println(
    "  test move off           : disable movement progress"
  );

  out.println(
    "  test move status        : show current movement state"
  );

  out.println();

  out.println(
    "  test ble ping [text]    : return a BLE/serial ping"
  );

  out.println(
    "  test ble burst [1-40]   : send numbered test lines"
  );

  out.println();

  out.println(
    "DOG_CLOCK is always enabled."
  );

  out.println();
}


// =====================================================================
// TEST COMMAND PARSER
// =====================================================================

void testCommand(
  const String &originalCommand
)
{
  String command =
    originalCommand;

  command.trim();
  command.toLowerCase();

  if (command == "test")
  {
    testPrintCommands();
  }
  else if (
    command == "test on"
  )
  {
    testEnabled = true;

    out.println(
      "TEST monitoring enabled."
    );
  }
  else if (
    command == "test off"
  )
  {
    testEnabled = false;

    testIrStreamEnabled = false;
    testIrRateEnabled = false;

    testUsStreamEnabled = false;
    testUsStatsEnabled = false;

    testMovementStreamEnabled =
      false;

    out.println(
      "TEST monitoring and all diagnostic streams disabled."
    );

    out.println(
      "DOG_CLOCK remains enabled."
    );
  }
  else if (
    command == "test status"
  )
  {
    testPrintStatus();
  }
  else if (
    command == "test config"
  )
  {
    testPrintConfiguration();
  }
  else if (
    command == "test ir"
  )
  {
    testPrintIrSnapshot();
  }
  else if (
    command ==
    "test ir stream on"
  )
  {
    testIrStreamEnabled =
      true;

    out.println(
      "TEST IR raw stream armed."
    );

    out.println(
      "Send 'test on' to allow periodic output."
    );
  }
  else if (
    command ==
    "test ir stream off"
  )
  {
    testIrStreamEnabled =
      false;

    out.println(
      "TEST IR raw stream disabled."
    );
  }
  else if (
    command ==
    "test ir rate on"
  )
  {
    testIrRateEnabled =
      true;

    testLastIrRateMs =
      millis();

    testPreviousRawFrameCount =
      testRawFrameCount;

    for (
      uint8_t i = 0;
      i < BEAM_COUNT;
      i++
    )
    {
      testPreviousBeamArrivalCount[i] =
        testBeamArrivalCount[i];
    }

    out.println(
      "TEST IR rate stream armed."
    );

    out.println(
      "Send 'test on' to allow periodic output."
    );
  }
  else if (
    command ==
    "test ir rate off"
  )
  {
    testIrRateEnabled =
      false;

    out.println(
      "TEST IR rate stream disabled."
    );
  }
  else if (
    command ==
    "test ir reset"
  )
  {
    testResetIrCounters();

    out.println(
      "TEST IR counters reset."
    );
  }
  else if (
    command == "test us"
  )
  {
    testPrintUltrasonicSnapshot();
  }
  else if (
    command ==
    "test us stream on"
  )
  {
    testUsStreamEnabled =
      true;

    out.println(
      "TEST ultrasonic raw stream armed."
    );

    out.println(
      "Send 'test on' to allow periodic output."
    );
  }
  else if (
    command ==
    "test us stream off"
  )
  {
    testUsStreamEnabled =
      false;

    out.println(
      "TEST ultrasonic raw stream disabled."
    );
  }
  else if (
    command ==
    "test us stats"
  )
  {
    testPrintUltrasonicStatistics();
  }
  else if (
    command ==
    "test us stats on"
  )
  {
    testUsStatsEnabled =
      true;

    testLastUsStatsCycle =
      ultrasonicCycle;

    out.println(
      "TEST ultrasonic statistics armed."
    );

    out.println(
      "Send 'test on' to collect and print periodically."
    );
  }
  else if (
    command ==
    "test us stats off"
  )
  {
    testUsStatsEnabled =
      false;

    out.println(
      "TEST ultrasonic statistics disabled."
    );
  }
  else if (
    command ==
    "test us reset"
  )
  {
    testResetUsStatistics();

    out.println(
      "TEST ultrasonic statistics reset."
    );
  }
  else if (
    command ==
    "test move on"
  )
  {
    testMovementStreamEnabled =
      true;

    out.println(
      "TEST movement stream armed."
    );

    out.println(
      "Send 'test on' to allow progress output."
    );
  }
  else if (
    command ==
    "test move off"
  )
  {
    testMovementStreamEnabled =
      false;

    out.println(
      "TEST movement stream disabled."
    );
  }
  else if (
    command ==
    "test move status"
  )
  {
    out.print(
      "TEST MOVE STATUS current_steps "
    );

    out.print(
      motor.currentPosition()
    );

    out.print(
      " target_steps "
    );

    out.print(
      motor.targetPosition()
    );

    out.print(
      " distance_to_go "
    );

    out.print(
      motor.distanceToGo()
    );

    out.print(" running ");

    out.println(
      motor.isRunning()
        ? "yes"
        : "no"
    );
  }
  else if (
    command ==
    "test ble ping"
  )
  {
    testBlePing("");
  }
  else if (
    command.startsWith(
      "test ble ping "
    )
  )
  {
    String argument =
      originalCommand.substring(14);

    argument.trim();

    testBlePing(argument);
  }
  else if (
    command ==
    "test ble burst"
  )
  {
    testBleBurst(10);
  }
  else if (
    command.startsWith(
      "test ble burst "
    )
  )
  {
    String countText =
      command.substring(15);

    countText.trim();

    int lineCount =
      countText.toInt();

    if (
      lineCount < 1 ||
      lineCount > 40
    )
    {
      out.println(
        "ERROR: BLE burst line count must be from 1 to 40."
      );

      out.println(
        "Example: test ble burst 20"
      );

      return;
    }

    testBleBurst(lineCount);
  }
  else
  {
    out.print(
      "ERROR: Unknown test command: "
    );

    out.println(
      originalCommand
    );

    out.println(
      "Enter 'test' to display the test commands."
    );
  }
}


// =====================================================================
// PERIODIC TEST MONITOR
// =====================================================================

void testMonitor()
{
  if (!testEnabled)
  {
    return;
  }

  uint32_t now =
    millis();

  if (
    testIrStreamEnabled &&
    (uint32_t)(
      now -
      testLastIrStreamMs
    ) >= testIrStreamIntervalMs
  )
  {
    testLastIrStreamMs =
      now;

    testPrintIrSnapshot();
  }

  if (
    testIrRateEnabled &&
    (uint32_t)(
      now -
      testLastIrRateMs
    ) >= testIrRateIntervalMs
  )
  {
    testPrintIrRates();
  }

  if (
    testUsStreamEnabled &&
    (uint32_t)(
      now -
      testLastUsStreamMs
    ) >= testUsStreamIntervalMs
  )
  {
    testLastUsStreamMs =
      now;

    testPrintUltrasonicSnapshot();
  }

  if (testUsStatsEnabled)
  {
    testCollectUltrasonicStatistics();

    if (
      (uint32_t)(
        now -
        testLastUsStatsMs
      ) >= testUsStatsIntervalMs
    )
    {
      testLastUsStatsMs =
        now;

      testPrintUltrasonicStatistics();
    }
  }

  if (
    testMovementStreamEnabled &&
    testMovementActive &&
    (uint32_t)(
      now -
      testLastMovementReportMs
    ) >= testMovementIntervalMs
  )
  {
    testLastMovementReportMs =
      now;

    out.print(
      "TEST MOVE PROGRESS elapsed_ms "
    );

    out.print(
      (
        micros() -
        testMovementStartUs
      ) / 1000
    );

    out.print(
      " current_steps "
    );

    out.print(
      motor.currentPosition()
    );

    out.print(
      " target_steps "
    );

    out.print(
      motor.targetPosition()
    );

    out.print(
      " distance_to_go "
    );

    out.print(
      motor.distanceToGo()
    );

    out.print(
      " current_cm "
    );

    out.println(
      stepsToCm(
        motor.currentPosition()
      ),
      2
    );
  }
}


// =====================================================================
// MAIN COMMAND PARSER
// =====================================================================

void processCommand(
  String command
)
{
  command.trim();

  String originalCommand =
    command;

  command.toLowerCase();

  if (command == "stop")
  {
    stopMotor();
  }
  else if (
    command == "position"
  )
  {
    showPosition();
  }
  else if (
    command == "sensors"
  )
  {
    reportAllSensors();
  }
  else if (
    command == "run start"
  )
  {
    startRun();
  }
  else if (
    command == "run end"
  )
  {
    endRun();
  }
  else if (
    command == "status"
  )
  {
    printStatus();
  }
  else if (
    command == "reset"
  )
  {
    dogCount = 0;

    framesReceived = 0;
    framesSkipped = 0;

    testClockedDogCount = 0;

    Serial.println(
      "Counts reset."
    );
  }
  else if (
    command.startsWith(
      "sensor "
    )
  )
  {
    String sensorIdText =
      command.substring(7);

    sensorIdText.trim();

    if (
      sensorIdText.length() == 0
    )
    {
      out.println(
        "ERROR: Missing sensor id."
      );

      out.println(
        "Example: sensor GATE-L"
      );

      return;
    }

    reportSensor(
      sensorIdText
    );
  }
  else if (
    command == "zero"
  )
  {
    zeroMotorAtMinimum();
  }
  else if (
    command == "save"
  )
  {
    if (motor.isRunning())
    {
      out.println(
        "ERROR: Wait for the motor to stop before saving."
      );
    }
    else
    {
      saveCurrentPosition();
    }
  }
  else if (
    command == "forget"
  )
  {
    forgetSavedPosition();
  }
  else if (
    command.startsWith(
      "set "
    )
  )
  {
    String positionText =
      command.substring(4);

    positionText.trim();

    if (
      !isValidPositionNumber(
        positionText
      )
    )
    {
      out.println(
        "ERROR: Invalid position."
      );

      out.println(
        "Example: set 32"
      );

      return;
    }

    float knownPositionCm =
      positionText.toFloat();

    setKnownPosition(
      knownPositionCm
    );
  }
  else if (
    command == "help"
  )
  {
    printCommands();
  }
  else if (
    command.startsWith(
      "go "
    )
  )
  {
    if (runActive)
    {
      out.println(
        "ERROR: End the run before moving the bar."
      );

      return;
    }

    String positionText =
      command.substring(3);

    positionText.trim();

    if (
      !isValidPositionNumber(
        positionText
      )
    )
    {
      out.println(
        "ERROR: Invalid position."
      );

      out.println(
        "Example: go 40"
      );

      return;
    }

    float requestedPositionCm =
      positionText.toFloat();

    goToPosition(
      requestedPositionCm
    );
  }
  else if (
    command == "config"
  )
  {
    printMotorConfig();
  }
  else if (
    command == "time"
  )
  {
    showTime();
  }
  else if (
    command == "time build"
  )
  {
    setTimeFromBuild();
  }
  else if (
    command.startsWith(
      "time set "
    )
  )
  {
    setTimeFromText(
      command.substring(9)
    );
  }
  else if (
    command == "test" ||
    command.startsWith(
      "test "
    )
  )
  {
    testCommand(
      originalCommand
    );
  }
  else if (
    command.length() > 0
  )
  {
    out.print(
      "ERROR: Unknown command: "
    );

    out.println(command);

    out.println(
      "Enter 'help' to display the commands."
    );
  }
}


// =====================================================================
// NORMAL COMMAND LIST
// =====================================================================

void printCommands()
{
  out.println();

  out.println(
    "Available commands:"
  );

  out.println(
    "  go X      : move to X cm"
  );

  out.println(
    "  go 15     : move to the lower position"
  );

  out.println(
    "  go 60     : move to the upper position"
  );

  out.println(
    "  position  : show current and target positions"
  );

  out.println(
    "  stop      : decelerate and stop"
  );

  out.println(
    "  save      : save the current position manually"
  );

  out.println(
    "  zero      : define current physical position as 15 cm"
  );

  out.println(
    "  set X     : define the current physical position as X cm"
  );

  out.println(
    "  forget    : erase the saved position"
  );

  out.println(
    "  run start : start a timed run (bar must be stationary)"
  );

  out.println(
    "  run end   : end the run; no more timing is reported"
  );

  out.println(
    "  status    : show gate beams and counts"
  );

  out.println(
    "  reset     : reset the gate counts"
  );

  out.println(
    "  sensors   : report the left, right and IR gate sensors"
  );

  out.println(
    "  sensor X  : report one sensor: GATE-L, GATE-R or GATE-IR"
  );

  out.println(
    "  config    : show the motor speed settings"
  );

  out.println(
    "  time      : show the RTC date, time and temperature"
  );

  out.println(
    "  time set YYYY-MM-DD HH:MM:SS : set the RTC"
  );

  out.println(
    "  time build: set the RTC to the sketch compile time"
  );

  out.println(
    "  test      : show the testing commands"
  );

  out.println(
    "  help      : show this command list"
  );

  out.println();
}


// =====================================================================
// SETUP
// =====================================================================

void setup()
{
  Serial.begin(115200);

  Serial.setTimeout(50);

  motor.setMinPulseWidth(3);

  motor.setMaxSpeed(MOTOR_MAX_SPEED_STEPS_PER_S);

  motor.setAcceleration(MOTOR_ACCELERATION_STEPS_PER_S2);

  setupBle();

  delay(1000);


  // IR gate setup.
  pinMode(
    IR_RX_PIN,
    INPUT_PULLUP
  );

  for (
    uint8_t i = 0;
    i < LED_COUNT;
    i++
  )
  {
    pinMode(
      LED_PINS[i],
      OUTPUT
    );
  }

  setLeds(false);

  lastFrameTime =
    micros();

  attachInterrupt(
    digitalPinToInterrupt(
      IR_RX_PIN
    ),
    receiverISR,
    CHANGE
  );

  Serial.println();

  Serial.println(
    "ESP32 dog timing gate ready."
  );

  Serial.println(
    "Commands: status, reset"
  );


  // Ultrasonic setup.
  pinMode(
    TRIG_LEFT,
    OUTPUT
  );

  pinMode(
    ECHO_LEFT,
    INPUT
  );

  pinMode(
    TRIG_RIGHT,
    OUTPUT
  );

  pinMode(
    ECHO_RIGHT,
    INPUT
  );

  Serial.println(
    "Binary Detection Started"
  );

  xTaskCreatePinnedToCore(
    ultrasonicTask,
    "ultrasonic",
    4096,
    NULL,
    1,
    NULL,
    0
  );


  // RTC setup.
  setupRtc();

  out.println();

  out.println(
    "ESP32 stepper controller starting."
  );

  out.println(
    "Bluetooth ready: connect to \"JumpBar-ESP32\"."
  );

  out.println(
    "TEST BUILD: enter 'test' for diagnostic commands."
  );


  restoredFromMemory =
    restoreSavedPosition();

  if (restoredFromMemory)
  {
    out.println(
      "Saved motor position restored."
    );

    out.print(
      "Restored calculated position: "
    );

    out.print(
      stepsToCm(
        motor.currentPosition()
      ),
      2
    );

    out.println(" cm");

    out.print(
      "Restored step position: "
    );

    out.println(
      motor.currentPosition()
    );

    out.println();

    out.println("WARNING:");

    out.println(
      "This position has not been physically verified."
    );

    out.println(
      "Confirm that the mechanism has not been moved"
    );

    out.println(
      "while the ESP32 was switched off."
    );
  }
  else
  {
    motor.setCurrentPosition(0);

    out.println(
      "No valid saved position was found."
    );

    out.println(
      "Current position is being assumed to be 15 cm."
    );

    out.println(
      "Verify the physical position before moving."
    );

    saveCurrentPosition();
  }

  testClockedDogCount =
    dogCount;

  testResetIrCounters();

  testResetUsStatistics();

  out.println();

  out.println(
    "Motor will remain stationary until a command is received."
  );

  printMotorConfig();

  printCommands();
}


// =====================================================================
// MAIN LOOP
// =====================================================================

void loop()
{
  motor.run();

  /*
    Detect when movement has finished.

    The position is saved once after the motor stops.
  */
  if (
    motorWasMoving &&
    motor.distanceToGo() == 0
  )
  {
    motorWasMoving = false;

    out.println();

    out.println(
      "Movement complete."
    );

    out.print(
      "Final calculated position: "
    );

    out.print(
      stepsToCm(
        motor.currentPosition()
      ),
      2
    );

    out.println(" cm");

    if (
      motor.currentPosition() !=
      lastSavedPositionSteps
    )
    {
      saveCurrentPosition();
    }
  }


  /*
    The original IR gate remains paused while the motor is moving.
  */
  if (motor.isRunning())
  {
    syncFlag = false;

    lastFrameTime =
      micros();
  }
  else
  {
    updateReceiver();

    updateGate();

    reportGateEvents();

    reportUltrasonic();
  }


  /*
    DOG_CLOCK and movement completion tracking.

    This does not generate continuous diagnostic output.
  */
  testAlways();


  /*
    Periodic diagnostic output only runs after "test on".
  */
  testMonitor();


  /*
    Service the existing Nordic UART BLE connection.
  */
  serviceBle();


  /*
    Process commands from the Serial Monitor.
  */
  if (Serial.available() > 0)
  {
    String command =
      Serial.readStringUntil('\n');

    processCommand(command);
  }
}
