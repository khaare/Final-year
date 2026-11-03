/*
  ESP32-S3 IR Transmitter - 4 beam scanner

  CONNECTIONS (repeat for each beam n = 1..4):
    GPIO pin -> 1 kOhm resistor -> 2N2222 base
    2N2222 emitter -> GND
    2N2222 collector -> IR LED cathode
    IR LED anode -> series resistor -> supply voltage

    Beam 1: GPIO14
    Beam 2: GPIO13
    Beam 3: GPIO12
    Beam 4: GPIO11

  FRAME (repeats forever, about 9.4 ms):
    SYNC   all enabled LEDs on 2400 us
    off    600 us
    BEAM 1 on 600 us, off 600 us
    BEAM 2 on 600 us, off 600 us
    BEAM 3 on 600 us, off 600 us
    BEAM 4 on 600 us, off 2200 us

  Carrier:
    38 kHz during every "on" period.

  AUTOMATIC START:
    The transmitter starts automatically after power-up.
    The "on" command is not required after startup.

  SIGNAL INTERRUPTION:
    The "interrupt" command turns all emitters off for 150 ms.
    After the interruption, transmission restarts from the SYNC step.

  SERIAL COMMANDS:
    on
    off
    test
    interrupt
    status
    mask 1111

  MASK EXAMPLE:
    mask 1011

    Digits are written beam 1 first:
      digit 1 = beam 1
      digit 2 = beam 2
      digit 3 = beam 3
      digit 4 = beam 4

    Therefore "mask 1011" disables beam 2.
*/


// =====================================================================
// TRANSMITTER CONFIGURATION
// =====================================================================

const uint8_t BEAM_COUNT = 4;

const uint8_t IR_TX_PINS[BEAM_COUNT] =
{
  14,
  13,
  12,
  11
};

const uint32_t IR_FREQUENCY = 38000;

const uint8_t PWM_RESOLUTION = 8;

// Approximately 50% PWM duty cycle.
const uint8_t IR_DUTY = 128;


// =====================================================================
// TEST AND INTERRUPTION SETTINGS
// =====================================================================

const uint32_t TEST_DURATION_MS = 5000;

/*
  The receiver declares signal loss after 100 ms.

  A 150 ms interruption exceeds that period and gives some timing margin.
*/
const uint32_t SIGNAL_INTERRUPT_MS = 150;


// =====================================================================
// FRAME DEFINITION
// =====================================================================

struct Step
{
  // Bit 0 = beam 1, bit 1 = beam 2, etc.
  uint8_t beamBits;

  uint16_t durationUs;
};


const Step FRAME[] =
{
  {0x0F, 2400},   // Sync: all enabled beams
  {0x00, 600},

  {0x01, 600},    // Beam 1
  {0x00, 600},

  {0x02, 600},    // Beam 2
  {0x00, 600},

  {0x04, 600},    // Beam 3
  {0x00, 600},

  {0x08, 600},    // Beam 4
  {0x00, 2200}    // Idle gap
};


const uint8_t FRAME_STEPS =
  sizeof(FRAME) / sizeof(FRAME[0]);


// =====================================================================
// TRANSMITTER STATE
// =====================================================================

bool transmitterEnabled = false;

bool testRunning = false;

/*
  True while the intentional signal interruption is active.
*/
bool signalInterrupted = false;


// All four beams are enabled initially.
uint8_t enabledMask = 0x0F;

// Current unmasked frame bits.
uint8_t currentBits = 0;

// Current position in FRAME[].
uint8_t stepIndex = 0;


// =====================================================================
// TIMING STATE
// =====================================================================

uint32_t previousStepTime = 0;

uint32_t testStartTime = 0;

uint32_t signalInterruptStartTime = 0;


// =====================================================================
// OUTPUT CONTROL
// =====================================================================

void applyOutputs(uint8_t bits)
{
  /*
    Remember the requested frame bits so that changing the mask can be
    applied immediately to the current frame step.
  */
  currentBits = bits;

  uint8_t active =
    bits & enabledMask;

  for (uint8_t i = 0; i < BEAM_COUNT; i++)
  {
    bool beamActive =
      active & (1 << i);

    ledcWrite(
      IR_TX_PINS[i],
      beamActive ? IR_DUTY : 0
    );
  }
}


// =====================================================================
// START AND STOP TRANSMISSION
// =====================================================================

void startTransmission()
{
  transmitterEnabled = true;

  signalInterrupted = false;

  stepIndex = 0;

  previousStepTime =
    micros();

  /*
    Always restart from the complete synchronization step.
  */
  applyOutputs(
    FRAME[0].beamBits
  );

  Serial.println(
    "Transmitter: ON"
  );
}


void stopTransmission()
{
  transmitterEnabled = false;

  testRunning = false;

  signalInterrupted = false;

  applyOutputs(0);

  Serial.println(
    "Transmitter: OFF"
  );
}


// =====================================================================
// NORMAL FRAME SCANNING
// =====================================================================

void updateTransmitter()
{
  /*
    Do not advance the frame while the transmitter is off or while an
    intentional signal interruption is active.
  */
  if (
    !transmitterEnabled ||
    signalInterrupted
  )
  {
    return;
  }

  uint32_t currentTime =
    micros();

  if (
    (uint32_t)(
      currentTime -
      previousStepTime
    ) >= FRAME[stepIndex].durationUs
  )
  {
    /*
      Add the expected step duration instead of setting the time directly
      to currentTime. This reduces accumulated frame timing drift.
    */
    previousStepTime +=
      FRAME[stepIndex].durationUs;

    stepIndex =
      (stepIndex + 1) %
      FRAME_STEPS;

    applyOutputs(
      FRAME[stepIndex].beamBits
    );
  }
}


// =====================================================================
// CONTROLLED SIGNAL INTERRUPTION
// =====================================================================

void interruptSignal()
{
  if (!transmitterEnabled)
  {
    Serial.println(
      "ERROR: Transmitter is not running."
    );

    return;
  }

  if (signalInterrupted)
  {
    Serial.println(
      "Signal interruption is already active."
    );

    return;
  }

  signalInterrupted = true;

  signalInterruptStartTime =
    millis();

  /*
    Immediately turn every emitter off.
  */
  applyOutputs(0);

  Serial.print(
    "Transmitter signal interrupted for "
  );

  Serial.print(
    SIGNAL_INTERRUPT_MS
  );

  Serial.println(" ms.");
}


void updateSignalInterrupt()
{
  if (!signalInterrupted)
  {
    return;
  }

  if (
    (uint32_t)(
      millis() -
      signalInterruptStartTime
    ) >= SIGNAL_INTERRUPT_MS
  )
  {
    signalInterrupted = false;

    /*
      Restart from the synchronization step rather than continuing from
      the frame position at which the signal was interrupted.
    */
    stepIndex = 0;

    previousStepTime =
      micros();

    applyOutputs(
      FRAME[0].beamBits
    );

    Serial.println(
      "Transmitter signal restored."
    );

    Serial.println(
      "Frame restarted from the SYNC step."
    );
  }
}


// =====================================================================
// FIVE-SECOND TEST
// =====================================================================

void startTest()
{
  testRunning = true;

  testStartTime =
    millis();

  startTransmission();

  /*
    startTransmission() does not normally change testRunning. Assigning
    it here again makes the intended test state explicit.
  */
  testRunning = true;

  Serial.println(
    "Five-second IR test started."
  );
}


void updateTest()
{
  if (
    testRunning &&
    (uint32_t)(
      millis() -
      testStartTime
    ) >= TEST_DURATION_MS
  )
  {
    stopTransmission();

    Serial.println(
      "Test complete."
    );
  }
}


// =====================================================================
// BEAM MASK
// =====================================================================

void setMask(
  const String &bitsText
)
{
  if (
    bitsText.length() !=
    BEAM_COUNT
  )
  {
    Serial.println(
      "Usage: mask 1111  (4 digits, beam 1 first)"
    );

    return;
  }

  uint8_t newMask = 0;

  for (uint8_t i = 0; i < BEAM_COUNT; i++)
  {
    char c =
      bitsText[i];

    if (
      c != '0' &&
      c != '1'
    )
    {
      Serial.println(
        "Usage: mask 1111  (4 digits, beam 1 first)"
      );

      return;
    }

    if (c == '1')
    {
      newMask |=
        (1 << i);
    }
  }

  enabledMask = newMask;

  /*
    Do not reactivate an output during a signal interruption.
  */
  if (signalInterrupted)
  {
    applyOutputs(0);
  }
  else if (transmitterEnabled)
  {
    applyOutputs(currentBits);
  }
  else
  {
    applyOutputs(0);
  }

  Serial.print(
    "Beam mask set: "
  );

  Serial.println(
    bitsText
  );
}


// =====================================================================
// STATUS
// =====================================================================

void printStatus()
{
  Serial.println();

  Serial.println(
    "----- TX STATUS -----"
  );

  Serial.print(
    "Transmitter: "
  );

  Serial.println(
    transmitterEnabled
      ? "ON"
      : "OFF"
  );

  Serial.print(
    "Signal interrupted: "
  );

  Serial.println(
    signalInterrupted
      ? "YES"
      : "NO"
  );

  Serial.print(
    "Five-second test: "
  );

  Serial.println(
    testRunning
      ? "RUNNING"
      : "NOT RUNNING"
  );

  Serial.print(
    "Current frame step: "
  );

  Serial.print(stepIndex);

  Serial.print(" of ");

  Serial.println(
    FRAME_STEPS - 1
  );

  for (uint8_t i = 0; i < BEAM_COUNT; i++)
  {
    bool enabled =
      enabledMask & (1 << i);

    Serial.print("Beam ");

    Serial.print(i + 1);

    Serial.print(" (GPIO");

    Serial.print(
      IR_TX_PINS[i]
    );

    Serial.print("): ");

    Serial.println(
      enabled
        ? "enabled"
        : "disabled"
    );
  }

  Serial.println(
    "---------------------"
  );
}


// =====================================================================
// COMMAND LIST
// =====================================================================

void printCommands()
{
  Serial.println();

  Serial.println(
    "Available commands:"
  );

  Serial.println(
    "  on        : start or restart transmission"
  );

  Serial.println(
    "  off       : stop transmission"
  );

  Serial.println(
    "  test      : transmit for five seconds, then stop"
  );

  Serial.println(
    "  interrupt : switch all emitters off for 150 ms"
  );

  Serial.println(
    "  status    : display transmitter status"
  );

  Serial.println(
    "  mask 1111 : enable or disable individual beams"
  );

  Serial.println(
    "  help      : display this command list"
  );

  Serial.println();
}


// =====================================================================
// SERIAL COMMAND PROCESSING
// =====================================================================

void readSerialCommand()
{
  if (!Serial.available())
  {
    return;
  }

  String command =
    Serial.readStringUntil('\n');

  command.trim();

  command.toLowerCase();

  if (command == "on")
  {
    testRunning = false;

    startTransmission();
  }
  else if (command == "off")
  {
    stopTransmission();
  }
  else if (command == "test")
  {
    startTest();
  }
  else if (command == "interrupt")
  {
    interruptSignal();
  }
  else if (command == "status")
  {
    printStatus();
  }
  else if (command == "help")
  {
    printCommands();
  }
  else if (
    command.startsWith("mask ")
  )
  {
    String bitsText =
      command.substring(5);

    bitsText.trim();

    setMask(bitsText);
  }
  else if (command.length() > 0)
  {
    Serial.print(
      "Unknown command: "
    );

    Serial.println(command);

    Serial.println(
      "Commands: on, off, test, interrupt, status, mask 1111, help"
    );
  }
}


// =====================================================================
// SETUP
// =====================================================================

void setup()
{
  Serial.begin(115200);

  Serial.setTimeout(50);

  delay(1000);

  for (uint8_t i = 0; i < BEAM_COUNT; i++)
  {
    if (
      !ledcAttach(
        IR_TX_PINS[i],
        IR_FREQUENCY,
        PWM_RESOLUTION
      )
    )
    {
      Serial.print(
        "ERROR: Failed to configure 38 kHz PWM on GPIO"
      );

      Serial.println(
        IR_TX_PINS[i]
      );

      while (true)
      {
        delay(1000);
      }
    }
  }

  /*
    Ensure all emitters are off before starting the scanner.
  */
  applyOutputs(0);

  Serial.println();

  Serial.println(
    "ESP32-S3 4-beam IR transmitter ready."
  );

  printCommands();

  /*
    Start the original four-beam scanning process automatically.

    No manual "on" command is needed after power-up or reset.
  */
  startTransmission();
}


// =====================================================================
// MAIN LOOP
// =====================================================================

void loop()
{
  readSerialCommand();

  /*
    Restore transmission when the 150 ms interruption has finished.
  */
  updateSignalInterrupt();

  /*
    Run the normal scanning frame when transmission is active and not
    interrupted.
  */
  updateTransmitter();

  /*
    Stop transmission after five seconds when the test command is used.
  */
  updateTest();
}