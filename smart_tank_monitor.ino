/*
  ============================================================
  ESP32 SMART OVERHEAD TANK WATER QUALITY & SUPPLY MONITOR
  ============================================================

  Features:
  1. HC-SR04 overhead tank level monitoring
  2. Tank level converted to percentage and litres
  3. Pump ON at <= 30%
  4. Pump OFF at >= 90%
  5. Sump protection at <= 20%
  6. Water quality analog input
  7. 5-sample moving average
  8. Normal quality range: 400 - 600
  9. 5 consecutive abnormal readings required
  10. Single spike rejection
  11. Tanker fill detection
  12. Invalid tank level detection
  13. Relay control
  14. Quality LED
  15. Buzzer
  16. Safe startup after power restoration

  ============================================================
*/

// ============================================================
// PIN CONFIGURATION
// ============================================================

#define TRIG_PIN       5
#define ECHO_PIN       18

#define QUALITY_PIN    34
#define SUMP_PIN       35

#define PUMP_RELAY     26
#define QUALITY_LED    27
#define BUZZER_PIN     25


// ============================================================
// TANK CONFIGURATION
// ============================================================

const float TANK_CAPACITY_LITRES = 1000.0;

const float TANK_HEIGHT_CM = 100.0;

// Sensor mounting distance from sensor to tank bottom
const float SENSOR_EMPTY_DISTANCE_CM = 100.0;


// ============================================================
// PUMP CONTROL THRESHOLDS
// ============================================================

const float PUMP_START_LEVEL = 30.0;
const float PUMP_STOP_LEVEL  = 90.0;

const float SUMP_EMPTY_LEVEL = 20.0;


// ============================================================
// WATER QUALITY CONFIGURATION
// ============================================================

const int QUALITY_MIN = 400;
const int QUALITY_MAX = 600;

const int MOVING_AVERAGE_SIZE = 5;

const int REQUIRED_ABNORMAL_SAMPLES = 5;


// ============================================================
// TIMING
// ============================================================

const unsigned long SAMPLE_INTERVAL = 1000;

unsigned long lastSampleTime = 0;


// ============================================================
// MOVING AVERAGE VARIABLES
// ============================================================

int qualitySamples[MOVING_AVERAGE_SIZE];

int qualityIndex = 0;

bool qualityBufferFull = false;


// ============================================================
// SYSTEM VARIABLES
// ============================================================

bool pumpState = false;

bool qualityAlarm = false;

int abnormalCount = 0;

float previousTankLevel = 0;

bool firstTankReading = true;

unsigned long qualityShiftStartTime = 0;


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  // Sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(QUALITY_PIN, INPUT);
  pinMode(SUMP_PIN, INPUT);

  // Output pins
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(QUALITY_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Safe initial state
  digitalWrite(PUMP_RELAY, LOW);
  digitalWrite(QUALITY_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize quality buffer
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    qualitySamples[i] = 500;
  }

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" ESP32 SMART WATER MONITOR");
  Serial.println(" SYSTEM STARTING...");
  Serial.println("==========================================");

  delay(2000);

  Serial.println("System initialized.");
  Serial.println("Performing first sensor measurement...");

  // Initial sensor reading
  float initialTankLevel = readTankLevel();

  if (initialTankLevel >= 0) {

    previousTankLevel = initialTankLevel;

    firstTankReading = false;

    Serial.println("Initial tank level accepted.");

  } else {

    Serial.println("WARNING: Initial tank sensor fault.");

  }

  Serial.println("System ready.");
  Serial.println();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  unsigned long currentTime = millis();

  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {

    lastSampleTime = currentTime;

    // --------------------------------------------------------
    // 1. READ TANK LEVEL
    // --------------------------------------------------------

    float tankLevel = readTankLevel();


    // --------------------------------------------------------
    // 2. READ SUMP LEVEL
    // --------------------------------------------------------

    float sumpLevel = readSumpLevel();


    // --------------------------------------------------------
    // 3. READ WATER QUALITY
    // --------------------------------------------------------

    int rawQuality = analogRead(QUALITY_PIN);

    int qualityValue = map(rawQuality, 0, 4095, 0, 1000);

    int smoothedQuality = calculateMovingAverage(qualityValue);


    // --------------------------------------------------------
    // 4. PRINT SENSOR DATA
    // --------------------------------------------------------

    Serial.println();
    Serial.println("------------------------------------------");

    Serial.print("Tank Level: ");

    if (tankLevel >= 0) {

      Serial.print(tankLevel, 1);

      Serial.print("%");

      float litres = calculateLitres(tankLevel);

      Serial.print(" | Volume: ");

      Serial.print(litres, 1);

      Serial.println(" L");

    } else {

      Serial.println("SENSOR FAULT");

    }


    Serial.print("Sump Level: ");

    Serial.print(sumpLevel, 1);

    Serial.println("%");


    Serial.print("Quality Raw: ");

    Serial.print(qualityValue);

    Serial.print(" | Average: ");

    Serial.println(smoothedQuality);


    // --------------------------------------------------------
    // 5. CHECK TANK SENSOR
    // --------------------------------------------------------

    if (tankLevel < 0 || tankLevel > 100) {

      Serial.println("WARNING: INVALID TANK LEVEL");

      stopPump();

    }

    else {

      // ------------------------------------------------------
      // 6. DETECT TANKER FILL
      // ------------------------------------------------------

      detectTankerFill(tankLevel);


      // ------------------------------------------------------
      // 7. CONTROL PUMP
      // ------------------------------------------------------

      controlPump(tankLevel, sumpLevel);


      // Update previous level
      previousTankLevel = tankLevel;

      firstTankReading = false;

    }


    // --------------------------------------------------------
    // 8. WATER QUALITY MONITORING
    // --------------------------------------------------------

    monitorWaterQuality(smoothedQuality);


    // --------------------------------------------------------
    // 9. PRINT FINAL STATUS
    // --------------------------------------------------------

    Serial.println("------------------------------------------");

    Serial.print("Pump Status: ");

    if (pumpState) {

      Serial.println("ON");

    } else {

      Serial.println("OFF");

    }


    Serial.print("Quality Status: ");

    if (qualityAlarm) {

      Serial.println("ABNORMAL - ALERT");

    } else {

      Serial.println("NORMAL");

    }

    Serial.println("------------------------------------------");
  }
}


// ============================================================
// FUNCTION: READ TANK LEVEL
// ============================================================

float readTankLevel() {

  // Send ultrasonic trigger pulse

  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);


  // Read echo duration

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);


  // No echo received

  if (duration == 0) {

    return -1;

  }


  // Calculate distance in cm

  float distance = duration * 0.0343 / 2.0;


  // Calculate water height

  float waterHeight =
      SENSOR_EMPTY_DISTANCE_CM - distance;


  // Convert to percentage

  float levelPercentage =
      (waterHeight / TANK_HEIGHT_CM) * 100.0;


  // Check physical range

  if (levelPercentage < 0 ||
      levelPercentage > 100) {

    return -1;

  }


  return levelPercentage;
}


// ============================================================
// FUNCTION: CALCULATE LITRES
// ============================================================

float calculateLitres(float levelPercentage) {

  float litres =
      (levelPercentage / 100.0)
      * TANK_CAPACITY_LITRES;

  return litres;
}


// ============================================================
// FUNCTION: READ SUMP LEVEL
// ============================================================

float readSumpLevel() {

  int rawValue =
      analogRead(SUMP_PIN);


  float sumpPercentage =
      map(rawValue, 0, 4095, 0, 100);


  return sumpPercentage;
}


// ============================================================
// FUNCTION: CONTROL PUMP
// ============================================================

void controlPump(float tankLevel,
                 float sumpLevel) {


  // ----------------------------------------------------------
  // SUMP EMPTY PROTECTION
  // ----------------------------------------------------------

  if (sumpLevel <= SUMP_EMPTY_LEVEL) {

    stopPump();

    Serial.println(
      "SUMP LOW -> DRY RUN PROTECTION ACTIVE"
    );

    return;
  }


  // ----------------------------------------------------------
  // PUMP START CONDITION
  // ----------------------------------------------------------

  if (tankLevel <= PUMP_START_LEVEL) {

    startPump();

    Serial.println(
      "TANK LOW -> PUMP STARTED"
    );

  }


  // ----------------------------------------------------------
  // PUMP STOP CONDITION
  // ----------------------------------------------------------

  else if (tankLevel >= PUMP_STOP_LEVEL) {

    stopPump();

    Serial.println(
      "TANK FULL -> PUMP STOPPED"
    );

  }


  // ----------------------------------------------------------
  // HYSTERESIS REGION
  // ----------------------------------------------------------

  else {

    // Keep previous pump state

    if (pumpState) {

      Serial.println(
        "TANK FILLING -> PUMP CONTINUES"
      );

    } else {

      Serial.println(
        "TANK LEVEL NORMAL -> PUMP OFF"
      );

    }
  }
}


// ============================================================
// FUNCTION: START PUMP
// ============================================================

void startPump() {

  if (!pumpState) {

    digitalWrite(PUMP_RELAY, HIGH);

    pumpState = true;

    Serial.println("RELAY -> ON");

  }
}


// ============================================================
// FUNCTION: STOP PUMP
// ============================================================

void stopPump() {

  if (pumpState) {

    digitalWrite(PUMP_RELAY, LOW);

    pumpState = false;

    Serial.println("RELAY -> OFF");

  }
}


// ============================================================
// FUNCTION: MOVING AVERAGE
// ============================================================

int calculateMovingAverage(int newValue) {

  qualitySamples[qualityIndex] =
      newValue;


  qualityIndex++;

  if (qualityIndex >= MOVING_AVERAGE_SIZE) {

    qualityIndex = 0;

    qualityBufferFull = true;

  }


  int sampleCount;

  if (qualityBufferFull) {

    sampleCount =
        MOVING_AVERAGE_SIZE;

  } else {

    sampleCount =
        qualityIndex;

  }


  long total = 0;


  for (int i = 0;
       i < sampleCount;
       i++) {

    total += qualitySamples[i];

  }


  if (sampleCount == 0) {

    return newValue;

  }


  return total / sampleCount;
}


// ============================================================
// FUNCTION: WATER QUALITY MONITOR
// ============================================================

void monitorWaterQuality(
    int smoothedQuality) {


  bool abnormal = false;


  // Check normal range

  if (smoothedQuality < QUALITY_MIN ||
      smoothedQuality > QUALITY_MAX) {

    abnormal = true;

  }


  // ----------------------------------------------------------
  // ABNORMAL READING
  // ----------------------------------------------------------

  if (abnormal) {

    abnormalCount++;


    Serial.print(
      "ABNORMAL QUALITY SAMPLE: "
    );

    Serial.print(abnormalCount);

    Serial.print("/");

    Serial.println(
      REQUIRED_ABNORMAL_SAMPLES
    );


    // Record shift start

    if (abnormalCount == 1) {

      qualityShiftStartTime =
          millis();

    }


    // --------------------------------------------------------
    // GENUINE QUALITY SHIFT
    // --------------------------------------------------------

    if (abnormalCount >=
        REQUIRED_ABNORMAL_SAMPLES) {


      if (!qualityAlarm) {

        qualityAlarm = true;


        Serial.println();
        Serial.println(
          "!!! QUALITY SHIFT DETECTED !!!"
        );

        Serial.print(
          "New Quality Value: "
        );

        Serial.println(
          smoothedQuality
        );

        Serial.println(
          "Persistence: 5 consecutive samples"
        );

        Serial.println(
          "BUZZER -> ON"
        );

        Serial.println(
          "QUALITY LED -> ON"
        );

      }


      digitalWrite(
        QUALITY_LED,
        HIGH
      );

      digitalWrite(
        BUZZER_PIN,
        HIGH
      );

    }

  }


  // ----------------------------------------------------------
  // NORMAL QUALITY
  // ----------------------------------------------------------

  else {

    if (qualityAlarm) {

      Serial.println(
        "QUALITY RETURNED TO NORMAL"
      );

    }


    abnormalCount = 0;

    qualityAlarm = false;


    digitalWrite(
      QUALITY_LED,
      LOW
    );

    digitalWrite(
      BUZZER_PIN,
      LOW
    );

  }
}


// ============================================================
// FUNCTION: TANKER FILL DETECTION
// ============================================================

void detectTankerFill(
    float currentLevel) {


  if (firstTankReading) {

    return;

  }


  // Tank rising while pump is OFF

  if (currentLevel >
      previousTankLevel + 1.0) {


    if (!pumpState) {

      Serial.println();

      Serial.println(
        "TANK LEVEL RISING"
      );

      Serial.println(
        "PUMP IS OFF"
      );

      Serial.println(
        "POSSIBLE TANKER FILL DETECTED"
      );

    }

  }
}
