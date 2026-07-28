/*
  ============================================================
  OVERHEAD TANK WATER QUALITY & SUPPLY MONITOR
  SIH 2026 - LEVEL 2 MODIFIED VERSION
  ============================================================

  CONTROLLER:
  ESP32 DevKit V1

  HARDWARE / SIMULATION:
  HC-SR04       -> Tank Level
  Potentiometer -> Water Quality / TDS Simulation
  Potentiometer -> Sump Level Simulation
  Relay         -> Pump Control
  LED           -> Water Quality Alert
  Buzzer        -> Water Quality Alert

  ============================================================
  ESP32 PIN MAPPING
  ============================================================

  HC-SR04 TRIG        -> GPIO 5
  HC-SR04 ECHO        -> GPIO 18

  Quality Sensor      -> GPIO 34
  Sump Sensor         -> GPIO 35

  Pump Relay          -> GPIO 26
  Quality LED         -> GPIO 27
  Buzzer              -> GPIO 25

  ============================================================
  SIH LEVEL 2 CHANGE 1
  ============================================================

  QUALITY DEVIATION FROM NORMAL

  Normal Quality Range = 400 to 600 ADC
  Normal Centre        = 500 ADC

  Quality Deviation =
      Smoothed Quality - Normal Centre

  Example:
      Smoothed Quality = 520
      Deviation = +20 ADC

  ============================================================
  SIH LEVEL 2 CHANGE 2
  ============================================================

  IMPOSSIBLE SENSOR READING -> SENSOR FAULT

  Tank geometry:
      Empty tank distance = 125 cm
      Full tank distance  = 25 cm

  Valid sensor range:
      25 cm to 125 cm

  If distance is outside this range:
      - Sensor Fault
      - Tank Level = INVALID
      - Volume = INVALID
      - Pump = OFF
      - No false "empty tank" detection
      - No false quality alarm

  ============================================================
*/

// ============================================================
// 1. PIN DEFINITIONS
// ============================================================

const int TRIG_PIN = 5;
const int ECHO_PIN = 18;

const int QUALITY_PIN = 34;
const int SUMP_PIN = 35;

const int PUMP_RELAY_PIN = 26;
const int QUALITY_LED_PIN = 27;
const int BUZZER_PIN = 25;


// ============================================================
// 2. TANK CONFIGURATION
// ============================================================

// Tank capacity
const float TANK_CAPACITY_LITRES = 1000.0;

// Distance from ultrasonic sensor to tank bottom
// when tank is empty
const float EMPTY_DISTANCE_CM = 125.0;

// Distance from ultrasonic sensor to water surface
// when tank is considered full
const float FULL_DISTANCE_CM = 25.0;

// Effective water height
const float EFFECTIVE_TANK_HEIGHT_CM =
    EMPTY_DISTANCE_CM - FULL_DISTANCE_CM;


// ============================================================
// 3. PUMP CONTROL
// ============================================================

// Pump starts at or below 30%
const float PUMP_START_LEVEL = 30.0;

// Pump stops at or above 90%
const float PUMP_STOP_LEVEL = 90.0;


// ============================================================
// 4. SUMP PROTECTION
// ============================================================

// If sump is 20% or below,
// pump must be OFF
const float SUMP_EMPTY_THRESHOLD = 20.0;


// ============================================================
// 5. WATER QUALITY CONFIGURATION
// ============================================================

// Simulated normal quality range
const int QUALITY_MIN = 400;
const int QUALITY_MAX = 600;

// Centre of normal range
const int QUALITY_NORMAL_CENTER =
    (QUALITY_MIN + QUALITY_MAX) / 2;


// ============================================================
// 6. QUALITY MOVING AVERAGE
// ============================================================

const int MOVING_AVERAGE_SIZE = 5;

int qualitySamples[MOVING_AVERAGE_SIZE];

int qualitySampleIndex = 0;

long qualitySampleTotal = 0;

int numberOfQualitySamples = 0;

int smoothedQuality = 0;


// ============================================================
// 7. QUALITY PERSISTENCE
// ============================================================

// Quality must remain abnormal for
// 5 consecutive samples
const int REQUIRED_ABNORMAL_SAMPLES = 5;

int abnormalSampleCount = 0;

bool qualityAlertActive = false;


// ============================================================
// 8. PUMP STATE
// ============================================================

bool pumpState = false;


// ============================================================
// 9. SENSOR FAULT STATE
// ============================================================

bool levelSensorFault = false;


// ============================================================
// 10. TANKER DETECTION
// ============================================================

float previousTankLevel = -1.0;

const float TANKER_RISE_THRESHOLD = 1.0;


// ============================================================
// 11. SAMPLING INTERVAL
// ============================================================

const unsigned long SAMPLE_INTERVAL = 1000;

unsigned long lastSampleTime = 0;


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  // -------------------------------
  // Ultrasonic Sensor
  // -------------------------------

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  // -------------------------------
  // Outputs
  // -------------------------------

  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(QUALITY_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);


  // -------------------------------
  // Safe Startup State
  // -------------------------------

  digitalWrite(PUMP_RELAY_PIN, LOW);

  digitalWrite(QUALITY_LED_PIN, LOW);

  digitalWrite(BUZZER_PIN, LOW);


  // -------------------------------
  // Initialize Quality Buffer
  // -------------------------------

  for (int i = 0;
       i < MOVING_AVERAGE_SIZE;
       i++) {

    qualitySamples[i] = 0;
  }


  Serial.println();
  Serial.println("================================================");
  Serial.println(" OVERHEAD TANK WATER QUALITY & SUPPLY MONITOR ");
  Serial.println("================================================");

  Serial.println("ESP32 SYSTEM STARTED");

  Serial.println();
  Serial.println("SIH LEVEL 2 MODIFICATIONS:");
  Serial.println("CHANGE 1: QUALITY DEVIATION FROM NORMAL");
  Serial.println("CHANGE 2: IMPOSSIBLE SENSOR READING DETECTION");

  Serial.println();

  Serial.print("Tank Capacity: ");
  Serial.print(TANK_CAPACITY_LITRES);
  Serial.println(" Litres");

  Serial.print("Pump Start Level: ");
  Serial.print(PUMP_START_LEVEL);
  Serial.println("%");

  Serial.print("Pump Stop Level: ");
  Serial.print(PUMP_STOP_LEVEL);
  Serial.println("%");

  Serial.print("Sump Empty Threshold: ");
  Serial.print(SUMP_EMPTY_THRESHOLD);
  Serial.println("%");

  Serial.print("Quality Normal Range: ");
  Serial.print(QUALITY_MIN);
  Serial.print(" - ");
  Serial.print(QUALITY_MAX);
  Serial.println(" ADC");

  Serial.print("Quality Normal Centre: ");
  Serial.print(QUALITY_NORMAL_CENTER);
  Serial.println(" ADC");

  Serial.print("Valid Ultrasonic Range: ");
  Serial.print(FULL_DISTANCE_CM);
  Serial.print(" - ");
  Serial.print(EMPTY_DISTANCE_CM);
  Serial.println(" cm");

  Serial.println();
  Serial.println("System is ready.");
  Serial.println("================================================");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  unsigned long currentTime = millis();


  // Run monitoring every 1 second
  if (currentTime - lastSampleTime <
      SAMPLE_INTERVAL) {

    return;
  }

  lastSampleTime = currentTime;


  Serial.println();
  Serial.println("------------------------------------------------");


  // ==========================================================
  // STEP 1: READ TANK LEVEL
  // ==========================================================

  float tankLevel = readTankLevel();


  // ==========================================================
  // CHANGE 2:
  // HANDLE BROKEN / IMPOSSIBLE SENSOR READING
  // ==========================================================

  if (tankLevel < 0) {

    handleLevelSensorFault();

    // IMPORTANT:
    // Do not calculate litres
    // Do not control pump using invalid level
    // Do not detect tanker fill

    // We still read quality independently.
    // This ensures a level sensor fault does
    // NOT create a false quality alarm.

    readAndMonitorQuality();

    Serial.println("------------------------------------------------");

    return;
  }


  // Sensor is working correctly

  levelSensorFault = false;


  // ==========================================================
  // STEP 2: CALCULATE TANK VOLUME
  // ==========================================================

  float tankLitres =
      calculateLitres(tankLevel);


  // ==========================================================
  // STEP 3: READ SUMP LEVEL
  // ==========================================================

  float sumpLevel =
      readSumpLevel();


  // ==========================================================
  // STEP 4: DISPLAY TANK INFORMATION
  // ==========================================================

  Serial.println("TANK INFORMATION");

  Serial.print("Tank Level: ");
  Serial.print(tankLevel, 1);
  Serial.println("%");

  Serial.print("Tank Volume: ");
  Serial.print(tankLitres, 1);
  Serial.println(" L");


  // ==========================================================
  // STEP 5: DISPLAY SUMP INFORMATION
  // ==========================================================

  Serial.println();

  Serial.print("Sump Level: ");
  Serial.print(sumpLevel, 1);
  Serial.println("%");


  // ==========================================================
  // STEP 6: CONTROL PUMP
  // ==========================================================

  controlPump(
    tankLevel,
    sumpLevel
  );


  // ==========================================================
  // STEP 7: QUALITY MONITORING
  // ==========================================================

  readAndMonitorQuality();


  // ==========================================================
  // STEP 8: TANKER FILL DETECTION
  // ==========================================================

  detectTankerFill(tankLevel);


  // Store valid tank level
  previousTankLevel = tankLevel;


  Serial.println("------------------------------------------------");
}


// ============================================================
// FUNCTION: READ TANK LEVEL
// ============================================================

float readTankLevel() {

  // Send trigger pulse

  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);


  // Read echo
  // Timeout = 30 ms

  unsigned long duration =
      pulseIn(
        ECHO_PIN,
        HIGH,
        30000
      );


  // ==========================================================
  // CASE 1: NO ECHO
  // ==========================================================

  if (duration == 0) {

    Serial.println();
    Serial.println("!!! ULTRASONIC SENSOR FAULT !!!");

    Serial.println("Reason: No Echo Received");

    return -1;
  }


  // Calculate distance

  float distance =
      (duration * 0.0343) / 2.0;


  Serial.print("Ultrasonic Distance: ");
  Serial.print(distance, 2);
  Serial.println(" cm");


  // ==========================================================
  // CASE 2:
  // IMPOSSIBLE / OUT-OF-RANGE READING
  // ==========================================================

  if (distance < FULL_DISTANCE_CM ||
      distance > EMPTY_DISTANCE_CM) {

    Serial.println();
    Serial.println(
      "!!! IMPOSSIBLE SENSOR READING !!!"
    );

    Serial.print("Measured Distance: ");
    Serial.print(distance, 2);
    Serial.println(" cm");

    Serial.print("Valid Range: ");
    Serial.print(FULL_DISTANCE_CM);
    Serial.print(" - ");
    Serial.print(EMPTY_DISTANCE_CM);
    Serial.println(" cm");

    Serial.println(
      "Reading rejected as SENSOR FAULT"
    );

    return -1;
  }


  // ==========================================================
  // VALID SENSOR READING
  // ==========================================================

  float waterHeight =
      EMPTY_DISTANCE_CM - distance;


  // Convert water height to percentage

  float levelPercentage =
      (waterHeight /
       EFFECTIVE_TANK_HEIGHT_CM)
      * 100.0;


  // Safety validation

  if (levelPercentage < 0.0 ||
      levelPercentage > 100.0) {

    Serial.println(
      "ERROR: CALCULATED LEVEL INVALID"
    );

    return -1;
  }


  return levelPercentage;
}


// ============================================================
// FUNCTION: HANDLE LEVEL SENSOR FAULT
// ============================================================

void handleLevelSensorFault() {

  levelSensorFault = true;


  Serial.println();
  Serial.println("==============================================");
  Serial.println("          LEVEL SENSOR FAULT");
  Serial.println("==============================================");

  Serial.println(
    "Tank Level : INVALID"
  );

  Serial.println(
    "Tank Volume: INVALID"
  );

  Serial.println();

  Serial.println(
    "The sensor reading is NOT treated as a"
  );

  Serial.println(
    "genuine empty/full tank measurement."
  );

  Serial.println();

  Serial.println(
    "Safety Action: PUMP FORCED OFF"
  );


  // Force pump OFF

  stopPump();


  Serial.println(
    "Pump Status: OFF"
  );

  Serial.println();

  Serial.println(
    "Quality monitoring continues independently."
  );

  Serial.println(
    "Sensor fault will NOT create a false"
  );

  Serial.println(
    "water-quality alarm."
  );

  Serial.println("==============================================");
}


// ============================================================
// FUNCTION: CALCULATE LITRES
// ============================================================

float calculateLitres(
  float levelPercentage
) {

  return (
    levelPercentage /
    100.0
  ) * TANK_CAPACITY_LITRES;
}


// ============================================================
// FUNCTION: READ SUMP LEVEL
// ============================================================

float readSumpLevel() {

  int rawValue =
      analogRead(SUMP_PIN);


  // ESP32 ADC:
  // 0 - 4095

  float sumpPercentage =
      ((float)rawValue / 4095.0)
      * 100.0;


  // Limit to 0-100%

  sumpPercentage =
      constrain(
        sumpPercentage,
        0.0,
        100.0
      );


  return sumpPercentage;
}


// ============================================================
// FUNCTION: CONTROL PUMP
// ============================================================

void controlPump(
  float tankLevel,
  float sumpLevel
) {


  // ==========================================================
  // SAFETY PRIORITY 1:
  // LEVEL SENSOR FAULT
  // ==========================================================

  if (levelSensorFault) {

    stopPump();

    Serial.println(
      "PUMP: OFF - LEVEL SENSOR FAULT"
    );

    return;
  }


  // ==========================================================
  // SAFETY PRIORITY 2:
  // SUMP EMPTY
  // ==========================================================

  if (sumpLevel <=
      SUMP_EMPTY_THRESHOLD) {

    stopPump();

    Serial.println(
      "SUMP LEVEL <= 20%"
    );

    Serial.println(
      "DRY-RUN PROTECTION ACTIVE"
    );

    Serial.println(
      "PUMP: OFF"
    );

    return;
  }


  // ==========================================================
  // PUMP START
  // ==========================================================

  if (tankLevel <=
      PUMP_START_LEVEL) {

    startPump();

    Serial.println(
      "Tank Level <= 30%"
    );

    Serial.println(
      "PUMP: ON"
    );
  }


  // ==========================================================
  // PUMP STOP
  // ==========================================================

  else if (tankLevel >=
           PUMP_STOP_LEVEL) {

    stopPump();

    Serial.println(
      "Tank Level >= 90%"
    );

    Serial.println(
      "PUMP: OFF"
    );
  }


  // ==========================================================
  // HYSTERESIS REGION
  // ==========================================================

  else {

    Serial.print(
      "Tank Level: "
    );

    Serial.print(
      tankLevel,
      1
    );

    Serial.println(
      "% - Hysteresis Region"
    );


    if (pumpState) {

      Serial.println(
        "PUMP: ON - Maintaining State"
      );

    } else {

      Serial.println(
        "PUMP: OFF - Maintaining State"
      );
    }
  }
}


// ============================================================
// FUNCTION: START PUMP
// ============================================================

void startPump() {

  if (!pumpState) {

    digitalWrite(
      PUMP_RELAY_PIN,
      HIGH
    );

    pumpState = true;

    Serial.println(
      "Pump Relay: ON"
    );
  }
}


// ============================================================
// FUNCTION: STOP PUMP
// ============================================================

void stopPump() {

  digitalWrite(
    PUMP_RELAY_PIN,
    LOW
  );

  pumpState = false;
}


// ============================================================
// FUNCTION: READ AND MONITOR QUALITY
// ============================================================

void readAndMonitorQuality() {

  // Read raw quality sensor

  int rawQuality =
      analogRead(
        QUALITY_PIN
      );


  // Update moving average

  smoothedQuality =
      updateQualityAverage(
        rawQuality
      );


  // ==========================================================
  // CHANGE 1:
  // QUALITY DEVIATION FROM NORMAL
  // ==========================================================

  int qualityDeviation =
      smoothedQuality -
      QUALITY_NORMAL_CENTER;


  int absoluteDeviation =
      abs(
        qualityDeviation
      );


  // ==========================================================
  // DISPLAY QUALITY DATA
  // ==========================================================

  Serial.println();

  Serial.println(
    "WATER QUALITY MONITORING"
  );

  Serial.print(
    "Raw Quality ADC: "
  );

  Serial.println(
    rawQuality
  );


  Serial.print(
    "Smoothed Quality: "
  );

  Serial.println(
    smoothedQuality
  );


  Serial.print(
    "Normal Range: "
  );

  Serial.print(
    QUALITY_MIN
  );

  Serial.print(
    " - "
  );

  Serial.println(
    QUALITY_MAX
  );


  Serial.print(
    "Normal Centre: "
  );

  Serial.println(
    QUALITY_NORMAL_CENTER
  );


  // ==========================================================
  // NEW VALUE FOR SIH CHANGE 1
  // ==========================================================

  Serial.print(
    "Quality Deviation from Normal: "
  );


  if (qualityDeviation >= 0) {

    Serial.print("+");
  }


  Serial.print(
    qualityDeviation
  );

  Serial.println(
    " ADC"
  );


  Serial.print(
    "Absolute Quality Deviation: "
  );

  Serial.print(
    absoluteDeviation
  );

  Serial.println(
    " ADC"
  );


  // ==========================================================
  // QUALITY STATUS
  // ==========================================================

  if (smoothedQuality >= QUALITY_MIN &&
      smoothedQuality <= QUALITY_MAX) {

    Serial.println(
      "Quality Status: NORMAL"
    );

  } else {

    Serial.println(
      "Quality Status: OUTSIDE NORMAL RANGE"
    );
  }


  // Run persistence detection

  monitorQualityPersistence(
    smoothedQuality
  );
}


// ============================================================
// FUNCTION: UPDATE QUALITY MOVING AVERAGE
// ============================================================

int updateQualityAverage(
  int newValue
) {

  // Add new sample

  qualitySampleTotal +=
      newValue;


  qualitySamples[
    qualitySampleIndex
  ] = newValue;


  qualitySampleIndex++;


  // Keep index inside buffer

  if (qualitySampleIndex >=
      MOVING_AVERAGE_SIZE) {

    qualitySampleIndex = 0;
  }


  // Remove oldest sample only
  // after buffer becomes full

  if (numberOfQualitySamples <
      MOVING_AVERAGE_SIZE) {

    numberOfQualitySamples++;

  } else {

    // The sample that was overwritten
    // is already at the next index.
    //
    // Calculate total again for clarity
    // and reliability.

    qualitySampleTotal = 0;

    for (int i = 0;
         i < MOVING_AVERAGE_SIZE;
         i++) {

      qualitySampleTotal +=
          qualitySamples[i];
    }
  }


  // Calculate average

  int average =
      qualitySampleTotal /
      numberOfQualitySamples;


  return average;
}


// ============================================================
// FUNCTION: QUALITY PERSISTENCE DETECTION
// ============================================================

void monitorQualityPersistence(
  int quality
) {


  // Check if outside normal range

  bool abnormal =
      (
        quality < QUALITY_MIN ||
        quality > QUALITY_MAX
      );


  // ==========================================================
  // ABNORMAL SAMPLE
  // ==========================================================

  if (abnormal) {

    abnormalSampleCount++;


    Serial.print(
      "Abnormal Quality Persistence: "
    );

    Serial.print(
      abnormalSampleCount
    );

    Serial.print(
      " / "
    );

    Serial.println(
      REQUIRED_ABNORMAL_SAMPLES
    );


    // ========================================================
    // GENUINE QUALITY SHIFT
    // ========================================================

    if (
      abnormalSampleCount >=
      REQUIRED_ABNORMAL_SAMPLES
    ) {


      if (!qualityAlertActive) {

        qualityAlertActive = true;


        Serial.println();
        Serial.println(
          "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        );

        Serial.println(
          "   QUALITY SHIFT DETECTED"
        );

        Serial.println(
          "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        );


        Serial.print(
          "New Quality Value: "
        );

        Serial.println(
          quality
        );


        Serial.print(
          "Deviation from Normal: "
        );

        int deviation =
            quality -
            QUALITY_NORMAL_CENTER;


        if (deviation >= 0) {

          Serial.print("+");
        }


        Serial.print(
          deviation
        );

        Serial.println(
          " ADC"
        );


        Serial.print(
          "Persistence: "
        );

        Serial.print(
          REQUIRED_ABNORMAL_SAMPLES
        );

        Serial.println(
          " consecutive samples"
        );


        // Turn ON alert

        digitalWrite(
          QUALITY_LED_PIN,
          HIGH
        );

        digitalWrite(
          BUZZER_PIN,
          HIGH
        );


        Serial.println(
          "Quality LED: ON"
        );

        Serial.println(
          "Buzzer: ON"
        );
      }
    }
  }


  // ==========================================================
  // QUALITY RETURNED TO NORMAL
  // ==========================================================

  else {

    // Reset persistence counter

    if (abnormalSampleCount > 0) {

      Serial.println(
        "Quality returned to normal range."
      );
    }


    abnormalSampleCount = 0;


    // Clear quality alert

    if (qualityAlertActive) {

      qualityAlertActive = false;


      digitalWrite(
        QUALITY_LED_PIN,
        LOW
      );

      digitalWrite(
        BUZZER_PIN,
        LOW
      );


      Serial.println(
        "Quality Alert: CLEARED"
      );

      Serial.println(
        "Quality LED: OFF"
      );

      Serial.println(
        "Buzzer: OFF"
      );
    }
  }
}


// ============================================================
// FUNCTION: TANKER FILL DETECTION
// ============================================================

void detectTankerFill(
  float currentTankLevel
) {


  // If no previous valid reading
  // is available, skip detection

  if (previousTankLevel < 0) {

    return;
  }


  // Calculate level rise

  float levelRise =
      currentTankLevel -
      previousTankLevel;


  // ==========================================================
  // TANKER FILL CONDITION
  // ==========================================================

  if (
    !pumpState &&
    levelRise >= TANKER_RISE_THRESHOLD
  ) {


    Serial.println();

    Serial.println(
      "POSSIBLE TANKER FILL DETECTED"
    );


    Serial.print(
      "Previous Level: "
    );

    Serial.print(
      previousTankLevel,
      1
    );

    Serial.println(
      "%"
    );


    Serial.print(
      "Current Level: "
    );

    Serial.print(
      currentTankLevel,
      1
    );

    Serial.println(
      "%"
    );


    Serial.print(
      "Level Rise: "
    );

    Serial.print(
      levelRise,
      1
    );

    Serial.println(
      "%"
    );


    Serial.println(
      "Pump is OFF."
    );

    Serial.println(
      "External filling may be occurring."
    );


    Serial.println(
      "This is NOT treated as a sensor fault."
    );
  }
}