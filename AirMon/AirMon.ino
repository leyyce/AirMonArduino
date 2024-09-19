#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <EEPROM.h>

#include "bsec.h"  // Proprietary Bosch library for the BME680
#include "Adafruit_CCS811.h"
#include "Adafruit_SGP30.h"

// Service and characteristic UUIDs for the environmental data
#define SERV_UUID "ba59dd58-afe1-4ae0-a151-802a39967e89"
#define BME680_CHAR_UUID "93260fdf-3636-4809-8abc-214217dd419e"
#define CCS811_CHAR_UUID "99603130-0509-4dd7-af3a-bfff45cc1467"
#define SGP30_CHAR_UUID "6dccd7a8-3452-4802-9555-0e020f0d8047"


/* Configure the BSEC library with information about the sensor
    18v/33v = Voltage at Vdd. 1.8V or 3.3V
    3s/300s = BSEC operating mode, BSEC_SAMPLE_RATE_LP or BSEC_SAMPLE_RATE_ULP
    4d/28d = Operating age of the sensor in days
    generic_18v_3s_4d
    generic_18v_3s_28d
    generic_18v_300s_4d
    generic_18v_300s_28d
    generic_33v_3s_4d
    generic_33v_3s_28d
    generic_33v_300s_4d
    generic_33v_300s_28d
*/
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};


#define STATE_SAVE_PERIOD UINT32_C(360 * 60 * 1000)  // 360 minutes - 4 times a day

Bsec bme680;
// Needed to save the BME680 calibration state between resets
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = { 0 };
uint16_t stateUpdateCounter = 0;

Adafruit_CCS811 ccs811;

Adafruit_SGP30 sgp30;

BLECharacteristic *bme680Characteristic;
BLECharacteristic *ccs811Characteristic;
BLECharacteristic *sgp30Characteristic;

bool devConn = false;  // Is a device currently connected?
bool oldConn = false;  // Was a device previously connected?

class AirMonCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.printf("[!] Client connected to server\n");
    devConn = true;
  };
  void onDisconnect(BLEServer *pServer) {
    Serial.printf("[!] Client disconnected from server\n");
    devConn = false;
    // nicht verbunden -> erneut advertising starten
    if (oldConn) {
      oldConn = false;
      Serial.println("[#] Advertising started...");
      BLEDevice::startAdvertising();
    }
  }
};

void setup() {
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);  // Prepare EEPROM for storage of BME680 state
  Serial.begin(115200);
  Serial.println("[#] Initializing sensors...");

  Serial.println("[#] Initializing BME680...");
  bme680.begin(BME68X_I2C_ADDR_HIGH, Wire);
  Serial.println("[#] BSEC library version " + String(bme680.version.major) + "." + String(bme680.version.minor) + "." + String(bme680.version.major_bugfix) + "." + String(bme680.version.minor_bugfix));

  bme680.setConfig(bsec_config_iaq);
  bme680LoadState();

  if (checkBME680SensorStatus() == BSEC_OK) {
    bsec_virtual_sensor_t sensorList[13] = {
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_STABILIZATION_STATUS,
      BSEC_OUTPUT_RUN_IN_STATUS,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
      BSEC_OUTPUT_GAS_PERCENTAGE
    };
    bme680.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
    if (checkBME680SensorStatus() == BSEC_OK) {
      Serial.println("[!] BME680 initialized succesfully");
    }
  } else {
    Serial.println("[!] BME680 initialization failed! Please check your wiring.");
  }

  Serial.println("[#] Initializing CCS811...");
  if (!ccs811.begin()) {
    Serial.println("[!] Failed to start CCS811 sensor! Please check your wiring.");
  } else {
    Serial.println("[!] CCS811 initialized succesfully");
  }
  Serial.println("[#] Initializing SGP30...");
  if (!sgp30.begin()) {
    Serial.println("[!] Failed to start SGP30 sensor! Please check your wiring.");
  } else {
    Serial.println("[!] SGP30 initialized succesfully");
  }

  Serial.println("[#] Starting AirMon server...");

  BLEDevice::init("AirMon");
  BLEServer *airMonServer = BLEDevice::createServer();
  airMonServer->setCallbacks(new AirMonCallbacks());

  BLEService *environmentalDataService = airMonServer->createService(SERV_UUID);

  bme680Characteristic = environmentalDataService->createCharacteristic(
    BME680_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  // Needed descriptor for notify/indicate
  bme680Characteristic->addDescriptor(new BLE2902());

  ccs811Characteristic = environmentalDataService->createCharacteristic(
    CCS811_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  // Needed descriptor for notify/indicate
  ccs811Characteristic->addDescriptor(new BLE2902());

  sgp30Characteristic = environmentalDataService->createCharacteristic(
    SGP30_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  // Needed descriptor for notify/indicate
  sgp30Characteristic->addDescriptor(new BLE2902());

  environmentalDataService->start();

  // advertising starten
  BLEAdvertising *airMonAdvertising = BLEDevice::getAdvertising();
  airMonAdvertising->addServiceUUID(SERV_UUID);
  airMonAdvertising->setScanResponse(true);
  airMonAdvertising->setMinPreferred(0x06);
  airMonAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[#] BLE server has started. Waiting for connection...");
}

void loop() {
  // Caches the sensor readout data before it is copied into the actual buffer used for transmission
  // Data is formated like this "val1;val2;val3;..."
  String output;

  char bme680Value[256];             // Transmission buffer for the BME680 values
  bool bme680Updated = false;        // Has the transmission buffer been updated?
  bool bme680HasTempAndHum = false;  // Was the sensor able to read values. In this case especially temperature and humidity data?
  float bme680temp;
  float bme680hum;
  if (bme680.run()) {  // If new data is available
    bme680temp = bme680.temperature;
    bme680hum = bme680.humidity;
    float bme680CO2 = bme680.co2Equivalent;
    float bme680voc = bme680.breathVocEquivalent;
    output = String(bme680.staticIaq);
    output += ";" + String(bme680.iaqAccuracy);
    output += ";" + String(bme680CO2);
    output += ";" + String(bme680voc);
    output += ";" + String(bme680.runInStatus);
    output += ";" + String(bme680temp);
    output += ";" + String(bme680hum);
    output += ";" + String(calculateIAQ(bme680CO2, bme680voc * 1000, bme680hum, bme680temp));
    Serial.println("[#] BME680: " + output);
    output.toCharArray(bme680Value, 256);
    bme680Updated = true;
    bme680HasTempAndHum = true;
    bme680UpdateState();
  } else {
    int code = checkBME680SensorStatus();
    if (code != BSEC_OK) {
      output = "BME680 error code: " + String(code);
      output.toCharArray(bme680Value, 256);
      bme680Updated = true;
    }
  }

  // If the BME680 had data available to read this loop cycle,
  // set the meassured temperature and humitdity for meassurment corrections on the CCS881 and SGP30 sensors
  if (bme680HasTempAndHum) {
    ccs811.setEnvironmentalData(bme680hum, bme680temp);
    sgp30.setHumidity(getAbsoluteHumidity(bme680temp, bme680hum));
  }

  char ccs811Value[256];       // Transmission buffer for the CCS811 values
  bool ccs811Updated = false;  // Has the transmission buffer been updated?
  if (ccs811.available()) {
    if (!ccs811.readData()) {
      uint16_t eCO2 = ccs811.geteCO2();
      uint16_t tvoc = ccs811.getTVOC();
      Serial.print("[#] CCS811: eCO2: " + String(eCO2) + "ppm; ");
      Serial.print("TVOC: " + String(tvoc));
      output = String(eCO2);
      output += ";" + String(tvoc);
      if (bme680HasTempAndHum) {  // If temperature and humidity data are available...
        // ...calculate own IAQ score...
        uint8_t iaq = calculateIAQ(eCO2, tvoc, bme680hum, bme680temp);
        output += ";" + String(iaq);
        Serial.println("; IAQ: " + String(iaq));
        // ...and prepare the data for transmission
        output.toCharArray(ccs811Value, 256);
        ccs811Updated = true;
      } else {
        // Don't transmit data this itteration, because IAQ couldnt be calculated.
        Serial.println("");
      }
    } else {
      output = "Error reading from CCS811 sensor";
      Serial.println("[!] " + output);
      output.toCharArray(ccs811Value, 256);
      ccs811Updated = true;
    }
  } else {
    output = "CCS811 sensor data not available";
    Serial.println("[!] " + output);
    output.toCharArray(ccs811Value, 256);
    ccs811Updated = true;
  }

  char sgp30Value[256];       // Transmission buffer for the SGP30 values
  bool sgp30Updated = false;  // Has the transmission buffer been updated?
  if (sgp30.IAQmeasure()) {
    uint16_t eCO2 = sgp30.eCO2;
    uint16_t tvoc = sgp30.TVOC;
    Serial.print("[#] SGP30: eCO2: " + String(eCO2) + "ppm; ");
    Serial.print("TVOC: " + String(tvoc));
    output = String(eCO2);
    output += ";" + String(tvoc);
    if (bme680HasTempAndHum) {  // If temperature and humidity data are available...
      // ...calculate own IAQ score...
      uint8_t iaq = calculateIAQ(eCO2, tvoc, bme680hum, bme680temp);
      output += ";" + String(iaq);
      Serial.println("; IAQ: " + String(iaq));
      // ...and prepare the data for transmission
      output.toCharArray(sgp30Value, 256);
      sgp30Updated = true;
    } else {
      // Don't transmit data this itteration, because IAQ couldnt be calculated.
      Serial.println("");
    }
  } else {
    output = "Error reading from SGP30 sensor";
    Serial.println("[!] " + output);
    output.toCharArray(sgp30Value, 256);
    sgp30Updated = true;
  }

  // If a device is connected and the corresponding sensors did meassure some new data in this loop itteration,
  // notify the connected device with the updated data.
  if (devConn) {
    if (bme680Updated) {
      bme680Characteristic->setValue(bme680Value);
      bme680Characteristic->notify();

      if (ccs811Updated) {
        ccs811Characteristic->setValue(ccs811Value);
        ccs811Characteristic->notify();
      }
      if (sgp30Updated) {
        sgp30Characteristic->setValue(sgp30Value);
        sgp30Characteristic->notify();
      }
    }

    oldConn = true;  // Change flag to indicate a device was previously connected.
  }
  delay(1000);
}

// Checks for errors with the BME680 sensor and prints it if error occured.
// Returns 0 if no error was found or any non zero value in case of an error.
int checkBME680SensorStatus(void) {
  if (bme680.bsecStatus != BSEC_OK) {
    if (bme680.bsecStatus < BSEC_OK) {
      Serial.println("[!] BSEC error code : " + String(bme680.bsecStatus));
    } else {
      Serial.println("[!] BSEC warning code : " + String(bme680.bsecStatus));
    }
  }

  if (bme680.bme68xStatus != BME68X_OK) {
    if (bme680.bme68xStatus < BME68X_OK) {
      Serial.println("[!] BME68X error code : " + String(bme680.bme68xStatus));
    } else {
      Serial.println("[!] BME68X warning code : " + String(bme680.bme68xStatus));
    }
  }
  return bme680.bsecStatus | bme680.bme68xStatus;
}

// Loads a previously saved BME680 state from EEPROM if available,
// else erases EEPROM
void bme680LoadState(void) {
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("[!] BME680: Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
    }

    bme680.setState(bsecState);
    checkBME680SensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("[!] BME680: Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

// Checks if BME680 state saved in EEPROM should be updated
// and updates it if it's the case
void bme680UpdateState() {
  bool update = false;
  if (stateUpdateCounter == 0) {
    /* First state update when IAQ accuracy is >= 3 */
    if (bme680.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD minutes */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    bme680.getState(bsecState);
    if (checkBME680SensorStatus() == BSEC_OK) {
      Serial.println("[!] BME680: Writing state to EEPROM");

      for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
        EEPROM.write(i + 1, bsecState[i]);
      }

      EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
      EEPROM.commit();
    }
  }
}

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));  // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                 // [mg/m^3]
  return absoluteHumidityScaled;
}

// float calculateIAQ(uint16_t eCO2, uint16_t tvoc, float temp, float hum) {
//   float eCO2Contrib = ((eCO2) / (8000. - 400.)) * 500.;
//   float tvocContrib = (tvoc / 6000.) * 500.;
//   float tempPenalty = 1 + abs((temp - 22.5) / 22.5) * 0.1;
//   float humPenalty = 1 + abs((hum - 45) / 45) * 0.1;
//   return (eCO2Contrib * 0.45 + tvocContrib * 0.55) * tempPenalty * humPenalty;
// }

uint8_t calculateIAQ(float eCO2, float tvoc, float hum, float temp) {
  uint8_t eCO2Value = geteCO2IAQValue(eCO2);
  uint8_t tvocValue = geteTVOCIAQValue(tvoc);
  uint8_t humTempValue = getHumTempIAQValue(hum, temp);

  return int(roundf((eCO2Value + tvocValue + humTempValue) / 3.));
}

static uint8_t geteCO2IAQValue(float eCO2) {
  if (eCO2 <= 400) return 1;
  if (eCO2 <= 1000) return 2;
  if (eCO2 <= 1500) return 3;
  if (eCO2 <= 2000) return 4;
  if (eCO2 <= 5000) return 5;
  if (eCO2 > 5000) return 6;
}

static uint8_t geteTVOCIAQValue(float tvoc) {
  if (tvoc <= 65) return 1;
  if (tvoc <= 220) return 2;
  if (tvoc <= 660) return 3;
  if (tvoc <= 2200) return 4;
  if (tvoc <= 3850) return 5;
  if (tvoc > 3850) return 6;
}

static uint8_t getHumTempIAQValue(float humidity, float temperature) {
  int table[9][13] = {
    { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6 },  // 10% row
    { 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6 },  // 20% row
    { 6, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 5, 6 },  // 30% row
    { 6, 5, 5, 4, 3, 3, 3, 2, 2, 3, 4, 5, 6 },  // 40% row
    { 5, 5, 4, 3, 2, 1, 1, 1, 2, 3, 4, 5, 6 },  // 50% row
    { 5, 4, 3, 2, 1, 1, 1, 2, 2, 3, 4, 5, 6 },  // 60% row
    { 5, 4, 3, 2, 1, 1, 1, 2, 3, 4, 5, 5, 6 },  // 70% row
    { 5, 4, 2, 2, 2, 2, 2, 3, 4, 5, 5, 5, 6 },  // 80% row
    { 6, 5, 4, 3, 3, 3, 3, 4, 5, 5, 6, 6, 6 },  // 90% row
  };

  // Define the humidity and temperature steps
  int humiditySteps[] = { 10, 20, 30, 40, 50, 60, 70, 80, 90 };                     // Humidity steps in %
  int temperatureSteps[] = { 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28 };  // Temperature steps in °C

  // Find the closest humidity index
  int humidityIndex = 0;
  for (int i = 0; i < 9; i++) {
    if (humidity <= humiditySteps[i]) {
      humidityIndex = i;
      break;
    }
  }

  // Find the closest temperature index
  int temperatureIndex = 0;
  for (int i = 0; i < 13; i++) {
    if (temperature <= temperatureSteps[i]) {
      temperatureIndex = i;
      break;
    }
  }

  // Return the value from the table
  return table[humidityIndex][temperatureIndex];
}