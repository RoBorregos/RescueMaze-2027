#include "VLX.h"

#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint8_t kDefaultAddress = 0x29;
constexpr uint8_t kFirstAssignedAddress = 0x30;
constexpr uint8_t kStartupAddress = kFirstAssignedAddress + VlxID::kCount;
constexpr uint8_t kAddressRegister = 0x8A;
constexpr uint8_t kSoftResetRegister = 0xBF;
constexpr uint8_t kAddressNack = 2;
constexpr uint32_t kTimingBudgetUs = 33000;
constexpr uint32_t kReadTimeoutMs = 2 * MazeConfig::kVlxPeriodMs;
constexpr uint8_t kSignalFail = 2;
constexpr uint8_t kPhaseFail = 4;
constexpr uint8_t kSamplesPerWindow = 4;
constexpr uint8_t kRequiredValidSamples = 3;
constexpr uint8_t kNavigationSamples = 2;

uint8_t probeAddress(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission();
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool moveAddress(uint8_t from, uint8_t to) {
    if (probeAddress(to) != kAddressNack || !writeRegister(from, kAddressRegister, to)) return false;
    delay(10);
    return probeAddress(to) == 0 && probeAddress(from) == kAddressNack;
}

bool resetAtDefaultAddress() {
    if (!writeRegister(kDefaultAddress, kSoftResetRegister, 0)) return false;
    delay(2);
    if (!writeRegister(kDefaultAddress, kSoftResetRegister, 1)) return false;
    delay(15);
    return true;
}

bool stageUnswitchedSensor() {
    uint8_t found = 0;
    // A Teensy reset can leave the powered sensor at a previously assigned address.
    for (uint8_t address = kDefaultAddress; address <= kStartupAddress;
         address = address == kDefaultAddress ? kFirstAssignedAddress : address + 1) {
        const uint8_t result = probeAddress(address);
        if (result == kAddressNack) continue;
        if (result != 0 || found != 0) return false;
        found = address;
    }
    return found == 0 || found == kStartupAddress || moveAddress(found, kStartupAddress);
}

uint16_t median(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) {
        const uint16_t first = a;
        a = b;
        b = first;
    }
    return c < a ? a : (c > b ? b : c);
}
}

VLX::VLX(uint8_t sensorIndex) : index(sensorIndex) {}

void VLX::setSensorIndex(uint8_t sensorIndex) {
    if (!initialized) index = sensorIndex;
}

bool VLX::prepareBus() {
    for (uint8_t i = 0; i < VlxID::kCount; ++i) {
        if (i + 1 == VlxID::kWithoutXshut) continue;
        const uint8_t pin = Pins::kVlxXshut[i];
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    }
    delay(10);
    // Free the operational addresses while each sensor with XSHUT boots.
    if (VlxID::kWithoutXshut == VlxID::kNone || stageUnswitchedSensor()) return true;
    Serial.println("VLX: no se pudo aislar el sensor sin XSHUT antes de inicializar los demas.");
    return false;
}

bool VLX::begin() {
    if (initialized) return true;
    if (index >= sizeof(Pins::kVlxXshut)) return false;
    const bool hasXshut = index + 1 != VlxID::kWithoutXshut;
    const uint8_t address = hasXshut ? kFirstAssignedAddress + index : kDefaultAddress;
    const uint8_t pin = Pins::kVlxXshut[index];
    if (hasXshut) {
        pinMode(pin, INPUT);  // Release XSHUT to the module's pull-up.
        delay(15);
    } else if (probeAddress(kStartupAddress) == 0) {
        if (!moveAddress(kStartupAddress, kDefaultAddress) || !resetAtDefaultAddress()) {
            Serial.printf("VLX%u: no se pudo reiniciar en 0x29 en Wire.\n", unsigned(index + 1));
            return false;
        }
    }
    const uint8_t probe = probeAddress(kDefaultAddress);
    const char *failure = nullptr;
    if (probe != 0) failure = "sin respuesta en 0x29";
    else if (!sensor.begin(address, false, &Wire)) failure = "fallo de identificacion o direccion";
    else if (!sensor.setMeasurementTimingBudgetMicroSeconds(kTimingBudgetUs)) failure = "fallo de tiempo de medicion";
    else if (!sensor.startRangeContinuous(MazeConfig::kVlxPeriodMs)) failure = "fallo al iniciar medicion continua";
    initialized = failure == nullptr;
    if (!hasXshut) {
        Serial.printf("VLX%u Wire 0x%02X sin XSHUT: %s (I2C=%u, estado=%d).\n",
                      unsigned(index + 1), unsigned(address), initialized ? "activo" : failure, unsigned(probe),
                      int(sensor.Status));
    } else if (!initialized) {
        Serial.printf("VLX%u XSHUT=%u: %s (I2C=%u).\n",
                      unsigned(index + 1), unsigned(pin), failure, unsigned(probe));
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
        delay(5);
    }
    return initialized;
}

void VLX::updateDistance() {
    VlxSample sample{};
    readSample(sample);
}

bool VLX::readRangeResult(uint16_t &millimeters) {
    const uint16_t raw = sensor.readRangeResult();
    if (raw == 0 || raw == 0xFFFF || sensor.readRangeStatus() != 0) return false;
    millimeters = raw;
    return true;
}

bool VLX::readReadyMillimeters(uint16_t &millimeters) {
    return initialized && sensor.isRangeComplete() && sensor.Status == VL53L0X_ERROR_NONE &&
           readRangeResult(millimeters);
}

bool VLX::readSample(VlxSample &sample) {
    SampleWindow window{};
    startWindow(window);
    while (!pollWindow(window, kSamplesPerWindow, kRequiredValidSamples)) delay(1);
    valid = window.validCount == kRequiredValidSamples;
    if (!valid) return false;
    sample = window.sample;
    lastMillimeters = sample.millimeters;
    distance = lastMillimeters / 10.0f;
    return true;
}

void VLX::startNavigationRead() {
    startWindow(navigationWindow);
    navigationPending = true;
}

bool VLX::pollNavigationRead(VlxReading &reading, uint16_t &millimeters) {
    if (navigationPending && !pollWindow(navigationWindow, kNavigationSamples, 1)) return false;
    navigationPending = false;
    const SampleWindow &window = navigationWindow;
    if (window.validCount != 0) {
        millimeters = window.sample.millimeters;
        reading = VlxReading::kDistance;
    } else {
        reading = window.noReturnSamples == kNavigationSamples ? VlxReading::kNoReturn : VlxReading::kUnknown;
    }
    return true;
}

void VLX::startWindow(SampleWindow &window) {
    window = {};
    // Discard a queued result and the conversion that may already be underway.
    // Filter only samples acquired for this read, after the robot has stopped.
    if (initialized && sensor.isRangeComplete() && sensor.Status == VL53L0X_ERROR_NONE) {
        sensor.readRangeResult();
    }
    window.waitingSince = millis();
}

bool VLX::pollWindow(SampleWindow &window, uint8_t maxSamples, uint8_t requiredValid) {
    if (!initialized) return true;
    if (!sensor.isRangeComplete()) {
        if (millis() - window.waitingSince < kReadTimeoutMs) return false;
        Serial.printf("VLX%u: tiempo de espera agotado.\n", unsigned(index + 1));
        return true;
    }
    if (sensor.Status != VL53L0X_ERROR_NONE) {
        Serial.printf("VLX%u: error al consultar la muestra (%d).\n",
                      unsigned(index + 1), int(sensor.Status));
        return true;
    }

    uint16_t raw = 0;
    const bool measured = readRangeResult(raw);
    if (sensor.Status != VL53L0X_ERROR_NONE) {
        Serial.printf("VLX%u: error al leer la muestra (%d).\n",
                      unsigned(index + 1), int(sensor.Status));
        return true;
    }
    const bool discard = window.attempts++ == 0;
    auto &samples = window.sample.raw;
    if (!discard && measured) {
        samples[window.validCount++] = raw;
    } else if (!discard) {
        const uint8_t status = sensor.readRangeStatus();
        if (status == kSignalFail || status == kPhaseFail) ++window.noReturnSamples;
        else Serial.printf("VLX%u: muestra descartada, RangeStatus=%u.\n",
                           unsigned(index + 1), unsigned(status));
    }
    if (window.validCount == requiredValid) {
        window.sample.millimeters = requiredValid == 1 ? raw : median(samples[0], samples[1], samples[2]);
        return true;
    }
    if (window.attempts <= maxSamples) {
        window.waitingSince = millis();
        return false;
    }
    if (window.noReturnSamples != maxSamples) {
        Serial.printf("VLX%u: %u muestras validas, %u sin retorno util.\n",
                      unsigned(index + 1), unsigned(window.validCount), unsigned(window.noReturnSamples));
    }
    return true;
}

float VLX::getDistance() {
    updateDistance();
    return valid ? distance : kOutOfRange;
}

bool VLX::readMillimeters(uint16_t &millimeters) {
    updateDistance();
    if (!valid) return false;
    millimeters = lastMillimeters;
    return true;
}

void VLX::printDistance() {
    uint16_t millimeters = 0;
    if (readMillimeters(millimeters)) Serial.printf("VLX%u: %u mm\n", unsigned(index + 1), unsigned(millimeters));
    else Serial.printf("VLX%u: sin lectura valida\n", unsigned(index + 1));
}

uint16_t VLX::wallThresholdMm() const {
    return MazeConfig::wallDistanceMm(VlxID::sideOf(index + 1));
}

bool VLX::isWall(bool &wall) {
    uint16_t millimeters = 0;
    if (!readMillimeters(millimeters)) return false;
    wall = millimeters < wallThresholdMm();
    return true;
}

bool VLX::isWall() {
    bool wall = false;
    return isWall(wall) && wall;
}
