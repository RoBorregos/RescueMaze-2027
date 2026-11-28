#pragma once

#include <EEPROM.h>
#include <stddef.h>
#include <stdint.h>

namespace CalibrationStorage {
constexpr int kTcsAddress = 128;
constexpr int kRecordCapacity = 64;

template <typename Profile>
struct Record {
    uint32_t version;
    uint32_t generation;
    Profile profile;
    uint32_t checksum;
};

template <typename Profile>
uint32_t checksum(const Record<Profile> &record) {
    const auto *bytes = reinterpret_cast<const uint8_t *>(&record.profile);
    uint32_t value = 2166136261u ^ record.generation;
    for (size_t i = 0; i < sizeof(record.profile); ++i) {
        value = (value ^ bytes[i]) * 16777619u;
    }
    return value;
}

template <typename Profile>
bool readRecord(int address, uint32_t version, Record<Profile> &record) {
    static_assert(sizeof(Record<Profile>) <= kRecordCapacity, "Calibration record exceeds its EEPROM region");
    EEPROM.get(address, record);
    return record.version == version && record.checksum == checksum(record);
}

template <typename Profile>
int latestSlot(int address, uint32_t version, Record<Profile> &record) {
    const bool firstValid = readRecord(address, version, record);
    Record<Profile> second{};
    const bool secondValid = readRecord(address + kRecordCapacity, version, second);
    const uint32_t advance = second.generation - record.generation;
    if (secondValid && (!firstValid || (advance != 0 && advance < 0x80000000u))) {
        record = second;
        return 1;
    }
    return firstValid ? 0 : -1;
}

template <typename Profile>
bool load(int address, uint32_t version, Profile &profile) {
    Record<Profile> record{};
    if (latestSlot(address, version, record) < 0) return false;
    profile = record.profile;
    return true;
}

template <typename Profile>
bool save(int address, uint32_t version, const Profile &profile) {
    Record<Profile> current{};
    const int active = latestSlot(address, version, current);
    // Write the other slot so an interrupted save leaves the previous record usable.
    const int destination = active == 0 ? address + kRecordCapacity : address;
    Record<Profile> record{version, active < 0 ? 1u : current.generation + 1, profile, 0};
    record.checksum = checksum(record);
    EEPROM.put(destination, record);
    Record<Profile> stored{};
    return readRecord(destination, version, stored) && stored.checksum == record.checksum;
}
}
