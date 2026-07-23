#include "PreferencesSettingsStore.h"

bool PreferencesSettingsStore::begin() {
  ready_ = preferences_.begin("motor_cfg_v2", false);
  return ready_;
}

DeviceSettings PreferencesSettingsStore::load() const {
  const DeviceSettings defaults = DeviceSettings::defaults();
  if (!ready_ ||
      preferences_.getBytesLength("snapshot") != sizeof(DeviceSettings)) {
    return defaults;
  }

  DeviceSettings stored{};
  if (preferences_.getBytes("snapshot", &stored, sizeof(stored)) !=
        sizeof(stored) ||
      !validateDeviceSettings(stored)) {
    return defaults;
  }
  return stored;
}

bool PreferencesSettingsStore::save(const DeviceSettings& settings) {
  return ready_ && validateDeviceSettings(settings) &&
         preferences_.putBytes("snapshot", &settings, sizeof(settings)) ==
           sizeof(settings);
}
