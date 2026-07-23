#pragma once

#include <Preferences.h>

#include "DeviceSettings.h"

class PreferencesSettingsStore {
 public:
  bool begin();
  DeviceSettings load() const;
  bool save(const DeviceSettings& settings);

 private:
  mutable Preferences preferences_;
  bool ready_ = false;
};
