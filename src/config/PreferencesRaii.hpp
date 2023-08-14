#pragma once

#include "Preferences.h"

class PreferencesRaii {
 public:
    PreferencesRaii(const char *namespaceName);
    ~PreferencesRaii();

    size_t putBytes(const char *key, const void *value, size_t len);
    size_t getBytes(const char *key, void *buf, size_t maxLen);
    bool isKey(const char *key);

 private:
    Preferences preferences_;
};