#pragma once

#include "Preferences.h"

class PreferencesRaii : private Preferences {
 public:
    PreferencesRaii(const char *namespaceName);
    ~PreferencesRaii();

    using Preferences::isKey;

    // Required for output config
    using Preferences::getBytes;
    using Preferences::putBytes;

    // Required for instance id
    using Preferences::getString;
    using Preferences::putString;
};