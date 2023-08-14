#include "PreferencesRaii.hpp"

PreferencesRaii::PreferencesRaii(const char * namespaceName) {
    preferences_.begin(namespaceName);
}

PreferencesRaii::~PreferencesRaii() {
    preferences_.end();
}

size_t PreferencesRaii::putBytes(const char *key, const void *value, size_t len) {
    preferences_.putBytes(key, value, len);
}
size_t PreferencesRaii::getBytes(const char *key, void *buf, size_t maxLen) {
    preferences_.getBytes(key, buf, maxLen);
}

bool PreferencesRaii::isKey(const char *key) {
    return preferences_.isKey(key);
}