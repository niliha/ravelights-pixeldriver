#include "PreferencesRaii.hpp"

PreferencesRaii::PreferencesRaii(const char * namespaceName) {
    begin(namespaceName);
}

PreferencesRaii::~PreferencesRaii() {
    end();
}
