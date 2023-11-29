#pragma once

class AbstractInterfaceHandler {
 public:
    virtual ~AbstractInterfaceHandler() = default;
    virtual void start() = 0;
    virtual void handleReceived() = 0;
};