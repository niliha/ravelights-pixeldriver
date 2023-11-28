#pragma once

class AbstractNetworkInterface {
 public:
    virtual ~AbstractNetworkInterface() = default;
    virtual void start() = 0;
    virtual void handleReceived() = 0;
};