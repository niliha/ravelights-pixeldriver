#pragma once

#include <Arduino.h>
#include <WebServer.h>

#include "AbstractNetworkInterface.hpp"

class RestApi : public AbstractNetworkInterface {
    public:
    RestApi(int port = 80);
    virtual void start() override;
    virtual void handleReceived() override;

    private:
    WebServer server_;
    const int port_;

    void on_get_config();
    void on_set_config();
};