#pragma once

#include <WebServer.h>

#include "AbstractInterfaceHandler.hpp"

class RestApi : public AbstractInterfaceHandler {
    public:
    RestApi(int port = 80);
    virtual void start() override;
    virtual void handleReceived() override;

    private:
    const int port_;
    WebServer server_;

    void onGetConfig();
    void onSetConfig();
};