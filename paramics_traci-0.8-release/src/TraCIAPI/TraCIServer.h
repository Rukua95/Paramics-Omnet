#pragma once

#include "socket.h"
#include "storage.h"
#include "programmer.h"
#include <thread>
#include "storage.h"
#include "Constants.h"
#include "VehicleManager.h"
#include "Exceptions.h"
#include "Network.h"
#include "Subscriptions.h"
#include <windows.h>
#include "Simulation.h"
#include "Subscriptions.h"

namespace traci_api
{
    class VariableSubscription;

    class TraCIServer
    {
    public:

        TraCIServer(int port);
        ~TraCIServer();
        void waitForConnection();
        void close();

        // lockstep methods
        void preStep();
        void postStep();

    private:

        tcpip::Socket ssocket;
        // in/out storages
        tcpip::Storage outgoing;
        tcpip::Storage incoming;
        size_t incoming_size;

        // timestep params
        bool multiple_timestep;
        int target_time;

        bool running;
        int port;
        std::vector<VariableSubscription*> subs;

        std::mutex socket_lock;

        void cmdGetSimVar(uint8_t simvar);
        void cmdGetVhcVar(tcpip::Storage& input);
        void cmdGetNetworkVar(tcpip::Storage& input, uint8_t cmdid);
        void cmdSetVhcState(tcpip::Storage& input);
        void cmdGetPolygonVar(tcpip::Storage& input);
        void cmdGetVtpVar(tcpip::Storage& input);

        bool parseCommand(tcpip::Storage& storage);
        void writeStatusResponse(uint8_t cmdId, uint8_t cmdStatus, std::string description);
        void writeVersion();
        void sendResponse();

        void writeToOutputWithSize(tcpip::Storage& storage, bool force_extended);
        static void writeToStorageWithSize(tcpip::Storage& src, tcpip::Storage& dest, bool force_extended);

        void addSubscription(uint8_t sub_type, std::string object_id, int start_time, int end_time, std::vector<uint8_t> variables);
        void processSubscriptions(tcpip::Storage& sub_store);

        //commands
        void cmdShutDown();
    };
}
