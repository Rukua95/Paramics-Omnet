#pragma once

#include "storage.h"
#include "Exceptions.h"
#include <unordered_map>
#include <programmer.h>
#include "Constants.h"

namespace traci_api
{
    class Network
    {
    private:
        static const uint8_t VAR_NDE_POS = 0x42;
        static const uint8_t VAR_NDE_SHP = 0x4e;
        static const uint8_t VAR_RTE_EDGES = 0x54;

        static Network* instance;

        std::unordered_map<BUSROUTE*, std::vector<std::string>> route_links_map;
        std::unordered_map<std::string, BUSROUTE*> route_name_map;

        Network();

        ~Network()
        {
        }

    public:
        static Network* getInstance();
        void getLinkVariable(tcpip::Storage& input, tcpip::Storage& output) const throw(traci_api::NoSuchObjectError);
        void getJunctionVariable(tcpip::Storage& input, tcpip::Storage& output) const throw(traci_api::NoSuchObjectError);
        void getRouteVariable(tcpip::Storage& input, tcpip::Storage& output) const throw(traci_api::NoSuchObjectError);

        /* prevent alternative instantiation */
        Network(Network const&) = delete;
        void operator=(Network const&) = delete;
    };
}
