#pragma once
#include "storage.h"
#include <mutex>

#include <programmer.h>
#include "TraCIServer.h"
#include "Constants.h"
#include "VehicleManager.h"
#include <algorithm>

namespace traci_api
{
    class Simulation
    {
    public:
        // Command constants:
        // simulation time
        static const uint8_t VAR_SIMTIME = 0x70;

        // loaded vehicles
        static const uint8_t VAR_LOADEDVHC_CNT = 0x71;
        static const uint8_t VAR_LOADEDVHC_LST = 0x72;

        // departed vehicles
        static const uint8_t VAR_DEPARTEDVHC_CNT = 0x73;
        static const uint8_t VAR_DEPARTEDVHC_LST = 0x74;

        // arrived vehicles
        static const uint8_t VAR_ARRIVEDVHC_CNT = 0x79;
        static const uint8_t VAR_ARRIVEDVHC_LST = 0x7a;

        // simulation timestep size
        static const uint8_t VAR_TIMESTEPSZ = 0x7b;

        // network boundary box
        static const uint8_t VAR_NETWORKBNDS = 0x7c;

        // teleporting vehicles
        static const uint8_t VAR_VHCSTARTTELEPORT_CNT = 0x75;
        static const uint8_t VAR_VHCSTARTTELEPORT_LST = 0x76;
        static const uint8_t VAR_VHCENDTELEPORT_CNT = 0x77;
        static const uint8_t VAR_VHCENDTELEPORT_LST = 0x78;

        // parking
        static const uint8_t VAR_VHCSTARTPARK_CNT = 0x6c;
        static const uint8_t VAR_VHCSTARTPARK_LST = 0x6d;
        static const uint8_t VAR_VHCENDPARK_CNT = 0x6e;
        static const uint8_t VAR_VHCENDPARK_LST = 0x6f;

        //Simulation();
        ~Simulation();
        // prevent alternative instantiation
        Simulation(Simulation const&) = delete;
        void operator =(Simulation const&) = delete;

        static Simulation* getInstance();
        static void deleteInstance();

        bool packSimulationVariable(uint8_t varID, tcpip::Storage& result_store);
        void getSimulationVariable(uint8_t varID, tcpip::Storage& result);

        float getCurrentTimeSeconds();
        int getCurrentTimeMilliseconds();
        int getTimeStepSizeMilliseconds();

        void getRealNetworkBounds(double& llx, double& lly, double& urx, double& ury);

    private:
        static Simulation* instance;
        Simulation();
        int stepcnt;
    };
}
