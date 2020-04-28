#pragma once
#include <vector>
#include "Utils.h"
#include <unordered_map>
#include "Triggers.h"
#include "Constants.h"
#include <string>
#include "Simulation.h"
#include "Exceptions.h"
#include <map>


namespace std
{
    class runtime_error;
}

namespace tcpip
{
    class Storage;
}

namespace traci_api
{
    class NotImplementedError;
    class NoSuchObjectError;
    class NoSuchObjectError;
    class LaneSetTrigger;
    class BaseSpeedController;

    class VehicleManager
    {
    public:

        /* vehicle variables */
        static const uint8_t VAR_VHC_LIST = 0x00;
        static const uint8_t VAR_VHC_COUNT = 0x01;
        static const uint8_t VAR_VHC_SPEED = 0x40;
        static const uint8_t VAR_VHC_POS = 0x42;
        static const uint8_t VAR_VHC_POS3D = 0x39;
        static const uint8_t VAR_VHC_ANGLE = 0x43;
        static const uint8_t VAR_VHC_ROAD = 0x50;
        static const uint8_t VAR_VHC_LANE = 0x51;
        static const uint8_t VAR_VHC_LANEIDX = 0x52;
        static const uint8_t VAR_VHC_TYPE = 0x4f;
        static const uint8_t VAR_VHC_LENGTH = 0x44;
        static const uint8_t VAR_VHC_HEIGHT = 0xbc;
        static const uint8_t VAR_VHC_WIDTH = 0x4d;
        static const uint8_t VAR_VHC_SLOPE = 0x36;

        /* not implemented yet*/
        static const uint8_t VAR_VHC_ROUTE = 0x53;
        static const uint8_t VAR_VHC_ROUTEIDX = 0x69;
        static const uint8_t VAR_VHC_EDGES = 0x54;
        static const uint8_t VAR_VHC_COLOR = 0x45;
        static const uint8_t VAR_VHC_LANEPOS = 0x56;
        static const uint8_t VAR_VHC_DIST = 0x84;
        static const uint8_t VAR_VHC_SIGNALST = 0x5b;
        static const uint8_t VAR_VHC_CO2 = 0x60;
        static const uint8_t VAR_VHC_CO = 0x61;
        static const uint8_t VAR_VHC_HC = 0x62;
        static const uint8_t VAR_VHC_PMX = 0x63;
        static const uint8_t VAR_VHC_NOX = 0x64;
        static const uint8_t VAR_VHC_FUELCONS = 0x65;
        static const uint8_t VAR_VHC_NOISE = 0x66;
        static const uint8_t VAR_VHC_ELECCONS = 0x71;
        static const uint8_t VAR_VHC_BESTLANES = 0xb2;
        static const uint8_t VAR_VHC_STOPSTATE = 0xb5;
        static const uint8_t VAR_VHC_VMAX = 0x41;
        static const uint8_t VAR_VHC_ACCEL = 0x46;
        static const uint8_t VAR_VHC_DECEL = 0x47;
        static const uint8_t VAR_VHC_TAU = 0x48;
        static const uint8_t VAR_VHC_SIGMA = 0x5d;
        static const uint8_t VAR_VHC_SPDFACTOR = 0x5e;
        static const uint8_t VAR_VHC_SPEEDDEV = 0x5f;
        static const uint8_t VAR_VHC_VCLASS = 0x49;
        static const uint8_t VAR_VHC_EMSCLASS = 0x4a;
        static const uint8_t VAR_VHC_SHAPE = 0x4b;
        static const uint8_t VAR_VHC_MINGAP = 0x4c;
        static const uint8_t VAR_VHC_WAITTIME = 0x7a;
        static const uint8_t VAR_VHC_NEXTTLS = 0x70;
        static const uint8_t VAR_VHC_SPEEDMODE = 0xb3;
        static const uint8_t VAR_VHC_ALLOWEDSPD = 0xb7;
        static const uint8_t VAR_VHC_LINE = 0xbd;
        static const uint8_t VAR_VHC_PNUMBER = 0x67;
        static const uint8_t VAR_VHC_VIAEDGES = 0xbe;
        static const uint8_t VAR_VHC_NONTRACISPD = 0xb1;
        static const uint8_t VAR_VHC_VALIDROUTE = 0x92;

        // only for types
        static const uint8_t VAR_VHC_MAXLATSPD = 0xba;
        static const uint8_t VAR_VHC_LATGAP = 0Xbb;
        static const uint8_t VAR_VHC_LATALIGN = 0Xb9;

        /* vehicle states */
        static const uint8_t STA_VHC_STOP = 0x12;
        static const uint8_t STA_VHC_CHANGELANE = 0x13;
        static const uint8_t STA_VHC_SLOWDWN = 0x14;
        static const uint8_t STA_VHC_RESUME = 0x19;
        static const uint8_t STA_VHC_CHANGETARGET = 0x31;
        static const uint8_t STA_VHC_SPEED = 0x40;
        static const uint8_t STA_VHC_COLOUR = 0x45;
        static const uint8_t STA_VHC_CHANGEROUTEID = 0x53;
        static const uint8_t STA_VHC_CHANGEROUTE = 0x57;
        static const uint8_t STA_VHC_CHANGEEDGETTIME = 0x58;
        static const uint8_t STA_VHC_SIGNALSTATES = 0x5b;
        static const uint8_t STA_VHC_MOVETO = 0x5c;
        static const uint8_t STA_VHC_MOVETOXY = 0xb4;
        static const uint8_t STA_VHC_REROUTE = 0x90;
        static const uint8_t STA_VHC_SPEEDMODE = 0xb3;
        static const uint8_t STA_VHC_SPEEDFACTOR = 0x5e;
        static const uint8_t STA_VHC_MAXSPEED = 0x41;
        static const uint8_t STA_VHC_CHANGELANEMODE = 0xb6;
        static const uint8_t STA_VHC_ADD = 0x80;
        static const uint8_t STA_VHC_ADDFULL = 0x85;
        static const uint8_t STA_VHC_REMOVE = 0x81;
        static const uint8_t STA_VHC_LENGTH = 0x44;
        static const uint8_t STA_VHC_VHCCLASS = 0x49;
        static const uint8_t STA_VHC_EMSCLASS = 0x4a;
        static const uint8_t STA_VHC_WIDTH = 0x4d;
        static const uint8_t STA_VHC_HEIGHT = 0xbc;
        static const uint8_t STA_VHC_MINGAP = 0x4c;
        static const uint8_t STA_VHC_SHAPECLASS = 0x4b;
        static const uint8_t STA_VHC_ACC = 0x46;
        static const uint8_t STA_VHC_DEC = 0x47;
        static const uint8_t STA_VHC_IMPERFECTION = 0x5d;
        static const uint8_t STA_VHC_TAU = 0x48;
        static const uint8_t STA_VHC_TYPE = 0x4f;
        static const uint8_t STA_VHC_VIA = 0xbe;

        static VehicleManager* getInstance();
        static void deleteInstance();

        void reset();

        void packVehicleVariable(tcpip::Storage& input, tcpip::Storage& output) throw(NotImplementedError, std::runtime_error, NoSuchObjectError);
        void getVehicleVariable(std::string vid, uint8_t varID, tcpip::Storage& output) throw(NotImplementedError, std::runtime_error, NoSuchObjectError);
        void setVehicleState(tcpip::Storage& input);

        void vehicleDepart(VEHICLE* vehicle);
        void vehicleArrive(VEHICLE* vehicle);

        std::vector<std::string> getDepartedVehicles();
        std::vector<std::string> getArrivedVehicles();
        int getDepartedVehicleCount() const;
        int getArrivedVehicleCount() const;

        int currentVehicleCount() const;
        std::vector<std::string> getVehiclesInSim();

        float getSpeed(std::string vid) throw(NoSuchObjectError);

        PositionalData getPosition(std::string vid) throw(NoSuchObjectError);
        DimensionalData getDimensions(std::string vid) throw(NoSuchObjectError);

        std::string getRoadID(std::string vid) throw(NoSuchObjectError);
        std::string getLaneID(std::string vid) throw(NoSuchObjectError);
        int getLaneIndex(std::string vid) throw(NoSuchObjectError);
        std::vector<std::string> getRouteEdges(std::string vid) throw(NoSuchObjectError);

        std::string getVehicleType(std::string vid) throw(NoSuchObjectError);


        //void stopVehicle(tcpip::Storage& input) throw(NoSuchObjectError, NoSuchObjectError, std::runtime_error);
        void changeLane(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error);
        void slowDown(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error);
        void changeColour(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error);
        void setSpeed(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error);
        void setMaxSpeed(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error);
        void setRoute(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error);

        /* prevent alternative instantiation */
        VehicleManager(VehicleManager const&) = delete;
        void operator=(VehicleManager const&) = delete;

        void handleDelayedTriggers();

        VEHICLE* findVehicle(int vid) throw(NoSuchObjectError);
        VEHICLE* findVehicle(std::string vid) throw(NoSuchObjectError);

        //types
        void packVhcTypesVariable(tcpip::Storage& input, tcpip::Storage& output) throw(std::runtime_error, NotImplementedError);
        void getVhcTypesVariable(int type_id, uint8_t varID, tcpip::Storage& output) throw(std::runtime_error, NotImplementedError);

        int rerouteVehicle(VEHICLE* vhc, LINK* lnk);
        void routeReEval(VEHICLE* vhc);

        bool speedControlOverride(VEHICLE* vhc, float& speed);

    private:

        static VehicleManager* instance;

        VehicleManager();

        ~VehicleManager()
        {
        }

        std::unordered_map<VEHICLE*, LaneSetTrigger*> lane_set_triggers;
        std::unordered_map<VEHICLE*, BaseSpeedController*> speed_controllers;

        std::unordered_map<VEHICLE*, std::unordered_map<LINK*, int>*> vhc_routes;

        std::unordered_map<int, VEHICLE*> vehicles_in_sim;
        std::vector<VEHICLE*> departed_vehicles;
        std::vector<VEHICLE*> arrived_vehicles;

        // vehicle types
        std::unordered_map<std::string, int> types_index_map;

        // nodes and links
        static int findExit(NODE* node, LINK* exit_link) throw(NoSuchObjectError);
    };
}
