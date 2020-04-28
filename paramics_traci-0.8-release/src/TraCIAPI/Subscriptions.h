#pragma once
#include <vector>
#include "storage.h"
#include "Constants.h"
#include "VehicleManager.h"
#include "Exceptions.h"
#include "Simulation.h"

namespace traci_api
{
    class VariableSubscription
    {
    public:
        static const uint8_t RES_SUB_INDVAR = 0xe0;
        static const uint8_t RES_SUB_MULTVAR = 0xe1;
        static const uint8_t RES_SUB_TLIGHTVAR = 0xe2;
        static const uint8_t RES_SUB_LANEVAR = 0xe3;
        static const uint8_t RES_SUB_VHCVAR = 0xe4;
        static const uint8_t RES_SUB_VHCTYPEVAR = 0xe5;
        static const uint8_t RES_SUB_RTEVAR = 0xe6;
        static const uint8_t RES_SUB_POIVAR = 0xe7;
        static const uint8_t RES_SUB_POLVAR = 0xe8;
        static const uint8_t RES_SUB_JUNCTVAR = 0xe9;
        static const uint8_t RES_SUB_EDGEVAR = 0xea;
        static const uint8_t RES_SUB_SIMVAR = 0xeb;


        static const uint8_t STATUS_OK = 0x00;
        static const uint8_t STATUS_TIMESTEPNOTREACHED = 0x01;
        static const uint8_t STATUS_EXPIRED = 0x02;
        static const uint8_t STATUS_ERROR = 0xff;
        static const uint8_t STATUS_OBJNOTFOUND = 0xee;

        //update statuses
        static const uint8_t STATUS_NOUPD = 0xa0;
        static const uint8_t STATUS_UNSUB = 0xa1;


        VariableSubscription(std::string obj_id, int begin_time, int end_time, std::vector<uint8_t> vars) :
            objID(obj_id),
            beginTime(begin_time),
            endTime(end_time),
            vars(vars),
            sub_type(-1)
        {
        }

        int checkTime() const;

        virtual ~VariableSubscription()
        {
        };
        uint8_t handleSubscription(tcpip::Storage& output, bool validate, std::string& errors);
        virtual void getObjectVariable(uint8_t var_id, tcpip::Storage& result) = 0;
        uint8_t getSubType() const { return sub_type; }
        virtual uint8_t getResponseCode() const = 0;

        uint8_t updateSubscription(uint8_t sub_type, std::string obj_id, int begin_time, int end_time, std::vector<uint8_t> vars, tcpip::Storage& result, std::string& errors);

    protected:
        std::string objID;
        int beginTime;
        int endTime;
        std::vector<uint8_t> vars;
        int sub_type;
    };

    class VehicleVariableSubscription : public VariableSubscription
    {
    public:

        VehicleVariableSubscription(std::string vhc_id, int begin_time, int end_time, std::vector<uint8_t> vars)
            : VariableSubscription(vhc_id, begin_time, end_time, vars)
        {
            sub_type = CMD_SUB_VHCVAR;
        }

        ~VehicleVariableSubscription() override
        {
        }

        void getObjectVariable(uint8_t var_id, tcpip::Storage& result) override;
        uint8_t getResponseCode() const override;
    };

    class SimulationVariableSubscription : public VariableSubscription
    {
    public:
        SimulationVariableSubscription(std::string object_id, int begin_time, int end_time, const std::vector<uint8_t>& vars)
            : VariableSubscription(object_id, begin_time, end_time, vars)
        {
            sub_type = CMD_SUB_SIMVAR;
        }

        ~SimulationVariableSubscription() override
        {
        }

        void getObjectVariable(uint8_t var_id, tcpip::Storage& result) override;
        uint8_t getResponseCode() const override;
    };
}
