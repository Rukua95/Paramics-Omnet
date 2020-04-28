#pragma once
#include "programmer.h"

#include "Constants.h"
#include <cmath>
#include "Simulation.h"

namespace traci_api
{
    /* Triggered events (multiple causes: time, changing lanes, etc */
    class BaseTrigger
    {
    public:
        virtual ~BaseTrigger()
        {
        };
        virtual void handleTrigger() = 0;
        virtual bool repeat() = 0;
    };

    class LaneSetTrigger : public BaseTrigger
    {
    public:
        int target_lane;
        int end_time;
        VEHICLE* vehicle;

        LaneSetTrigger(VEHICLE* vhc, int target_lane, int duration);
        void handleTrigger() override;
        bool repeat() override;

        //private: 
        //    int orig_llane;
        //    int orig_hlane;
    };


    class BaseSpeedController
    {
    public:
        virtual ~BaseSpeedController()
        {
        }
        virtual float nextTimeStep() = 0;
        virtual bool repeat() = 0;
    };

    class HoldSpeedController : public BaseSpeedController
    {
    private:
        VEHICLE* vhc;
        float target_speed;

    public:
        HoldSpeedController(VEHICLE* vhc, float target_speed) : vhc(vhc), target_speed(target_speed){}
        ~HoldSpeedController() override {}

        float nextTimeStep() override;
        bool repeat() override { return true; }
    };

    class LinearSpeedChangeController : public BaseSpeedController
    {
    private:
        VEHICLE* vhc;
        int duration;
        bool done;

        float acceleration;

    public:
        LinearSpeedChangeController(VEHICLE* vhc, float target_speed, int duration);
        ~LinearSpeedChangeController() override {};

        float nextTimeStep() override;
        bool repeat() override { return !done; }
    };
}
