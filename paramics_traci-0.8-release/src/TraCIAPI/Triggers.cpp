#include "Triggers.h"

traci_api::LaneSetTrigger::LaneSetTrigger(VEHICLE* vhc, int target_lane, int duration) : target_lane(target_lane), vehicle(vhc)
{
    end_time = Simulation::getInstance()->getCurrentTimeMilliseconds() + duration;
    //orig_hlane = qpg_VHC_laneHigh(vhc);
    //orig_llane = qpg_VHC_laneLow(vhc);
}


void traci_api::LaneSetTrigger::handleTrigger()
{
    int t_lane = target_lane;

    int maxlanes = qpg_LNK_lanes(qpg_VHC_link(vehicle));
    if (t_lane > maxlanes)
        t_lane = maxlanes;
    else if (t_lane < 1)
        t_lane = 1;

    // additionally, set lane range for better lane change behavior
    //qps_VHC_laneRange(vehicle, t_lane, t_lane);

    int current_lane = qpg_VHC_lane(vehicle);
    if (current_lane > t_lane)
        qps_VHC_laneChange(vehicle, -1);
    else if (current_lane < t_lane)
        qps_VHC_laneChange(vehicle, +1);
    else
        qps_VHC_laneChange(vehicle, 0);
}

bool traci_api::LaneSetTrigger::repeat()
{
    if (Simulation::getInstance()->getCurrentTimeMilliseconds() >= end_time)
    {
        // reset lane range
        //qps_VHC_laneRange(vehicle, orig_llane, orig_hlane);
        return false;
    }
    else return true;
}

float traci_api::HoldSpeedController::nextTimeStep()
{
    float current_speed = qpg_VHC_speed(vhc);
    float diff = target_speed - current_speed;
    if (abs(diff) < NUMERICAL_EPS)
    {
        if (target_speed < NUMERICAL_EPS && !qpg_VHC_stopped(vhc))
            qps_VHC_stopped(vhc, PTRUE);
        return current_speed;
    }

    /* find acceleration/deceleration needed to reach speed asap */
    float accel;
    if (diff < 0)
    {
        /* decelerate */
        accel = max(qpg_VTP_deceleration(qpg_VHC_type(vhc)), diff);
    }
    else
    {
        /* accelerate */
        accel = min(qpg_VTP_acceleration(qpg_VHC_type(vhc)), diff);
    }

    return current_speed + (qpg_CFG_timeStep()*accel);
}

traci_api::LinearSpeedChangeController::LinearSpeedChangeController(VEHICLE* vhc, float target_speed, int duration) : vhc(vhc), duration(0), done(false)
{
    /* 
     * calculate acceleration needed for each timestep. if duration is too short, i.e.
     * it causes the needed acceleration to be greater than the maximum allowed, we'll use
     * the maximum for the duration, but we'll never reach the desired speed.
     */

    float current_speed = qpg_VHC_speed(vhc);
    float diff = target_speed - current_speed;
    // first, check if we actually need to change the speed
    // this will do nothing if we don't
    if (abs(diff) < NUMERICAL_EPS)
    {
        done = true;
        acceleration = 0;
        return;
    }

    float timestep_sz = qpg_CFG_timeStep();
    float duration_s = duration / 1000.0f;
    int d_factor = round(duration_s / timestep_sz);
    this->duration = d_factor * (timestep_sz * 1000);

    acceleration = diff / (duration / 1000.0f); // acceleration (m/s2)
    if (diff < 0)
    {
        /* decelerate */
        acceleration = max(qpg_VTP_deceleration(qpg_VHC_type(vhc)), acceleration);
    }
    else
    {
        /* accelerate */
        acceleration = min(qpg_VTP_acceleration(qpg_VHC_type(vhc)), acceleration);
    }
}

float traci_api::LinearSpeedChangeController::nextTimeStep()
{
    float timestep_sz = qpg_CFG_timeStep();
    duration -= timestep_sz * 1000;
    if (duration <= 0)
        done = true;
    
    return qpg_VHC_speed(vhc) + (timestep_sz * acceleration);
}
