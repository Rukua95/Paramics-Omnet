//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include <veins/modules/application/pveins_providencia/ExampleProvidencia.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/application/pveins/json.hpp>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <cstdlib>
#include <algorithm>

using json = nlohmann::json;

Define_Module(ExampleProvidencia);

//std::mutex ExampleProvidencia::lock;
bool ExampleProvidencia::accident_car_set = false;

void ExampleProvidencia::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);

    switch (stage)
    {
    case 0:
        // init
        EV << "ExampleProvidencia::initialize stage 0\n";
        mobility = Veins::TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        accident_car = false;
        stopped = false;
        rerouted = false;
        recolored = false;
        break;
    case 1:
    {
        // schedule selfbeacons
        EV << "ExampleProvidencia::initialize stage 1\n";
        SimTime beginTime = SimTime(uniform(0.001, 1.0));
        selfbeacon = new cMessage();
        ping_interval = SimTime(0.1);

        scheduleAt(simTime() + ping_interval + beginTime, selfbeacon);

        accident_road = par("AccidentRoad").stringValue();
        accident_distance = par("AccidentDistance").doubleValue();
        alternative_road = par("AlternativeRoad").stringValue();
        const char* dests = par("AffectedDestinations").stringValue();
        warning_interval = par("WarningInterval").longValue();

        destinations = cStringTokenizer(dests).asIntVector();

        const char* roads_s = par("AffectedRoads").stringValue();
        roads = cStringTokenizer(roads_s).asVector();

        sent_warnings = 0;
        rcvd_warnings = 0;
    }
        break;
    default:
        break;
    }
}
void ExampleProvidencia::finish()
{
    BaseWaveApplLayer::finish();

    ExtTraCIScenarioManagerLaunchd* sceman = dynamic_cast<ExtTraCIScenarioManagerLaunchd*>(mobility->getManager());

    bool arrived;
    if (sceman->shuttingDown())
        arrived = false;
    else arrived = true;

    recordScalar("ArrivedAtDest", arrived);
    recordScalar("SentWarnings", sent_warnings);
    recordScalar("RcvdWarnings", rcvd_warnings);
}
void ExampleProvidencia::handleSelfMsg(cMessage *msg){

    EV << "ExampleProvidencia::handleSelfMsg car: " << mobility->getId() << "\n";
    if (!accident_car)
    {
        if(accident_car_set)
        {
            cancelAndDelete(selfbeacon);
            return;
        }
        else if (!accident_car_set && traciVehicle->getRoadId() == accident_road)
        {
            accident_car = true;
            accident_car_set = true;
            warning_msg = prepareWSM("data", beaconLengthBits, type_CCH, beaconPriority, -1, -1);
            warning_msg->setWsmData("WARNING");
        }
        else if (!accident_car_set)
        {
            scheduleAt(simTime() + ping_interval, selfbeacon);
            return;
        }
    }

    if(accident_car)
    {
        if(!stopped && traciVehicle->getRoadId() == accident_road && traciVehicle->getLanePosition() >= accident_distance)
        {
            // stop
            traciVehicle->setColor(Veins::TraCIColor::fromTkColor("orange"));
            traciVehicle->setSpeed(0.0);
            stopped = true;

            ping_interval = SimTime(warning_interval, SIMTIME_S);
            scheduleAt(simTime() + ping_interval, selfbeacon);
        }
        else if (stopped)
        {
            // send warning message
            traciVehicle->setColor(Veins::TraCIColor::fromTkColor("red"));
            sendWSM((WaveShortMessage*)warning_msg->dup());
            sent_warnings++;
            scheduleAt(simTime() + ping_interval, selfbeacon);
        }
        else
            scheduleAt(simTime() + ping_interval, selfbeacon);
    }

}

void ExampleProvidencia::changeRoute()
{
    // first, check if we're on a potentially affected road
    std::string current_road = traciVehicle->getRoadId();
    if (std::find(roads.begin(), roads.end(), current_road) == roads.end())
        return;

    // check if destination is affected
    std::list<std::string> route = traciVehicle->getPlannedRoadIds();
    int dest = atoi(&route.back()[0u]);

    if(std::find(destinations.begin(), destinations.end(), dest) == destinations.end())
        return;

    // set new route
    std::list<std::string> new_route = { current_road, alternative_road };
    if (traciVehicle->changeVehicleRoute(new_route))
    {
        traciVehicle->setColor(Veins::TraCIColor::fromTkColor("green"));
        rerouted = true;
        recolored = true;
    }
}

void ExampleProvidencia::onData(WaveShortMessage *wsm)
{
    if (!accident_car)
    {
        if(!rerouted)
            changeRoute();

        if(!recolored){
            traciVehicle->setColor(Veins::TraCIColor::fromTkColor("yellow"));
            recolored = true;
        }

        rcvd_warnings++;
    }
    delete wsm;
}
void ExampleProvidencia::onBeacon(WaveShortMessage *wsm){

}
