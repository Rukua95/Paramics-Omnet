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

#include <veins/modules/application/pveins/RouteChangeExample.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/application/pveins/json.hpp>
#include <omnetpp.h>

using json = nlohmann::json;

Define_Module(RouteChangeExample);

void RouteChangeExample::initialize(int stage)
{
    EV << "RouteChangeExample::initialize\n";
    BaseWaveApplLayer::initialize(stage);

    switch (stage)
    {
    case 0:
        // init

        mobility = Veins::TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        break;
    case 1:
    {
        triggered = false;
        // schedule selfbeacons
        SimTime beginTime = SimTime(uniform(0.001, 1.0));
        selfbeacon = new cMessage();
        ping_interval = SimTime(500, SIMTIME_MS);
        scheduleAt(simTime() + ping_interval + beginTime, selfbeacon);
    }
        break;
    default:
        break;
    }
}
void RouteChangeExample::finish()
{
    BaseWaveApplLayer::finish();
}
void RouteChangeExample::handleSelfMsg(cMessage *msg)
{
    EV << "RouteChangeExample::handleSelfMsg\n";
    // need to change route?
    std::string current_road = traciVehicle->getRoadId();
    if (!triggered && current_road == trigger_road)
    {
        std::cerr << "triggered yay" << std::endl;
        /* note: need to comment out
         * if (getRoadId().find(':') != std::string::npos) return false;
         *
         * in changeVehicleRoute(). This is for SUMO, which separates lanes
         * from road names with a colon and thus this checks that no lanes
         * are passed to the function. In Paramics, road names include colons,
         * so this check prevents the method from working.
         */
        if (!traciVehicle->changeVehicleRoute(route))
        {
            std::cerr << "wtf" << std::endl;
            throw -1;
        }
        triggered = true;
    }
    else if (!triggered)
    {
        scheduleAt(simTime() + ping_interval, selfbeacon);
    }
}
void RouteChangeExample::onData(WaveShortMessage *wsm)
{

}
void RouteChangeExample::onBeacon(WaveShortMessage *wsm){

}
