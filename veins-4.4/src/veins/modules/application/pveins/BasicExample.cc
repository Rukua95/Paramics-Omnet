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

#include <veins/modules/application/pveins/BasicExample.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/application/pveins/json.hpp>

using json = nlohmann::json;

Define_Module(BasicExample);

void BasicExample::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);

    switch (stage)
    {
    case 0:
        // init

        mobility = Veins::TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        accident_car = false;
        stopped = false;
        change_lane = false;
        slow = false;
        msg_payload = "";
        break;
    case 1:
        // check if I'm "accident car"
        if(mobility->getExternalId() == par("AccidentCar").stringValue())
        {
            accident_car = true;
            traciVehicle->setColor(Veins::TraCIColor::fromTkColor("red"));

            // schedule selfbeacons
            SimTime beginTime = SimTime(uniform(0.001, 1.0));
            selfbeacon = new cMessage();
            ping_interval = SimTime(1);
            scheduleAt(simTime() + ping_interval + beginTime, selfbeacon);

            accident_road = par("AccidentRoad").stringValue();
            accident_distance = par("AccidentDistance").doubleValue();
        }
        break;
    default:
        break;
    }
}
void BasicExample::finish()
{
    BaseWaveApplLayer::finish();
}
void BasicExample::handleSelfMsg(cMessage *msg){
    if (accident_car)
    {
        if (slow && 2 == traciVehicle->getLaneIndex())
        {
            traciVehicle->setSpeed(0.0);
            slow = false;
            stopped = true;
        }

        if(!stopped)
        {
            std::string current_road = traciVehicle->getRoadId();
            double distance = traciVehicle->getLanePosition();

            // are we at accident location?
            if (!slow && current_road == accident_road && distance >= accident_distance)
            {
                // yes we are, stop
                traciVehicle->changeLane(2, 500);
                traciVehicle->setSpeed(2.0);
                slow = true;
            }
        }

        if (stopped)
        {
            //send warning
            if (msg_payload == "")
            {
                std::string current_road = traciVehicle->getRoadId();
                double distance = traciVehicle->getLanePosition();

                std::string id = mobility->getExternalId();
                int lane = traciVehicle->getLaneIndex();

                json data_j = {
                        {"id", id},
                        {"road", current_road},
                        {"lane", lane},
                        {"position", distance}
                };

                msg_payload = data_j.dump();
            }

            WaveShortMessage* warning = prepareWSM("data", beaconLengthBits, type_CCH, beaconPriority, -1, -1);
            warning->setWsmData(msg_payload.c_str());
            sendWSM(warning);
        }
        scheduleAt(simTime() + ping_interval, selfbeacon);
    }
}
void BasicExample::onData(WaveShortMessage *wsm)
{
    if (!accident_car)
    {
        json data_j = json::parse(std::string(wsm->getWsmData()));
        //std::cerr < data_j << std::endl;
        std::string accident_road = data_j["road"];
        double accident_distance = data_j["position"];

        std::string current_road = traciVehicle->getRoadId();
        double current_position = traciVehicle->getLanePosition();

        double delta = accident_distance - current_position;

        if (current_road == accident_road && !slow)
        {
            traciVehicle->setSpeed(5.0);
            slow = true;
        }
        else if (current_road != accident_road && slow)
        {
            traciVehicle->setSpeed(-1.0);
            traciVehicle->changeLane(1, -1);
            slow = false;
            change_lane = false;
        }
        else if (current_road == accident_road && delta < 100 && !change_lane)
        {
            traciVehicle->changeLane(1, 1000000);
            traciVehicle->setSpeed(8.0);
            change_lane = true;
        }
    }
}
void BasicExample::onBeacon(WaveShortMessage *wsm){

}
