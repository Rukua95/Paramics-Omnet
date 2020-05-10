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

#ifndef SRC_VEINS_MODULES_APPLICATION_PVEINS_ExampleProvidencia_H_
#define SRC_VEINS_MODULES_APPLICATION_PVEINS_ExampleProvidencia_H_

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include <mutex>

class ExampleProvidencia : public BaseWaveApplLayer
{
protected:
    Veins::TraCIMobility* mobility;
    Veins::TraCICommandInterface *traci;
    Veins::TraCICommandInterface::Vehicle *traciVehicle;

    WaveShortMessage* warning_msg;

    cMessage *selfbeacon;
    SimTime ping_interval;
    int warning_interval;

    // stats
    int sent_warnings;
    int rcvd_warnings;

    std::string accident_road;
    std::string alternative_road;
    double accident_distance;
    bool accident_car;
    bool stopped;
    bool rerouted;
    bool recolored;
    std::vector<int> destinations;
    std::vector<std::string> roads;

    void initialize(int stage);
    void finish();
    void handleSelfMsg(cMessage *msg);
    void onData(WaveShortMessage *wsm);
    void onBeacon(WaveShortMessage *wsm);

    void changeRoute();

public:
    static bool accident_car_set;
};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_ExampleProvidencia_H_ */
