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

#ifndef SRC_VEINS_MODULES_APPLICATION_PVEINS_PROVIDENCIA_EXTTRACISCENARIOMANAGERLAUNCHD_H_
#define SRC_VEINS_MODULES_APPLICATION_PVEINS_PROVIDENCIA_EXTTRACISCENARIOMANAGERLAUNCHD_H_

#include "veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.h"
#include <time.h>

class ExtTraCIScenarioManagerLaunchd: public Veins::TraCIScenarioManagerLaunchd {
public:
    ExtTraCIScenarioManagerLaunchd(): shut_down(false) {}
    virtual ~ExtTraCIScenarioManagerLaunchd(){}

    virtual void initialize(int stage);
    virtual void finish();
    virtual void handleSelfMsg(cMessage *msg);

    bool shuttingDown();


	void carOutOfJunction(int carId, int direction, double timeInJunction);
	void carFlux();
	void meanTime();


	

private:
    bool shut_down;

    std::time_t start_time;
    std::time_t end_time;
    cMessage* selfping;
    SimTime ping_interval;
    cOutVector VehiclesInSim;
    cOutVector realtimestamps;


	std::vector<int> out_of_junction_count;
    std::vector<double> direction_flux;
	double flux;


	std::vector<double> mean_time, direction_total_mean_time;
	double total_mean_time;
};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_PROVIDENCIA_EXTTRACISCENARIOMANAGERLAUNCHD_H_ */
