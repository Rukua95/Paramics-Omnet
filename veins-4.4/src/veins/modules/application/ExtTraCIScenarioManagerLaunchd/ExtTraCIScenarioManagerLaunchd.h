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
    void carInOfJunction(int carId, Coord coord_position, Coord coord_speed);
    void calculateMetrics();

    void saveWaitingTime(int direction, double time_in_wait);
    void detectColision();
    void getCarPoint(std::vector<Coord> &lim, double theta);

private:
    bool shut_down;
    bool registered_stuck;

    std::time_t start_time;
    std::time_t end_time;
    cMessage* selfping;
    SimTime ping_interval;
    cOutVector VehiclesInSim;
    cOutVector realtimestamps;

    // Variables para calculo de flujo y tiempo promedio
	std::vector<int> out_of_junction_count;
    std::vector<int> first_car_wait_time;

    std::map<int, std::pair<Coord, Coord> > car_position_intersection;
    std::map<std::pair<int, int>, double> collision_register;
	
	std::vector<double> flux, aux_mean_time, mean_time, aux_standard_deviation, standard_deviation;
    double total_flux, total_mean_time, total_standard_deviation;
};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_PROVIDENCIA_EXTTRACISCENARIOMANAGERLAUNCHD_H_ */
