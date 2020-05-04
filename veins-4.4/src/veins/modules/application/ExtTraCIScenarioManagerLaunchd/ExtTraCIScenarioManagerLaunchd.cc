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

#include "ExtTraCIScenarioManagerLaunchd.h"

Define_Module(ExtTraCIScenarioManagerLaunchd);

bool ExtTraCIScenarioManagerLaunchd::shuttingDown()
{
    return (shut_down);
}

void ExtTraCIScenarioManagerLaunchd::initialize(int stage)
{
    TraCIScenarioManagerLaunchd::initialize(stage);
	EV << "ExtTraCIScenarioManagerLaunchd::initialize\n";

    if (stage == 1)
    {
        // Get Simulation start time (in realtime!) and record it
        std::time(&start_time);
        // setup pings to record number of vehicles in sim
        selfping = new cMessage();
        ping_interval = SimTime(60, SIMTIME_S);
        scheduleAt(connectAt + ping_interval, selfping);

        VehiclesInSim.setName("vehiclesinsim");
        VehiclesInSim.setType(cOutVector::TYPE_INT);

        realtimestamps.setName("realtime");
        realtimestamps.setType(cOutVector::TYPE_INT);
        realtimestamps.setUnit("s");

        VehiclesInSim.record(this->activeVehicleCount);
        realtimestamps.record(0);

		// Flux
		out_of_junction_count.resize(4, 0);
		flux = 0.0;
		direction_flux.resize(4, 0);

		// Mean time in intersection
		mean_time.resize(4, 0);
		total_mean_time = 0.0;
		direction_total_mean_time.resize(4, 0);


    }
}

void ExtTraCIScenarioManagerLaunchd::finish()
{
    shut_down = true;

    cancelAndDelete(selfping);

    // Get Simulation end time (in realtime!) and record it
    std::time(&end_time);
    recordScalar("EndTime", end_time);
    recordScalar("StartTime", start_time);

    // record time difference
    recordScalar("TotalDuration", difftime(end_time, start_time));
	
	recordScalar("TotalFlux", flux);
	recordScalar("Direction 0 Flux", direction_flux[0]);
	recordScalar("Direction 1 Flux", direction_flux[1]);
	recordScalar("Direction 2 Flux", direction_flux[2]);
	recordScalar("Direction 3 Flux", direction_flux[3]);

	recordScalar("TotalMeanTime", total_mean_time);
	recordScalar("Direction 0 Mean Time", direction_total_mean_time[0]);
	recordScalar("Direction 1 Mean Time", direction_total_mean_time[1]);
	recordScalar("Direction 2 Mean Time", direction_total_mean_time[2]);
	recordScalar("Direction 3 Mean Time", direction_total_mean_time[3]);

    TraCIScenarioManagerLaunchd::finish();
}

void ExtTraCIScenarioManagerLaunchd::handleSelfMsg(cMessage *msg)
{
	EV << "ExtTraCIScenarioManagerLaunchd::handleSelfMsg <--\n";
	carFlux();
	meanTime();

    if (msg == selfping)
    {
        // record number of vehicles in simulation
        VehiclesInSim.record(this->activeVehicleCount);
        realtimestamps.record(difftime(std::time(NULL), start_time));
        scheduleAt(simTime() + ping_interval, msg);
    }
    else TraCIScenarioManagerLaunchd::handleSelfMsg(msg);
}


void ExtTraCIScenarioManagerLaunchd::carOutOfJunction(int carId, int direction, double timeInJunction)
{
	out_of_junction_count[direction]++;
	mean_time[direction] += timeInJunction;
}


void ExtTraCIScenarioManagerLaunchd::carFlux()
{
	int total = 0;
	for(int i=0; i<4; i++)
	{
		int car_count = out_of_junction_count[i];
		double sim_time = simTime().dbl();
		EV << "sim_time: " << sim_time << "\n";
		EV << "direction: " << i << "\n";
		EV << "car_count: " << car_count << "\n";
		EV << "sim time: " << sim_time << "\n";

		if(sim_time > 0)
			direction_flux[i] = ((car_count * 60.0) / sim_time);
		else
			direction_flux[i] = -1.0;

		EV << "car flux: " << direction_flux[i] << " [car/minute]\n";

		total += car_count;
	}

	double sim_time = simTime().dbl();
	EV << "total car_count: " << total << "\n";
	EV << "sim time: " << sim_time << "\n";

	flux = ((total * 60.0) / sim_time);
	EV << "car flux: " << flux << " [car/minute]\n";

}


void ExtTraCIScenarioManagerLaunchd::meanTime()
{
	double sum_total = 0;
	int cant_total = 0;
	for(int i=0; i<4; i++)
	{
		int car_count = out_of_junction_count[i];
		double sum_time = mean_time[i];
		
		EV << "direction: " << i << "\n";
		EV << "car_count: " << car_count << "\n";

		if(car_count > 0)
			direction_total_mean_time[i] = sum_time / (1.0 * car_count);
		else
			direction_total_mean_time[i] = -1.0;
		

		EV << "mean time: " << sum_time / (1.0 * car_count) << "\n";

		sum_total += sum_time;
		cant_total += car_count;

	}

	total_mean_time = sum_total / (1.0 * cant_total);
	EV << "total_mean_time: " << total_mean_time << "\n";
	
}