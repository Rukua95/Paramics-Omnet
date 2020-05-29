/*
Escenario utilizado para las simulaciones de protocolos.
Permite el calculo de flujo por segundo y tiempo promedio en interseccion.
*/

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
        // Obtencion de tiempo de inicio de simulacion (tiempo real) y su registro
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

    // Obtencion de tiempo de termino de simulacion (tiempo real) y su registro
    std::time(&end_time);
    recordScalar("EndTime", end_time);
    recordScalar("StartTime", start_time);

    // Registro de diferencia de tiempo de termino y de inicio
    recordScalar("TotalDuration", difftime(end_time, start_time));
	
	// Registro de flujo total y por direccion
	recordScalar("TotalFlux", flux);
	recordScalar("Direction 0 Flux", direction_flux[0]);
	recordScalar("Direction 1 Flux", direction_flux[1]);
	recordScalar("Direction 2 Flux", direction_flux[2]);
	recordScalar("Direction 3 Flux", direction_flux[3]);

	// Registro de tiempo en interseccion promedio total y por direccion
	recordScalar("TotalMeanTime", total_mean_time);
	recordScalar("Direction 0 Mean Time", direction_total_mean_time[0]);
	recordScalar("Direction 1 Mean Time", direction_total_mean_time[1]);
	recordScalar("Direction 2 Mean Time", direction_total_mean_time[2]);
	recordScalar("Direction 3 Mean Time", direction_total_mean_time[3]);

    TraCIScenarioManagerLaunchd::finish();
}


void ExtTraCIScenarioManagerLaunchd::handleSelfMsg(cMessage *msg)
{
	// Calculo de flujo y tiempo en interseccion promedio
	carFlux();
	meanTime();

    if (msg == selfping)
    {
        // Registra cantidad de vehiculos en simulacion
        VehiclesInSim.record(this->activeVehicleCount);
        realtimestamps.record(difftime(std::time(NULL), start_time));
        scheduleAt(simTime() + ping_interval, msg);
    }
    else TraCIScenarioManagerLaunchd::handleSelfMsg(msg);
}


/**
 * Funcion que registra cuando un vehiculo sale de la interseccion.
 */
void ExtTraCIScenarioManagerLaunchd::carOutOfJunction(int carId, int direction, double timeInJunction)
{
	out_of_junction_count[direction]++;
	mean_time[direction] += timeInJunction;
}


/**
 * Funcion que calcula el flujo vehicular total y direccional en interseccion, por segundo.
 */
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


/**
 * Funcion que calcula el tiempo en interseccion promedio y direccional.
 */
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