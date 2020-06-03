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

		// Flujo de vehiculos
		out_of_junction_count.resize(4, 0);
		total_flux = 0.0;
		flux.resize(4, 0);

		// Tiempo promedio en interseccion
		total_mean_time = 0.0;
		mean_time.resize(4, 0);
		aux_mean_time.resize(4, 0);

		// Desviacion estandar de tiempo en interseccion
		standard_deviation.resize(4, 0);
		aux_standard_deviation.resize(4, 0);
		total_standard_deviation = 0.0;

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
	recordScalar("TotalFlux", total_flux);
	recordScalar("Direction-South-Flux", flux[0]);
	recordScalar("Direction-West-Flux", flux[1]);
	recordScalar("Direction-North-Flux", flux[2]);
	recordScalar("Direction-East-Flux", flux[3]);

	// Registro de tiempo en interseccion promedio total y por direccion
	recordScalar("TotalMeanTime", total_mean_time);
	recordScalar("Direction-South-MeanTime", mean_time[0]);
	recordScalar("Direction-West-MeanTime", mean_time[1]);
	recordScalar("Direction-North-MeanTime", mean_time[2]);
	recordScalar("Direction-East-MeanTime", mean_time[3]);

	// Registro de desviacion estandar de tiempo promedio
	recordScalar("TotalStandardDeviation", total_standard_deviation);
	recordScalar("Direction-South-StDev", standard_deviation[0]);
	recordScalar("Direction-West-StDev", standard_deviation[1]);
	recordScalar("Direction-North-StDev", standard_deviation[2]);
	recordScalar("Direction-East-StDev", standard_deviation[3]);

    TraCIScenarioManagerLaunchd::finish();
}


void ExtTraCIScenarioManagerLaunchd::handleSelfMsg(cMessage *msg)
{
	// Calculo de flujo y tiempo en interseccion promedio
	calculateMetrics();

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
	aux_mean_time[direction] += timeInJunction;
	aux_standard_deviation[direction] += (timeInJunction * timeInJunction);
}


/**
 * Calculo de metricas: flujo, tiempo promedio y su desviacion estandar
 */
void ExtTraCIScenarioManagerLaunchd::calculateMetrics()
{
	int total_out_of_junction_count = 0;
	double sum_total_time_in_junction = 0;
	double sum_total_standard_deviation = 0.0;

	for(int i=0; i<4; i++)
	{
		// Calculando flujo por direccion de entrada
		int N = out_of_junction_count[i];
		double sim_time = simTime().dbl();

		EV << "direction: " << i << "\n";
		EV << "sim time: " << sim_time << "\n";
		EV << "number of cars: " << N << "\n";

		if(sim_time > 0)
			flux[i] = ((N * 60.0) / sim_time);
		else
			flux[i] = -1.0;

		EV << "car flux " << i << ": " << flux[i] << " [car/minute]\n";

		total_out_of_junction_count += N;


		// Calculando tiempo promedio por direccion de entrada
		double sum_time = aux_mean_time[i];

		if(N > 0)
			mean_time[i] = sum_time / (1.0 * N);
		else
			mean_time[i] = -1.0;
		
		EV << "mean time " << i << ": " << mean_time[i] << "[s]\n";

		sum_total_time_in_junction += sum_time;


		// Calculando desviacion estandar del tiempo promedio, por direccion de entrada
		double sum_st_dev = aux_standard_deviation[i];

		if(N > 0)
			standard_deviation[i] = sqrt((N * sum_st_dev - (sum_time * sum_time)) / (N * N));
		else
			standard_deviation[i] = -1.0;

		EV << "standard deviation " << i << ": " << standard_deviation[i] << "\n";

		sum_total_standard_deviation += sum_st_dev;

	}

	int N = total_out_of_junction_count;
	double sum_time = sum_total_time_in_junction;
	double sum_st_dev = sum_total_standard_deviation;
	double sim_time = simTime().dbl();

	// Flujo total
	total_flux = ((N * 60.0) / sim_time);

	// Tiempo promedio total
	total_mean_time = sum_time / N;

	// Desviacion estandard total
	
	total_standard_deviation = sqrt((N * sum_st_dev - (sum_time * sum_time)) / (N * N));

}