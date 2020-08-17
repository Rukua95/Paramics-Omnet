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
	EV << "Inicializacion manager\n";

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

		first_car_wait_time.resize(4, 0);
		registered_stuck = false;

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
	/*
	recordScalar("TotalFlux", total_flux);
	recordScalar("Direction-South-Flux", flux[0]);
	recordScalar("Direction-West-Flux", flux[1]);
	recordScalar("Direction-North-Flux", flux[2]);
	recordScalar("Direction-East-Flux", flux[3]);
	*/

	// Registro de tiempo en interseccion promedio total y por direccion
	/*
	recordScalar("TotalMeanTime", total_mean_time);
	recordScalar("Direction-South-MeanTime", mean_time[0]);
	recordScalar("Direction-West-MeanTime", mean_time[1]);
	recordScalar("Direction-North-MeanTime", mean_time[2]);
	recordScalar("Direction-East-MeanTime", mean_time[3]);
	*/

	// Registro de desviacion estandar de tiempo promedio
	/*
	recordScalar("TotalStandardDeviation", total_standard_deviation);
	recordScalar("Direction-South-StDev", standard_deviation[0]);
	recordScalar("Direction-West-StDev", standard_deviation[1]);
	recordScalar("Direction-North-StDev", standard_deviation[2]);
	recordScalar("Direction-East-StDev", standard_deviation[3]);
	*/

	// Registro de colisiones
	recordScalar("TotalCollision", collision_register.size());
	for(auto it=collision_register.begin(); it != collision_register.end(); it++)
	{
		int id1 = it->first.first, id2 = it->first.second;
		recordScalar("Car1", id1);
		recordScalar("Car2", id2);
	}

    TraCIScenarioManagerLaunchd::finish();
}


void ExtTraCIScenarioManagerLaunchd::handleSelfMsg(cMessage *msg)
{
	// Calculo de flujo y tiempo en interseccion promedio
	//calculateMetrics();

	// Determinar colision de vehiculos dentro de interseccion
	detectColision();

	bool stuck = true;
	for(double time : first_car_wait_time)
		if(time < 30.0)
			stuck = false;

	if(stuck && !registered_stuck)
	{
		recordScalar("DeadLock", 1);
		recordScalar("DeadLockTime", simTime().dbl());
		registered_stuck = true;
	}

    if (msg == selfping)
    {
        // Registra cantidad de vehiculos en simulacion
        VehiclesInSim.record(this->activeVehicleCount);
        realtimestamps.record(difftime(std::time(NULL), start_time));
        scheduleAt(simTime() + ping_interval, msg);
    }
    else 
		TraCIScenarioManagerLaunchd::handleSelfMsg(msg);
}


/**
 * Funcion que registra salida de vehiculos de la interseccion.
 */
void ExtTraCIScenarioManagerLaunchd::carOutOfJunction(int carId, int direction, double timeInJunction)
{
	// TODO: Eliminar vehiculo de la lista de vehiculos dentro de interseccion
	car_position_intersection.erase(carId);

	out_of_junction_count[direction]++;
	aux_mean_time[direction] += timeInJunction;
	aux_standard_deviation[direction] += (timeInJunction * timeInJunction);
}


/**
 * Funcion que registra entrada de vehiculos a la interseccion.
 */
void ExtTraCIScenarioManagerLaunchd::carInOfJunction(int carId, Coord coord_position, Coord coord_speed)
{
	car_position_intersection[carId] = std::pair<Coord, Coord>(coord_position, coord_speed);
	//detectColision();
}


/**
 * Funcion que detecta colision de vehiculo con otro, dada la info que envie este ultimo
 */
void ExtTraCIScenarioManagerLaunchd::detectColision()
{
	EV << ">>> Deteccion de colisiones <<<\n";
	for(auto it1 = car_position_intersection.begin(); it1 != car_position_intersection.end(); it1++)
	{
		for(auto it2 = it1; it2 != car_position_intersection.end(); it2++)
		{
			int id1 = it1->first, id2 = it2->first;
			if(id1 == id2)
				continue;

			// Posicion y velocidad de los vehiculos a comparar
			Coord p1 = it1->second.first, p2 = it2->second.first;
			Coord v1 = it1->second.second, v2 = it2->second.second;

			// Angulo a partir de vector de velocidad
			double theta1 = atan2(v1.y, v1.x), theta2 = atan2(v2.y, v2.x);

			// Posicion de vertices de vehiculo, dado 
			std::vector<Coord> lim1 = {Coord(1.0, 0.8), Coord(1.0, -0.8), Coord(-1.0, -0.8), Coord(-1.0, 0.8)};
			getCarPoint(lim1, theta1);

			std::vector<Coord> lim2 = {Coord(1.0, 0.8), Coord(1.0, -0.8), Coord(-1.0, -0.8), Coord(-1.0, 0.8)};
			getCarPoint(lim2, theta2);

			EV << ">>> p1: " << p1 << "   v1: " << v1 << "   theta1: " << theta1 <<"\n";
			EV << ">>> Car points:\n    ";
			for(int i=0; i<4; i++)
			{
				lim1[i] += p1;
				EV << lim1[i] << " ";
			}
			EV << "\n";

			EV << ">>> p2: " << p2 << "   v2: " << v2 << "   theta2: " << theta2 <<"\n";
			EV << ">>> Car sender points:\n    ";
			for(int i=0; i<4; i++)
			{
				lim2[i] += p2;
				EV << lim2[i] << " ";
			}
			EV << "\n";

			EV << ">>> Comparando vehiculos\n";
			bool car_intersect = false;
			for(int i=0; i<4; i++)
			{
				for(int j=0; j<4; j++)
				{
					Coord vec = lim1[(i + 1) % 4] - lim1[i];
					Coord q1 = lim2[j] - lim1[i];
					Coord q2 = lim2[(j + 1) % 4] - lim1[i];

					double val1 = q1.x * vec.y - q1.y * vec.x;
					double val2 = q2.x * vec.y - q2.y * vec.x;

					bool ver1 = (val1*val2 < 0);


					vec = lim2[(j + 1) % 4] - lim2[j];
					q1 = lim1[i] - lim2[j];
					q2 = lim1[(i + 1) % 4] - lim2[j];

					val1 = q1.x * vec.y - q1.y * vec.x;
					val2 = q2.x * vec.y - q2.y * vec.x;

					bool ver2 = (val1*val2 < 0);

					if(ver1 && ver2)
					{
						car_intersect = true;
						break;
					}
				}
			}

			if(car_intersect)
			{
				EV << ">>> Colision registrada\n";
				std::pair<int, int> p(id1, id2);
				if(id2 < id1) p = std::pair<int, int>(id2, id1);

				if(collision_register.count(p) == 0)
					collision_register[p] = simTime().dbl();

			}
		}
	}
}


/**
 * Funcion que obtiene las coordenadas de los vertices que representan un vehiculo
 */
void ExtTraCIScenarioManagerLaunchd::getCarPoint(std::vector<Coord> &lim, double theta)
{
	EV << ">>> cos(theta): " << cos(theta) << "   sin(theta): " << sin(theta) << "\n";
	for(int i=0; i<4; ++i) 
	{
		double aux_x = lim[i].x;
		double aux_y = lim[i].y;

		lim[i].x = aux_x * cos(theta) - aux_y * sin(theta);
		lim[i].y = aux_x * sin(theta) + aux_y * cos(theta);
	}
}


/**
 * Fucino que calcula: flujo vehicular y tiempo promedio en interseccion, y su desviacion estandar
 */
void ExtTraCIScenarioManagerLaunchd::calculateMetrics()
{
	EV << ">>> Calculando metricas <<<\n";
	int total_out_of_junction_count = 0;
	double sum_total_time_in_junction = 0;
	double sum_total_standard_deviation = 0.0;

	for(int i=0; i<4; i++)
	{
		// Calculando flujo por direccion de entrada
		int N = out_of_junction_count[i];
		double sim_time = simTime().dbl();

		EV << "direccion: " << i << "\n";
		EV << "tiempo de simulacion: " << sim_time << "\n";
		EV << "numero de vehiculos: " << N << "\n";

		if(sim_time > 0)
			flux[i] = ((N * 60.0) / sim_time);
		else
			flux[i] = -1.0;

		EV << "flujo de vehiculos hacia " << i << ": " << flux[i] << " [vehiculo/minuto]\n";

		total_out_of_junction_count += N;


		// Calculando tiempo promedio por direccion de entrada
		double sum_time = aux_mean_time[i];

		if(N > 0)
			mean_time[i] = sum_time / (1.0 * N);
		else
			mean_time[i] = -1.0;
		
		EV << "tiempo promedio " << i << ": " << mean_time[i] << "[s]\n";

		sum_total_time_in_junction += sum_time;


		// Calculando desviacion estandar del tiempo promedio, por direccion de entrada
		double sum_st_dev = aux_standard_deviation[i];

		if(N > 0)
			standard_deviation[i] = sqrt((N * sum_st_dev - (sum_time * sum_time)) / (N * N));
		else
			standard_deviation[i] = -1.0;

		EV << "desviacion estandar " << i << ": " << standard_deviation[i] << "\n";

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


/**
 * Funcion que registra tiempo en detencion que lleva el vehiculo primero en la pista
 */
void ExtTraCIScenarioManagerLaunchd::saveWaitingTime(int direction, double time_in_wait)
{
	first_car_wait_time[direction] = time_in_wait;
}