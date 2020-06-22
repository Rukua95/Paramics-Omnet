/*
Base para protocolos implementados
*/

#include <veins/modules/application/Protocolos/Base_Protocolo/Base.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <cstdlib>
#include <algorithm>
#include <math.h>


void Base::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);

    switch (stage)
    {
    case 0:
		// Obtencion de modulos importantes para comunicacion con Paramics
		mobility = Veins::TraCIMobilityAccess().get(getParentModule());
		traci = mobility->getCommandInterface();
		traciVehicle = mobility->getVehicleCommandInterface();

		// Vector de posicion y velocidad de vehiculos
		position = Coord(0.0, 0.0);
		velocity = Coord(0.0, 0.0);

		// Dimensiones de vehiculos (ajustadas manualmente a la simulacion)
		car_lenght = 4.0;
		car_width = 1.6;

		// Estados de vehiculos
		stoping = false;
		stoped = false;

		outJunction = false;
		
		crossing = false; 
		
		inSharedDataZone = false;

		// Direccion inicial de vehiculo
		direction_junction = -1;

		// Distancia a interseccion
		distance_to_junction = -1.0;

		// Tiempo estimado de llegada a interseccion
		time_to_junction = -1.0;

		// Variables para calculo de aceleracion.
		last_vel = 0.0;
		acceleration = 0.0;
		got_last_accel = false;

		// Variables de maxima y minima aceleracion, y maxima velocidad (ajustadas manualmente a la simulacion)
		max_accel = 2.5;
		min_accel = -4.5;
		max_velocidad = 16;

		// Tablas de informacion de vehiculos, separadas segun direccion inicial de los vehiculos.
		carTable = std::vector<std::map<int, vehicleData> >(4, std::map<int, vehicleData>());

		// Tabla que relaciona calle de inicio (arista de termino), con la direccion inicial de un vehiculo.
		directionMap["18"] = 0; // Sur
		directionMap["8"] = 1;  // Oeste
		directionMap["9"] = 2;	// Norte
		directionMap["6"] = 3;	// Este

		// Tabla que relaciona id de zonas de llegada, con la direccion propia del programa.
		arrivalMap["1"] = 1;
		arrivalMap["2"] = 0;
		arrivalMap["3"] = 2;
		arrivalMap["4"] = 3;

		// Mapa de direccion a nombre de direccion
		direction_name[0] = "Sur";
		direction_name[1] = "Oeste";
		direction_name[2] = "Norte";
		direction_name[3] = "Este";

		// Intervalo de tiempo utilizado para actualizar cada paso de la simulacion.
		sim_update_interval = dynamic_cast<ExtTraCIScenarioManagerLaunchd*>(mobility->getManager())->par("updateInterval");

		// Variable utilizada para calcular tiempo dentro de la interseccion.
		time_in_junction = simTime().dbl();
		intersection_exit_time = -1.0;
		time_in_wait = -1.0;

        break;

    case 1:
    {
        // Organizacion de selfbeacons
        SimTime beginTime = SimTime(uniform(0.0, sim_update_interval / 2));

		// Intervalo de tiempo entre self-message
		ping_interval = SimTime(sim_update_interval / 2);

        self_beacon = new cMessage();
		sharedDataZoneMessage = new cMessage();

		// Envio de self-message
        scheduleAt(simTime() + beginTime, self_beacon);

		// Variables tamaño de zona para compartir informacion y zona de eleccion de lider (y bloqueo de tokens)
		shared_data_radio = par("ShareDataZoneRadio").doubleValue();
		lider_select_radio = par("LiderSelectZoneRadio").doubleValue();

    }
        break;
    default:
        break;
    }
}


void Base::finish()
{
    BaseWaveApplLayer::finish();

	// Obtencion de escenario
    ExtTraCIScenarioManagerLaunchd* sceman = dynamic_cast<ExtTraCIScenarioManagerLaunchd*>(mobility->getManager());

	// Estado que indica si el vehiculo llego a zona de destino antes de que termine simulacion.
    bool arrived;
    if (sceman->shuttingDown())
        arrived = false;
    else 
		arrived = true;
	
	// Registro de informacion.

	recordScalar("VehicleRemoved", vehicle_removed);
    recordScalar("ArrivedAtDest", arrived);

	recordScalar("IntersectionEnterTime", time_in_junction);
	recordScalar("IntersectionExitTime", intersection_exit_time);
	recordScalar("NumberOfCollisions", colision_list.size());

	// Resgistro de tiempo y vehiculo con el que se realizo colision
	for(auto it = colision_list.begin(); it != colision_list.end(); it++)
	{
		std::string s = "CollisionTimeWith-" + std::to_string(it->first);
		char *c;
		strcpy(c, s.c_str());

		recordScalar(c, it->second);
	}
}


void Base::handleSelfMsg(cMessage *msg)
{
	Base::getBasicParameters();

	if(axis_speed < 0.0001)
	{
		if(time_in_wait < 0.0)
			time_in_wait = 0.0;
		else
			time_in_wait += ping_interval.dbl();
	}
	else
	{
		time_in_wait = -1.0;
	}

	if(distance_to_junction <= lider_select_radio)
	{
		double dist_x = std::abs(position.x - intersection.x);
		double dist_y = std::abs(position.y - intersection.y);

		// Vehiculo esta dentro de la interseccion
		if(!(startId == "1" && dist_x <= 11.4 && dist_y <= 11.4))
		{
			// Vehiculo salio de la interseccion
			if(crossing)
			{
				Base::registerOutOfJunction();
			}
		}
	}

	bool first = true;
	for(auto it = carTable[direction_junction].begin(); it != carTable[direction_junction].end(); it++)
	{
		if(it->second.distance_to_junction < distance_to_junction)
			first = false;
	}

	if(first)
	{
		ExtTraCIScenarioManagerLaunchd* sceman = dynamic_cast<ExtTraCIScenarioManagerLaunchd*>(mobility->getManager());
		sceman->saveWaitingTime(direction_junction, time_in_wait);
	}
}


void Base::onData(WaveShortMessage *wsm)
{
}

void Base::onBeacon(WaveShortMessage *wsm)
{
}


/**
 * Funcion que prepara mensaje a enviar al resto de vehiculos
 */
NodeInfoMessage* Base::prepareNIM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, int serial) {
	NodeInfoMessage* wsm =	new NodeInfoMessage(name.c_str());
	wsm->addBitLength(headerLength);
	wsm->addBitLength(lengthBits);

	switch (channel) {
		case type_SCH: wsm->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
		case type_CCH: wsm->setChannelNumber(Channels::CCH); break;
	}

	wsm->setPsid(0);
	wsm->setPriority(priority);
	wsm->setWsmVersion(1);
	wsm->setTimestamp(simTime());
	wsm->setSenderAddress(myId);
	wsm->setRecipientAddress(rcvId);
	wsm->setSenderPos(curPosition);
	wsm->setSerial(serial);

	if (name == "beacon") {
		DBG << "Creating Beacon with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
	}
	if (name == "data") {
		DBG << "Creating Data with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
	}

	return wsm;
}


/**
 * Funcion que prepara contenido de mensaje enviado por los auto.
 */
void Base::prepareMsgData(vehicleData& data, int msgTipe)
{
	// Adicion de contenido basico al mensaje.
	data.msg_type = msgTipe;
	data.vehicle_id = myId;

	data.direction_junction = direction_junction;
	data.direction_out = direction_out;
	data.distance_to_junction = distance_to_junction;
	data.time_to_junction = time_to_junction;

	data.position = position;
	data.speed = velocity;
	data.acceleration = acceleration;
	data.axis_speed = axis_speed;

	data.stoped = stoped;
	data.stoping = stoping;
	data.crossing = crossing;

}


/**
 * Determina la direccion final de un auto
 */
int Base::finalDirection()
{
	auto routes_ids = traciVehicle->getPlannedRoadIds();
	std::string last = routes_ids.back();

	return arrivalMap[last];
}


/**
 * Funcion para simular una detencion hacia la esquina
 */
void Base::smartDetention()
{
	// Si esta fuera del radio de seleccion de lider, aun no es necesario que desacelere
	bool verificador = (distance_to_junction > lider_select_radio);

	// Verificar si no hay vehiculo que se este deteniendo mas adelante
	for(auto it = carTable[direction_junction].begin(); it != carTable[direction_junction].end(); it++)
	{
		if((it->second.stoping || it->second.stoped) && it->second.distance_to_junction < distance_to_junction)
		{
			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("grey"));
			verificador = true;
			if(stoping)
				traciVehicle->setSpeed(-1.0);
				
			stoping = false;
			stoped = false;

			EV << "    Someone near is stoping before\n";

			break;
		}
	}

	if(verificador)
		EV << ">>> Cant stop\n";

	// Vehiculo puede detenerse: es el primero
	if(!stoping && !verificador)
	{
		traciVehicle->setColor(Veins::TraCIColor::fromTkColor("red"));
		EV << ">>> STOPING <<<\n";
		traciVehicle->setSpeed(6.0);
		stoping = true;
	}
	else if(!stoped && !verificador)
	{
		if(distance_to_junction > 45)
			traciVehicle->setSpeed(6.0);
		else if(distance_to_junction > 35)
			traciVehicle->setSpeed(4.0);
		else if(distance_to_junction > 25)
			traciVehicle->setSpeed(1.5);
		else if(distance_to_junction > 17)
			traciVehicle->setSpeed(0.75);
		else if(distance_to_junction > 15)
		{
			traciVehicle->setSpeed(0.0);
			stoped = true;
		}
	}
}


/**
 * Calculo de tiempo aproximado a interseccion, considerando velocidad y aceleracion maxima.
 */
void Base::timeToJunction()
{
	double d_obj = distance_to_junction - 13;
	double t_acel = (max_velocidad - axis_speed) / max_accel;
	double t_with_acel = (-axis_speed + std::sqrt(axis_speed * axis_speed + 2 * max_accel * d_obj)) / max_accel;

	if(t_with_acel < t_acel)
		time_to_junction = t_with_acel;
	else
	{
		double d_with_acel1 = axis_speed * t_acel + (1/2) * max_accel * t_acel * t_acel;
		time_to_junction = t_acel + (d_obj - d_with_acel1) / max_velocidad;
	}

	if(time_to_junction < 0.0)
		time_to_junction = 0.0;

}


/**
 * Funcion que establece variables como velocidad, aceleracion, distancia a interseccion y tiempo estimado de llegada
 * a interseccion
 */
void Base::getBasicParameters()
{
	/////////////////////////////////////////////////////////////////
	// Obtencion de datos basicos.
	/////////////////////////////////////////////////////////////////
	EV << ">>> Obtencion de datos base <<<\n";

	position = curPosition;
	velocity = mobility->getCurrentSpeed();
	intersection = traci->junction("1").getPosition();
	position.x = -(position.x - intersection.x) + intersection.x;

	EV << "    position: " << position << "\n";
	EV << "    velocity: " << velocity << "\n";
	EV << "    intersection: " << intersection << "\n";

	laneId = traciVehicle->getLaneId();
	roadId = traciVehicle->getRoadId();
	startId = roadId.substr(0, roadId.find(":"));
	endId = roadId.substr(roadId.find(":")+1, roadId.size() - roadId.find(":") - 1);

	EV << "    Lane ID: " << laneId << "\n";
	EV << "    Start: " << startId << "\n";
	EV << "    End: " << endId << "\n";

	// Obtencion de direccion inicial y final.
	if(direction_junction == -1 && directionMap.count(startId))
		direction_junction = directionMap[startId];

	direction_out = finalDirection();

	EV << "    stoped: " << stoped << "\n";
	EV << "    direction_junction: " << direction_junction << "\n";
	EV << "    direction_out: " << direction_out << "\n";


	/////////////////////////////////////////////////////////////////
	// Precalculo - Tiempo de llegada a esquina.
	/////////////////////////////////////////////////////////////////

	// Velocidad y distancia a interseccion.
	axis_speed = std::abs(velocity.y);
	distance_to_junction = std::abs(traci->junction("1").getPosition().y - position.y);
	if(distance_to_junction < 0.0)
		distance_to_junction = 0.0;
		
	if(direction_junction % 2 == 1)
	{
		axis_speed = std::abs(velocity.x);
		distance_to_junction = std::abs(traci->junction("1").getPosition().x - position.x);
	}

	// Aceleracion.
	if(got_last_accel)
	{
		last_vel = axis_speed;
		got_last_accel = false;
	}
	else
	{
		acceleration = (axis_speed - last_vel) / sim_update_interval;
		got_last_accel = true;
	}

	// Tiempo aproximado para llegar a interseccion.
	timeToJunction();
	
	EV << "    got_last_accel: " << got_last_accel << "\n";
	EV << "    axis_speed: " << axis_speed << "\n";
	EV << "    last_vel: " << last_vel << "\n";
	EV << "    acceleration: " << acceleration << "\n";
	EV << "    distance_to_junction: " << distance_to_junction << "\n";
	EV << "    time to junction: " << time_to_junction << "\n";
}


/**
 * Funcion que detecta colision de vehiculo con otro, dada la info que envie este ultimo
 */
void Base::detectColision(vehicleData data)
{
	Coord p1 = position;
	Coord v1 = velocity;

	double theta1 = atan2(v1.y, v1.x);
	std::vector<Coord> lim1 = {Coord(1.0, 0.8), Coord(1.0, -0.8), Coord(-1.0, -0.8), Coord(-1.0, 0.8)};

	EV << ">>> p1: " << p1 << "   v1: " << v1 << "   theta1: " << theta1 <<"\n";

	getCarPoint(lim1, theta1);

	EV << ">>> Car points:\n    ";
	for(int i=0; i<4; i++)
	{
		EV << lim1[i] << " ";
		lim1[i] += p1;
	}
	EV << "\n";


	Coord p2 = data.position;
	Coord v2 = data.speed;

	double theta2 = atan2(v2.y, v2.x);

	std::vector<Coord> lim2 = {Coord(1.0, 0.8), Coord(1.0, -0.8), Coord(-1.0, -0.8), Coord(-1.0, 0.8)};

	EV << ">>> p2: " << p2 << "   v2: " << v2 << "   theta2: " << theta2 <<"\n";

	getCarPoint(lim2, theta2);

	EV << ">>> Car sender points:\n    ";
	for(int i=0; i<4; i++)
	{
		EV << lim2[i] << " ";
		lim2[i] += p2;
	}
	EV << "\n";


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


			car_intersect = car_intersect || (ver1 && ver2);

		}
	}

	if(car_intersect)
	{
		// Escribir en alguna parte que hay colision
		// puedo dejar un registro dentro del vehiculo la cantidad de colisiones.

		EV << ">>> Colision registrada\n";

		colision_list[data.vehicle_id] = simTime().dbl();

	}

}


/**
 * Funcion que obtiene las coordenadas de los vertices que representan un vehiculo
 */
void Base::getCarPoint(std::vector<Coord> &lim, double theta)
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
 * Registra un vehiculo fuera de la interseccion
 */
void Base::registerOutOfJunction()
{
	ExtTraCIScenarioManagerLaunchd* sceman = dynamic_cast<ExtTraCIScenarioManagerLaunchd*>(mobility->getManager());
	intersection_exit_time = simTime().dbl();
	sceman->carOutOfJunction(myId, direction_junction, intersection_exit_time - time_in_junction);
}