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
#include <random>


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
		// TODO: agregar funcionalidad en plugin o utilizarla si es que existe
		car_lenght = 4.0;
		car_width = 1.6;

		// Estados de vehiculos
		stoping = false;
		stoped = false;

		// Flag de vehiculo fuera de interseccion
		outJunction = false;

		// Flag de vehiculo cruzando interseccion
		crossing = false; 

		// Flag extras
		anti_block = true;
		incorrect_exit = false;
		inSharedDataZone = false;
		anounce_msg = true;

		// Direccion inicial de vehiculo
		direction_junction = -1;

		// Distancia a interseccion
		distance_to_junction = -1.0;

		// Tiempo estimado de llegada a interseccion
		time_to_junction = -1.0;

		vehicle_removed = false;
		stuck = 0;
		stuck_reference_time = 0.0;

		// Variables de maxima y minima aceleracion, y maxima velocidad (ajustadas manualmente a la simulacion)
		// TODO: estos valores deberian obtenerse a traves de veins
		max_accel = 2.5; // traciVehicle->getDeccel();
		min_accel = -4.5; // traciVehicle->getAccel();
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
		double delta = 0.1;//uniform(0.0, 1.0);

		EV << ">>> Delta initial selfmsg: " << delta << "\n";
		EV << ">>> Interval time: " << sim_update_interval << "\n";

        SimTime beginTime = SimTime(delta);

		// Intervalo de tiempo entre self-message
		ping_interval = SimTime(sim_update_interval);

        self_beacon = new cMessage();
		sharedDataZoneMessage = new cMessage();

		// Envio de self-message
        scheduleAt(simTime() + beginTime, self_beacon);

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
	recordScalar("InitialDirection", direction_junction);
	recordScalar("FinalDirection", direction_out);

	recordScalar("VehicleRemoved", vehicle_removed);
    recordScalar("ArrivedAtDest", arrived);
	if(!arrived)
	{
		recordScalar("DistanceJunction", distance_to_junction);
	}

	recordScalar("MaxStuckTime", stuck);
	recordScalar("StuckReferenceTime", stuck_reference_time);

	recordScalar("IntersectionEnterTime", time_in_junction);
	recordScalar("IntersectionExitTime", intersection_exit_time);
}


void Base::handleSelfMsg(cMessage *msg)
{
	Base::getBasicParameters();

	if(axis_speed < 0.01)
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

	bool first = true; //distance_to_junction < shared_data_radio;
	for(auto it = carTable[direction_junction].begin(); it != carTable[direction_junction].end(); it++)
	{
		if(it->second.distance_to_junction < distance_to_junction && !it->second.crossing)
			first = false;
	}

	if(first)
	{
		ExtTraCIScenarioManagerLaunchd* sceman = dynamic_cast<ExtTraCIScenarioManagerLaunchd*>(mobility->getManager());
		sceman->saveWaitingTime(direction_junction, time_in_wait);

		if(time_in_wait > stuck)
		{
			stuck_reference_time = simTime().dbl();
			stuck = time_in_wait;
		}
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
	data.axis_speed = axis_speed;

	data.stoped = stoped;
	data.stoping = stoping;
	data.crossing = crossing;

}


/**
 * Determina la direccion final de un vehiculo
 */
int Base::finalDirection()
{
	auto routes_ids = traciVehicle->getPlannedRoadIds();
	std::string last = routes_ids.back();

	return arrivalMap[last];
}


/**
 * Funcion para simular una detencion en interseccion
 */
void Base::detention()
{
	// Si esta fuera del radio de seleccion de lider, aun no es necesario que desacelere
	bool verificador = false;

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

			EV << "    Someone stoping in front\n";

			break;
		}
	}

	if(verificador)
		EV << ">>> Cant stop\n";
	else
	{
		stoping = true;
		//traciVehicle->setColor(Veins::TraCIColor::fromTkColor("red"));

		if(distance_to_junction > 45)
			traciVehicle->setSpeed(4.0);
		else if(distance_to_junction > 35)
			traciVehicle->setSpeed(3.0);
		else if(distance_to_junction > 25)
			traciVehicle->setSpeed(2.0);
		else if(distance_to_junction > 17)
			traciVehicle->setSpeed(1.0);
		else if(distance_to_junction <= 17)
		{
			traciVehicle->setSpeed(0.0);
			stoped = true;
		}
	}
}


/**
 * Funcion para continuar viaje de un vehiculo en detencion
 */
void Base::continueTravel()
{
	traciVehicle->setColor(Veins::TraCIColor::fromTkColor("green"));
	traciVehicle->setSpeed(-1.0);
	stoped = false;
	stoping = false;
}


/**
 * Calculo de tiempo aproximado a interseccion, considerando velocidad y aceleracion maxima.
 */
void Base::timeToJunction()
{
	double d_obj = std::max(0.0, distance_to_junction - 17);
	double t_acel = (max_velocidad - axis_speed) / max_accel;
	double t_with_acel = (-axis_speed + std::sqrt(axis_speed * axis_speed + 2 * max_accel * d_obj)) / max_accel;

	if(t_with_acel < t_acel)
		time_to_junction = t_with_acel;
	else
	{
		double d_with_acel1 = axis_speed * t_acel + (1/2) * max_accel * t_acel * t_acel;
		time_to_junction = t_acel + (d_obj - d_with_acel1) / max_velocidad;
	}

	if(time_to_junction < 0.0 || d_obj < 0.0)
		time_to_junction = 0.0;

}


/**
 * Funcion que determina parametros basicos, como velocidad y distancia a interseccion
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
	
	laneId = traciVehicle->getLaneId();
	roadId = traciVehicle->getRoadId();
	startId = roadId.substr(0, roadId.find(":"));
	endId = roadId.substr(roadId.find(":")+1, roadId.size() - roadId.find(":") - 1);


	// Obtencion de direccion inicial y final.
	if(direction_junction == -1 && directionMap.count(startId))
		direction_junction = directionMap[startId];

	direction_out = finalDirection();


	/////////////////////////////////////////////////////////////////
	// Precalculo - Tiempo de llegada a esquina.
	/////////////////////////////////////////////////////////////////

	// Velocidad y distancia a interseccion.
	axis_speed = std::abs(velocity.y);
	distance_to_junction = std::abs(traci->junction("1").getPosition().y - position.y);
		
	if(direction_junction % 2 == 1)
	{
		axis_speed = std::abs(velocity.x);
		distance_to_junction = std::abs(traci->junction("1").getPosition().x - position.x);
	}

	// Tiempo aproximado para llegar a interseccion.
	timeToJunction();
	
	// 
	EV << "    position: " << position << "\n";
	EV << "    velocity: " << velocity << "\n";
	EV << "    intersection: " << intersection << "\n";
	EV << "    Lane ID: " << laneId << "\n";
	EV << "    Start: " << startId << "\n";
	EV << "    End: " << endId << "\n";
	EV << "    stoped: " << stoped << "\n";
	EV << "    direction_junction: " << direction_junction << "\n";
	EV << "    direction_out: " << direction_out << "\n";
	EV << "    axis_speed: " << axis_speed << "\n";
	EV << "    distance_to_junction: " << distance_to_junction << "\n";
	EV << "    time to junction: " << time_to_junction << "\n";
}


/**
 * Funcion que registra un vehiculo que sale de la interseccion
 */
void Base::registerOutOfJunction()
{
	ExtTraCIScenarioManagerLaunchd* sceman = dynamic_cast<ExtTraCIScenarioManagerLaunchd*>(mobility->getManager());
	intersection_exit_time = simTime().dbl();
	sceman->carOutOfJunction(myId, direction_junction, intersection_exit_time - time_in_junction);
}


/**
 * Funcion que registra un vehiculo que entra en la interseccion
 */
void Base::registerInOfJunction()
{
	traciVehicle->setColor(Veins::TraCIColor::fromTkColor("blue"));
	
	ExtTraCIScenarioManagerLaunchd* sceman = dynamic_cast<ExtTraCIScenarioManagerLaunchd*>(mobility->getManager());

	Coord coord_position = position;
	Coord coord_speed = velocity;
	sceman->carInOfJunction(myId, coord_position, coord_speed);
}


/**
 *Funcion para remover vehiculos cuando hay bloque producto de conflicto entre paramics y la simulacion.
 */
void Base::removeVehicle(int reason)
{
	traciVehicle->remove(reason);
	vehicle_removed = true;

	// Por ahora suponemos que reason = 0 es para simular un arrivo
	if(reason == 0)
		Base::registerOutOfJunction();
}
