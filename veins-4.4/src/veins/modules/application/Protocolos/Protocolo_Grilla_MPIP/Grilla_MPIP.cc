/*
Implementacion de variacion de protocolo de espacios discretos: MP-IP.
*/

#include <veins/modules/application/Protocolos/Protocolo_Grilla_MPIP/Grilla_MPIP.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <cstdlib>
#include <algorithm>


Define_Module(Grilla_MPIP);

void Grilla_MPIP::initialize(int stage)
{
    Base::initialize(stage);

    switch (stage)
    {
    case 0:
    {
		// Indice de la celda que se esta usando actualmente.
		id_cell_begin = 0;
		id_cell_end = -1;
		priority = 1e8;
		intersection_priority = 1e8;

		allow_continue = 0;

		// Tabla: celdas para vehiculo con direccion inicial i y direccion final j.
		cells_table = std::vector<std::vector<std::vector<int> > >(4, std::vector<std::vector<int>>(4, std::vector<int>()));
		cell_register.resize(5);
		setCells();

        break;
	}
    default:
        break;
    }
}

void Grilla_MPIP::finish()
{
    Base::finish();
}


void Grilla_MPIP::handleSelfMsg(cMessage *msg){
	/////////////////////////////////////////////////////////////////
	// Obtencion de datos basicos.
	/////////////////////////////////////////////////////////////////
	Base::handleSelfMsg(msg);
	

	/////////////////////////////////////////////////////////////////
	// Calculo de prioridad
	/////////////////////////////////////////////////////////////////
	if(allow_continue != 1 || priority >= 0.0)
		priority = time_to_junction; //simTime().dbl() + time_to_junction;

	EV << "    priority: " << priority << "\n";

	if(cell_list.size() == 0)
		getCells();


	/////////////////////////////////////////////////////////////////
	// Preparar mensaje
	/////////////////////////////////////////////////////////////////
	info_message = prepareNIM("data", beaconLengthBits, type_CCH, beaconPriority, -1, -1);

	vehicleData data;
	prepareMsgData(data, 0);

	// Vehiculos mandan mensajes durante el ciclo de espera 
	if(msg == sharedDataZoneMessage)
	{
		prepareMsgData(data, 0);
		info_message->setData(data);
		sendWSM((WaveShortMessage*) info_message->dup());

		scheduleAt(simTime() + ping_interval, self_beacon);

		return;
	}


	/////////////////////////////////////////////////////////////////
	// Verificacion de bloqueo de celdas
	/////////////////////////////////////////////////////////////////
	if(distance_to_junction <= lider_select_radio)
	{
		EV << ">>> Zona de reparto y bloqueo de tokens\n";
		double dist_x = std::abs(position.x - traci->junction("1").getPosition().x);
		double dist_y = std::abs(position.y - traci->junction("1").getPosition().y);

		// Vehiculo esta dentro de la interseccion
		if(startId == "1" && dist_x <= 11.4 && dist_y <= 11.4)
		{
			EV << ">>> Zona de cruce\n";
			crossing = true;

			// Actualizando indice de celda que se esta ocupando actualmente
			cellsInUse();
			
			// TODO: manejo de uso parcial de la ruta

			if(better_priority_cars.size() > 0)
				incorrect_exit = true;

			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("blue"));

		}
		else
		{
			// Vehiculo salio de la interseccion
			if(crossing)
			{
				EV << ">>> Out of junction <<<\n";
				outJunction = true;
				traciVehicle->setColor(Veins::TraCIColor::fromTkColor("purple"));
				
				prepareMsgData(data, 1);
				info_message->setData(data);
				sendWSM((WaveShortMessage*) info_message->dup());

				return ;

			}
			// Vehiculo entrando a la interseccion.
			else
			{
				// Comparacion de prioridades con vehiculos registrados.
				if(better_priority_cars.size() > 0)
				{
					EV << ">>> Existen vehiculos de mejor prioridad\n";
					// TODO: revisar cual es la maxima celda que se puede alcanzar considerando los vehiculos con mejor prioridad.
					allow_continue = 0;
					Base::detention();
				}
				else
				{
					if(allow_continue == 1)
					{
						EV << ">>> No existe vehiculo con mejor prioridad\n";
						bool isFirst = true;
						for(auto it=carTable[direction_junction].begin(); it != carTable[direction_junction].end(); it++)
							if(it->second.distance_to_junction < distance_to_junction && !it->second.crossing)
								isFirst = false;

						if(isFirst)
						{
							EV << ">>> Continuando con viaje, reservando prioridad maxima\n";
							priority = -1.0;
							Base::continueTravel();
						}
						else
						{
							EV << ">>> No se puede continuar, realizando detencion\n";
							allow_continue = 0;
							Base::detention();
						}
					}
					else
					{
						EV << ">>> Hasta ahora no hay vehiculo con mejor prioridad\n";
						EV << ">>> Esperando un ciclo de simulacion para asegurar\n";
						allow_continue = 1;
					}
				}
			}
		}
	}


	/////////////////////////////////////////////////////////////////
	// Area de envio de infomacion
	/////////////////////////////////////////////////////////////////
	if(distance_to_junction <= shared_data_radio)
	{
		EV << ">>> Shared data zone <<<\n";
		prepareMsgData(data, 0);
		info_message->setData(data);
		sendWSM((WaveShortMessage*) info_message->dup());

		// Al entrar a la zona para compartir informacion, vehiculo realiza un ciclo de espera
		// donde solo recive y envia mensajes de estado. No se realizan acciones adicionales.
		if(!inSharedDataZone)
		{
			EV << ">>> New car in shared data zone, waiting one cicle of simulation...\n";
			inSharedDataZone = true;
			scheduleAt(simTime() + ping_interval, sharedDataZoneMessage);
			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("yellow"));
		} 
		else
			scheduleAt(simTime() + ping_interval, self_beacon);

		return;

	}

	scheduleAt(simTime() + ping_interval, self_beacon);

}


void Grilla_MPIP::onData(WaveShortMessage *wsm)
{
	EV << ">>> Data message <<<\n";
	NodeInfoMessage* msg = check_and_cast<NodeInfoMessage*>(wsm);
	vehicleData data = msg->getData();

	int tipe = data.msg_type;
	int sender = wsm->getSenderAddress();
	int recipient = wsm->getRecipientAddress();

	EV << "    tipe: " << tipe << "\n";
	EV << "    sender: " << sender << "\n";
	EV << "    recipient: " << recipient << "\n";
	EV << "    time: " << data.time_to_junction << "\n";

	// Auto que recibe mensaje salio de la interseccion.
	if(outJunction)
	{
		EV << ">>> OUT OF JUNCTION <<<\n";
		return ;
	}

	// Auto aun no esta en area para compartir informacion.
	if(!inSharedDataZone)
	{
		EV << ">>> CANT RECEIVE, OUT OF SHARED DATA ZONE <<<\n";
		return ;
	}

	int sender_in = data.direction_junction;
	int sender_out = data.direction_out;
	double sender_dist = data.distance_to_junction;

	std::vector<int> sender_cells = cells_table[sender_in][sender_out];
	double sender_priority = data.priority;
	int sender_id_cell_begin = data.id_cell_begin;
	int sender_id_cell_end = data.id_cell_end;

	EV << "    id_cell_begin: " << data.id_cell_begin << "\n";
	EV << "    id_cell_end: " << data.id_cell_end << "\n";
	EV << "    sender_priority: " << sender_priority << "\n";
	EV << "    my prioridad: " << priority << "\n";


	/////////////////////////////////////////////////////////////////
	// Tipos de mensaje
	/////////////////////////////////////////////////////////////////


	// Tipo de mensaje 1: Vehiculo ya salio de la interseccion
	if(tipe == 1)
	{
		EV << ">>> Delete car " << sender << " from table <<<\n";
		carTable[sender_in].erase(sender);
		better_priority_cars.erase(sender);

		return ;
	}
	
	detectColision(data);
	carTable[sender_in][sender] = data;

	// Antes de compara, actualizar informacion
	Base::getBasicParameters();
	if(allow_continue != 1 || priority >= 0.0)
		priority = time_to_junction;

	EV << ">>> Sender data <<<\n";
	EV << "    time to junction: " << data.time_to_junction << "\n";
	EV << "    distance to junction: " << sender_dist << "\n";

	EV << ">>> Celdas y prioridad\n";

	for(int t : sender_cells)
		EV << "    Sender will use celda: " << t << "\n";

	EV << "    sender priority = " << sender_priority << "\n";
	EV << "    sender last_priority = " << data.last_priority << "\n";

	// Revisar colisiones con mensajes enviados y guardar aquellos vehiculos con mayor prioridad
	bool priority_comp = (sender_priority + 0.5 < priority) || (std::abs(sender_priority - priority) < 0.5 && sender < myId);
	if(priority_comp)
	{
		EV << "Comparando posibilidad de colision\n";
		if(compareCells(sender_in, sender_out, sender_id_cell_begin, sender_id_cell_end))
		{
			EV << "Existe posible colision\n";
			better_priority_cars.insert(sender);
		}
		else
		{
			EV << "No existe colision\n";
			better_priority_cars.erase(sender);
		}
		
	}
	else
	{
		EV << "Tengo mejor prioridad, eliminando del registro\n";
		better_priority_cars.erase(sender);
	}

}


void Grilla_MPIP::onBeacon(WaveShortMessage *wsm)
{
}


/**
 * Funcion que prepara contenido de mensaje enviado por los auto.
 */
void Grilla_MPIP::prepareMsgData(vehicleData& data, int msgTipe)
{
	Base::prepareMsgData(data, msgTipe);

	data.priority = priority;

	data.id_cell_begin = id_cell_begin;
	data.id_cell_end = id_cell_end;

}


/**
 * Inicializa la tabla de celdas seguns origen-destino y establece posicion de cada celda.
 */
void Grilla_MPIP::setCells()
{
	cells_table[0][0] = {1, 3};
	cells_table[0][1] = {1};
	cells_table[0][3] = {1, 2, 4};

	cells_table[1][0] = {2, 4, 3};
	cells_table[1][1] = {2, 1};
	cells_table[1][2] = {2};

	cells_table[2][1] = {4, 3, 1};
	cells_table[2][2] = {4, 2};
	cells_table[2][3] = {4};

	cells_table[3][0] = {3};
	cells_table[3][2] = {3, 1, 2};
	cells_table[3][3] = {3, 4};


	Coord junction = traci->junction("1").getPosition();
	double junction_limit = 13;

	cell_register[1].create(1, junction + Coord(-junction_limit, 0.0), junction + Coord(0.0, -junction_limit));
	cell_register[2].create(2, junction, junction + Coord(junction_limit, -junction_limit));
	cell_register[3].create(3, junction + Coord(-junction_limit, junction_limit), junction);
	cell_register[4].create(3, junction + Coord(0.0, junction_limit), junction + Coord(junction_limit, 0.0));

}


/**
 * Funcion que obtiene la lista de celdas a usar, dados el origen y destino del vehiculo
 */
void Grilla_MPIP::getCells()
{
	cell_list = cells_table[direction_junction][direction_out];
}


/**
 * Determina cual celda se esta utilizando. Si no hay ninguna en uso, no hace nada.
 */
void Grilla_MPIP::cellsInUse()
{
	EV << ">>> Comparando posicion vehiculo y posicion celdas\n";


}


/**
 * Funcion que detiene auto en ultima celda bloqueada a nombre del vehiculo.
 */
void Grilla_MPIP::detentionLastCell()
{
	// TODO: adaptar al cambio del manejo de prioridades
	traciVehicle->setColor(Veins::TraCIColor(0, 255, 255, 0));
	stoped = false;
	stoping = false;

	EV << ">>> Preparando detencion dentro de interseccion\n";

	if(id_cell_end == id_cell_begin)
	{
		if(id_cell_begin == 0 && !crossing)
			traciVehicle->setSpeed(2.0);
		else
			traciVehicle->setSpeed(0.0);

	}
	else
		traciVehicle->setSpeed(4.0);

}


bool Grilla_MPIP::compareCells(int in, int out, int id_begin, int id_end)
{
	bool colision = false;
	std::vector<int> other_cell_list = cells_table[in][out];

	// TODO: comparacion de celdas a utilizar

	return colision;
}


/**
 *Funcion para remover vehiculos cuando hay bloque producto de conflicto entre paramics y la simulacion.
 */
void Grilla_MPIP::removeVehicle(int reason)
{
	traciVehicle->remove(reason);
	vehicle_removed = true;

	// Por ahora suponemos que reason = 0 es para simular un arrivo
	if(reason == 0)
		Base::registerOutOfJunction();
}