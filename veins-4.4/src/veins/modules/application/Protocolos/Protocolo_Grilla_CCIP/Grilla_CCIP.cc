/*
Implementacion de variacion de protocolo de espacios discretos: CC-IP.
*/

#include <veins/modules/application/Protocolos/Protocolo_Grilla_CCIP/Grilla_CCIP.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <cstdlib>
#include <algorithm>


Define_Module(Grilla_CCIP);


void Grilla_CCIP::initialize(int stage)
{
    Base::initialize(stage);

    switch (stage)
    {
    case 0:
    {
		// Indice de la celda que se esta usando actualmente.
		id_cell_in_use = -1;

		in_block_area = false;
		in_cell_selection_zone = false;

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

void Grilla_CCIP::finish()
{
    Base::finish();
}


void Grilla_CCIP::handleSelfMsg(cMessage *msg){
	/////////////////////////////////////////////////////////////////
	// Obtencion de datos basicos.
	/////////////////////////////////////////////////////////////////
	Base::handleSelfMsg(msg);
	

	/////////////////////////////////////////////////////////////////
	// Calculo de prioridad
	/////////////////////////////////////////////////////////////////
	if(!in_block_area)
	{
		double aux_time = time_to_junction;
		double aux_dist = distance_to_junction;

		distance_to_junction += 13;
		distance_to_junction -= lider_select_radio;
		Base::timeToJunction();

		priority = simTime().dbl() + time_to_junction;
		time_to_junction = aux_time;
		distance_to_junction = aux_dist;
	}

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
		if(!in_block_area)
		{
			priority = simTime().dbl();
			in_block_area = true;
		}

		EV << ">>> Zona de reparto y bloqueo de celdas\n";
		double dist_x = std::abs(position.x - traci->junction("1").getPosition().x);
		double dist_y = std::abs(position.y - traci->junction("1").getPosition().y);

		// Vehiculo esta dentro de la interseccion
		if(startId == "1" && dist_x <= 11.4 && dist_y <= 11.4)
		{
			EV << ">>> Zona de cruce\n";
			crossing = true;

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

				return;
			}
			// vehiculo entrando a la interseccion.
			else
			{
				// mensaje para ciclo de obtencion de informacion
				if(!in_cell_selection_zone)
				{
					in_cell_selection_zone = true;
					info_message->setData(data);
					sendWSM((WaveShortMessage*) info_message->dup());
					
					scheduleAt(simTime() + ping_interval, sharedDataZoneMessage);

					return;
				}
				// TODO: cambiar forma de elegir si se puede cruzar
				/*
				- Revisar vehiculos que tienen mejor prioridad
					-> tal vez hay que esperar un ciclo de simulacion
				- Cuando un vehiculo se va de la interseccion, hay que revisar si mi prioridad me permite tomar las celdas
				*/
				// Comparacion de prioridades.

				// Comparacion de prioridades con vehiculos en registrados.
				for(int i=0; i<4; ++i)
				{
					if(better_priority_cars.size() > 0)
					{
						EV << "Existen vehiculos de mejor prioridad\n";
						Base::detention();
					}
					else
					{
						EV << "No existe vehiculo con mejor prioridad\n";
						Base::continueTravel();
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


void Grilla_CCIP::onData(WaveShortMessage *wsm)
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
		// TODO: autos que esperaban que pasara tienen que eliminarlo de la lista de espera
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
	int sender_cell_in_use = data.id_cell_begin;


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

	EV << ">>> Sender data <<<\n";
	EV << "    time to junction: " << data.time_to_junction << "\n";
	EV << "    distance to junction: " << sender_dist << "\n";

	EV << ">>> Celdas y prioridad\n";

	for(int t : sender_cells)
		EV << "    Sender will use celda: " << t << "\n";

	EV << "    sender priority = " << data.priority << "\n";
	EV << "    sender last_priority = " << data.last_priority << "\n";

	// Revisar colisiones con mensajes enviados y guardar aquellos vehiculos con mayor prioridad
	if(sender_priority < priority || (sender_priority == priority && sender > myId))
	{
		EV << "Comparando posibilidad de colision\n";
		if(compareCells(sender_in, sender_out, sender_cell_in_use))
		{
			EV << "Existe posible colision\n";
			better_priority_cars.insert(sender);
		}
		else
		{
			better_priority_cars.erase(sender);
		}
		
	}
	else
	{
		better_priority_cars.erase(sender);
	}
}


void Grilla_CCIP::onBeacon(WaveShortMessage *wsm)
{
}


/**
 * Funcion que prepara contenido de mensaje enviado por los auto.
 */
void Grilla_CCIP::prepareMsgData(vehicleData& data, int msgTipe)
{
	Base::prepareMsgData(data, msgTipe);

	data.priority = priority;
	data.id_cell_begin = id_cell_in_use;
}


/**
 * Inicializa la tabla de celdas seguns origen-destino y establece posicion de cada celda.
 */
void Grilla_CCIP::setCells()
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

	double junction_limit = 11.5;
	cell_register[1].create(1, junction + Coord(-junction_limit, 0.0), junction + Coord(0.0, -junction_limit));
	cell_register[2].create(2, junction, junction + Coord(junction_limit, -junction_limit));
	cell_register[3].create(3, junction + Coord(-junction_limit, junction_limit), junction);
	cell_register[4].create(3, junction + Coord(0.0, junction_limit), junction + Coord(junction_limit, 0.0));

}


/**
 * Funcion que obtiene la lista de celdas a usar, dados el origen y destino del vehiculo
 */
void Grilla_CCIP::getCells()
{
	cell_list = cells_table[direction_junction][direction_out];
	id_cell_in_use = 0;
}


/**
 * Determina cual celda se utilizara, dependiendo de la posicion del vehiculo
 */
void Grilla_CCIP::cellsUsed()
{
	EV << ">>> Comparando posicion vehiculo y posicion celdas\n";
	if(!crossing)
	{
		EV << ">>> Not crossing\n";
		return;
	}

	int i = id_cell_in_use;
	for(; i < cell_list.size(); i++)
	{
		int t = cell_list[i];

		Coord p1 = cell_register[t].p1;
		Coord p2 = cell_register[t].p2;

		EV << ">>> Celda " << t << " con puntos " << p1 << ", " << p2 << "\n";

		double x = position.x;
		double y = position.y;

		if(p1.x < x && x <= p2.x && p2.y < y && y <= p1.y)
		{
			EV << ">>> Cell " << t << " in use\n";
			id_cell_in_use = i;
			return ;
		}

		else
			EV << ">>> Cell " << t << " not in use\n";
		
	}

	id_cell_in_use = i;
}

bool Grilla_CCIP::compareCells(int in, int out, int cell_in_use)
{
	bool colision = false;
	std::vector<int> other_cell_list = cells_table[in][out];
	for(int t : cell_list)
	{
		if(std::find(other_cell_list.begin(), other_cell_list.end(), t) != other_cell_list.end()) // en caso que se pueda liberar celdas, usar cell_in_use al comparar
			colision = true;
			
	}

	return colision;
}