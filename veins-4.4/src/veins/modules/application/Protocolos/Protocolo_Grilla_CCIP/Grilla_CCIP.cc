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

		// Prioridad
		priority = 1e8;

		allow_continue = 0;

		// Tabla: celdas para vehiculo con direccion inicial i y direccion final j.
		cells_table = std::vector<std::vector<std::vector<int> > >(4, std::vector<std::vector<int>>(4, std::vector<int>()));
		cell_register.resize(5);
		setCells();

		// Radios de zonas
		shared_data_radio = par("shared_data_radio").doubleValue();
		cell_selection_radio = par("cell_selection_radio").doubleValue();

		// Cantidad de intervalos entre mensajes de vehiculo
		intervals_per_selfmsg = 4;
		intervals_counting = intervals_per_selfmsg - 1;

		first_msg = false;

        break;
	}
    default:
        break;
    }
}

void Grilla_CCIP::finish()
{
    Base::finish();
	recordScalar("FinishPriority", priority);
}


void Grilla_CCIP::handleSelfMsg(cMessage *msg){
	/////////////////////////////////////////////////////////////////
	// Obtencion de datos basicos.
	/////////////////////////////////////////////////////////////////
	Base::handleSelfMsg(msg);

	intervals_counting++;
	

	/////////////////////////////////////////////////////////////////
	// Calculo de prioridad
	/////////////////////////////////////////////////////////////////

	// Recalculando prioridad.
	if(allow_continue != 2 || priority >= 0.0)
		priority = time_to_junction;

	// Determinar celdas que se utilizaran
	if(cell_list.size() == 0)
		getCells();

	EV << "    priority: " << priority << "\n";


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
	if(distance_to_junction <= cell_selection_radio)
	{
		EV << ">>> Zona de reparto y bloqueo de celdas\n";
		double dist_x = std::abs(position.x - traci->junction("1").getPosition().x);
		double dist_y = std::abs(position.y - traci->junction("1").getPosition().y);

		// Vehiculo esta dentro de la interseccion
		if(startId == "1" && dist_x <= 11.4 && dist_y <= 11.4)
		{
			EV << ">>> Zona de cruce\n";
			crossing = true;
			Base::registerInOfJunction();

			// Actualizando indice de celda que se esta ocupando actualmente
			cellInUse();

			if(anti_block) 
			{
				anti_block = false;
				Base::continueTravel();
			}

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
				Base::registerOutOfJunction();

				outJunction = true;
				traciVehicle->setColor(Veins::TraCIColor::fromTkColor("purple"));
				
				prepareMsgData(data, 1);
				info_message->setData(data);
				sendWSM((WaveShortMessage*) info_message->dup());

				Base::removeVehicle(0);

				return;

			}
			// vehiculo entrando a la interseccion.
			else
			{
				// Comparacion de prioridades con vehiculos registrados.
				if(better_priority_cars.size() > 0)
				{
					EV << ">>> Existen vehiculos de mejor prioridad\n";
					for(auto it=better_priority_cars.begin(); it!=better_priority_cars.end(); it++)
						EV << "    " << *it << "\n";
					allow_continue = 0;
					
					Base::detention();
				}
				else
				{
					if(allow_continue == 2)
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
							//block_time = simTime().dbl();
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
						allow_continue++;
						Base::detention();
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
		
		if(!first_msg || intervals_counting % intervals_per_selfmsg == 0)
		{
			sendWSM((WaveShortMessage*) info_message->dup());
			first_msg = true;
		}

		// Al entrar a la zona para compartir informacion, vehiculo realiza un ciclo de espera
		// donde solo recive y envia mensajes de estado. No se realizan acciones adicionales.
		if(!inSharedDataZone)
		{
			EV << ">>> Entrando en zona de informacion, esperando un ciclo de simulacion...\n";
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

	EV << ">>> Data message <<<\n";
	NodeInfoMessage* msg = check_and_cast<NodeInfoMessage*>(wsm);
	vehicleData data = msg->getData();

	int tipe = data.msg_type;
	int sender = wsm->getSenderAddress();
	int recipient = wsm->getRecipientAddress();

	int sender_in = data.direction_junction;
	int sender_out = data.direction_out;
	double sender_dist = data.distance_to_junction;

	std::vector<int> sender_cells = cells_table[sender_in][sender_out];
	double sender_priority = data.priority;
	int sender_cell_in_use = data.id_cell_begin;

	EV << "    tipe: " << tipe << "\n";
	EV << "    sender: " << sender << "\n";
	EV << "    recipient: " << recipient << "\n";
	EV << "    time_junction: " << data.time_to_junction << "\n";
	EV << "    sender_in: " << sender_in << "\n";
	EV << "    sender_out: " << sender_out << "\n";
	EV << "    sender_priority: " << sender_priority << "\n";
	EV << "    sender_cell_in_use: " << sender_cell_in_use << "\n";


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

	carTable[sender_in][sender] = data;

	// Antes de compara, actualizar informacion
	Base::getBasicParameters();
	if(allow_continue != 2 || priority >= 0.0)
		priority = time_to_junction;

	EV << ">>> Sender data <<<\n";
	EV << "    time to junction: " << data.time_to_junction << "\n";
	EV << "    distance to junction: " << sender_dist << "\n";

	EV << ">>> Celdas y prioridad\n";
	EV << "    sender prioridad = " << data.priority << "\n";
	EV << "    propia proridad = " << priority << "\n";

	// Revisar colisiones con mensajes enviados y guardar aquellos vehiculos con mayor prioridad
	double priority_delta = 1.5;
	bool priority_comp = (sender_priority + priority_delta < priority) || (std::abs(sender_priority - priority) < priority_delta && sender < myId);
	
	if(sender_in == direction_junction && sender_dist > distance_to_junction)
		priority_comp = false;

	if(sender_in == direction_junction && sender_dist < distance_to_junction && !data.crossing)
		priority_comp = true;

	if(priority_comp)
	{
		EV << "Comparando posibilidad de colision\n";
		if(compareCells(sender_in, sender_out, sender_cell_in_use))
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


void Grilla_CCIP::cellInUse()
{
	if(crossing)
	{
		int i = id_cell_in_use;
		for(; i < cell_list.size(); i++)
		{
			int c = cell_list[i];

			Coord p1 = cell_register[c].p1;
			Coord p2 = cell_register[c].p2;

			EV << ">> Celda " << c << " con puntos " << p1 << ", " << p2 << "\n";

			double x = position.x;
			double y = position.y;

			if(p1.x < x && x <= p2.x && p2.y < y && y <= p1.y)
			{
				EV << ">>> Cell " << c << " in use\n";
				id_cell_in_use = i;

				return;
			}
			else
				EV << ">>> Cell " << c << " not in use\n";
			
		}
		id_cell_in_use = i;

	}
	EV << ">>> Celda en uso: " << cell_list[id_cell_in_use] << "(" << id_cell_in_use << ")\n";
}


bool Grilla_CCIP::compareCells(int in, int out, int cell_in_use)
{
	bool colision = false;
	std::vector<int> other_cell_list = cells_table[in][out];

	EV << ">>> Celdas vehiculo a comparar (desde celda " << other_cell_list[cell_in_use] << " id: " << cell_in_use << "):\n";
	for(int i = cell_in_use; i < other_cell_list.size(); i++)
		EV << other_cell_list[i] << " ";

	EV << "\n";

	EV << ">>> Celdas vehiculo propio:\n";
	for(int c : cell_list)
		EV << c << " ";
	EV << "\n";


	for(int i = cell_in_use; i < other_cell_list.size(); i++)
	{
		int t = other_cell_list[i];
		if(std::find(cell_list.begin(), cell_list.end(), t) != cell_list.end())
		{
			colision = true;
			break;
		}
	}

	if(colision)
		EV << ">>> Existe colision\n";

	return colision;
}