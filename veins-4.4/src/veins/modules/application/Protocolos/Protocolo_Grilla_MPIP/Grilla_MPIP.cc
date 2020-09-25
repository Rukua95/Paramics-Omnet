#include <veins/modules/application/Protocolos/Protocolo_Grilla_MPIP/Grilla_MPIP.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <cstdlib>
#include <algorithm>

/*
Implementacion de variacion de protocolo de espacios discretos: CC-IP.
*/

Define_Module(Grilla_MPIP);

void Grilla_MPIP::initialize(int stage)
{
    Base::initialize(stage);

    switch (stage)
    {
    case 0:
    {
		// Indice de la celda que se esta usando actualmente.
		id_cell_begin = -1;
		id_cell_end = -1;
		id_last_cell_reserved = -1;

		// Prioridad
		priority = 1e8;
		block_priority = false;
		has_bloqued_cells = false;

		allow_continue = 0;

		// Tabla: celdas para vehiculo con direccion inicial i y direccion final j.
		cells_table = std::vector<std::vector<std::vector<int> > >(4, std::vector<std::vector<int>>(4, std::vector<int>()));
		cell_register.resize(5);
		bloqued_cells.assign(5, std::set<int>());
		setCells();

		// Radios de zonas
		shared_data_radio = par("shared_data_radio").doubleValue();
		cell_selection_radio = par("cell_selection_radio").doubleValue();

		// Cantidad de intervalos entre mensajes de vehiculo
		intervals_per_selfmsg = 1;
		intervals_counting = rand() % intervals_per_selfmsg;//intervals_per_selfmsg - 1;

		first_msg = false;

        break;
	}
    default:
        break;
    }
}

void Grilla_MPIP::finish()
{
    Base::finish();
	recordScalar("FinishPriority", priority);
}


void Grilla_MPIP::handleSelfMsg(cMessage *msg){
	/////////////////////////////////////////////////////////////////
	// Obtencion de datos basicos.
	/////////////////////////////////////////////////////////////////
	Base::handleSelfMsg(msg);
	
	if(outJunction)
	{
		Base::removeVehicle(0);
		return;
	}

	intervals_counting++;

	
	/////////////////////////////////////////////////////////////////
	// Calculo de prioridad
	/////////////////////////////////////////////////////////////////

	// Recalculando prioridad.
	if(!block_priority)
		priority = time_to_junction;

	// Determinar celdas que se utilizaran
	if(cell_list.size() == 0)
		getCells();

	EV << "> Prioridad: " << priority << "\n";
	EV << "> Celdas a usar: ";
	for(int cell : cell_list)
		EV << cell << " ";
	EV << "\n";


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
			
		}
		else
		{
			// Vehiculo salio de la interseccion
			if(crossing)
			{
				EV << ">>> Out of junction <<<\n";
				Base::registerOutOfJunction();

				crossing = false;
				outJunction = true;
				traciVehicle->setColor(Veins::TraCIColor::fromTkColor("purple"));
				
				prepareMsgData(data, 1);
				info_message->setData(data);
				sendWSM((WaveShortMessage*) info_message->dup());

				scheduleAt(simTime() + ping_interval, self_beacon);

				return;
			}
			// vehiculo entrando a la interseccion.
			else
			{

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


void Grilla_MPIP::onData(WaveShortMessage *wsm)
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
	int sender_cell_begin = data.id_cell_begin;
	int sender_cell_end = data.id_cell_end;

	EV << "    tipe: " << tipe << "\n";
	EV << "    sender: " << sender << "\n";
	EV << "    recipient: " << recipient << "\n";
	EV << "    time_junction: " << data.time_to_junction << "\n";
	EV << "    sender_in: " << sender_in << "\n";
	EV << "    sender_out: " << sender_out << "\n";
	EV << "    sender_priority: " << sender_priority << "\n";
	EV << "    sender_cell_in_use: " << sender_cell_begin << "\n";
	EV << "    sender_cell_in_use: " << sender_cell_end << "\n";


	/////////////////////////////////////////////////////////////////
	// Tipos de mensaje
	/////////////////////////////////////////////////////////////////


	// Tipo de mensaje 1: Vehiculo ya salio de la interseccion
	if(tipe == 1)
	{
		EV << ">>> Delete car " << sender << " from table <<<\n";
		carTable[sender_in].erase(sender);
		better_priority_cars.erase(sender);
		for(int i=1; i<=4; i++)
			bloqued_cells[i].erase(sender);

		return ;
	}
	
	// Mensaje especial de vehiculos que se encuentran cruzando interseccion y se detendran en alguna celda
	if(tipe == 2)
	{

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

	// Revisar diferencia de prioridad con vehiculo que envio msg, utilizando un intervalo de confianza.
	double priority_comp = comparePriority(data, sender);


	// Si hay diferencia en prioridad y vehiculo cumple las condiciones, se compara las celdas que se estan utilizando
	if(priority_comp)
	{
		EV << "Comparando posibilidad de colision\n";
		if(Grilla_MPIP::compareCells(sender_in, sender_out, sender_cell_begin))
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

	double junction_limit = 11.5;
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
	id_cell_begin = 0;
}


/**
 * Funcion para determinar la celda que esta actualmente en uso.
 */
void Grilla_MPIP::cellInUse()
{
	// Aseguramos que vehiculo esta cruzando
	if(crossing)
	{
		int i = id_cell_begin;
		for(; i < cell_list.size(); i++)
		{
			int c = cell_list[i];

			Coord p1 = cell_register[c].p1;
			Coord p2 = cell_register[c].p2;

			EV << ">> Celda " << c << " con puntos " << p1 << ", " << p2 << "\n";

			double x = position.x;
			double y = position.y;

			// Identificar si vehiculo se encuentra dentro de la celda
			if(p1.x < x && x <= p2.x && p2.y < y && y <= p1.y)
			{
				EV << ">>> Cell " << c << " in use\n";
				id_cell_begin = i;

				return;
			}
			else
				EV << ">>> Cell " << c << " not in use\n";
			
		}
		id_cell_begin = i;

	}

	EV << ">>> Celda en uso: " << cell_list[id_cell_begin] << "(" << id_cell_begin << ")\n";
}


/**
 * Funcion para determinar cual es la ultima celda que se puede alcanzar
 */
int Grilla_MPIP::lastCell()
{
	// Determinamos la ultima celda que puede ser utilizada, considerando la prioridad que tienen otros vehiculos
	int id_last_posible_cell = cell_list.size() - 1;

	// Comparamos con vehiculos que intersectan con mejor prioridad
	for(auto it=better_priority_cars.begin(); it!=better_priority_cars.end(); it++)
	{
		int id = *it;

		// Identificamos direccion de entrada y salida de vehiculo
		int vh_in = 0, vh_out = 0;
		for(int i=0; i<4; i++)
		{
			if(carTable[i].count(id) != 0)
			{
				vh_in = carTable[i][id].direction_junction;
				vh_out = carTable[i][id].direction_out;

				break;
			}
		}

		EV << "Determinando colision de celdas con vehiculo " << id << "\n";
		std::vector<int> vh_cell_list = cells_table[vh_in][vh_out];
		for(int i=0; i<cell_list.size(); i++)
		{
			int cell = cell_list[i];

			if(std::find(vh_cell_list.begin(), vh_cell_list.end(), cell) != vh_cell_list.end())
			{
				EV << "> Celda mas lejana sin intersecctar: " << i-1 << "\n";
				id_last_posible_cell = std::min(id_last_posible_cell, i-1);
				break;
			}
		}
	}

	// Determinamos la ultima celda que puede ser utilizada
	for(int i=0; i<cell_list.size(); i++)
	{
		int cell = cell_list[i];
		if(bloqued_cells[cell].size() > 0)
		{
			EV << "> Celda mas lejana sin bloqueo: " << i-1 << "\n";
			id_last_posible_cell = std::min(id_last_posible_cell, i - 1);
			break;
		}
	}

	EV << ">> Celda mas lejana a la que se puede llegar: " << id_last_posible_cell << "\n";

	return id_last_posible_cell;
}


/**
 * Funcion que compara la prioridades
 */
bool Grilla_MPIP::comparePriority(vehicleData data, int sender_id)
{
	double sender_priority = data.priority;
	double sender_in = data.direction_junction;
	double sender_dist = data.distance_to_junction;
	double sender = sender_id;

	double priority_delta = 2.0;
	
	//bool priority_comp = (sender_priority + priority_delta < priority) || (std::abs(sender_priority - priority) < priority_delta && sender < myId);
	bool priority_comp = (sender < myId);
	
	// Solo se toma en cuenta informacion de los vehiculos que van primeros en pista o estan cruzando.
	bool is_sender_first = true; 
	for(auto it=carTable[sender_in].begin(); it!=carTable[sender_in].end(); it++)
	{
		// Omitimos vehiculo que envio msg
		if(it->first == sender)
			continue;

		if(it->second.distance_to_junction < sender_dist && !it->second.crossing)
			is_sender_first = false;
	}

	// Vehiculos cruzando siempre se toman en cuenta.
	if(data.crossing)
		is_sender_first = true;

	// Vehiculo que no esta primero NO se toma en cuenta, independiente de que pueda tener mejor prioridad.
	if(!is_sender_first)
		priority_comp = false;

	return priority_comp;
}


/**
 * Funcion que determina si hay colision con otro vehiculo. 
 * Una colision ocurre si ambos vehiculos comparte celdas en su recorrido.
 */
bool Grilla_MPIP::compareCells(int in, int out, int cell_in_use)
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

	// Iteramos por las celdas que utiliza el otro vehiculo
	for(int i = cell_in_use; i < other_cell_list.size() && !colision; i++)
	{
		int t = other_cell_list[i];
		// Iteramos por las celdas que utiliza este vehiculo
		for(int j=id_cell_begin; j < cell_list.size(); j++)
		{
			if(t == cell_list[j])
			{
				colision = true;
				break;
			}
		}
	}

	if(colision)
		EV << ">>> Existe colision\n";

	return colision;
}


/**
 * Metodo para detencion de vehiculos en ultima celda posible
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
