/*
Implementacion de protocolo en base a reserva de tokens.
*/

#include <veins/modules/application/Protocolos/Protocolo_Tokens/Tokens.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <stdlib.h>
#include <cstdlib>
#include <algorithm>


Define_Module(Tokens);


void Tokens::initialize(int stage)
{
    Base::initialize(stage);

    switch (stage)
    {
    case 0:
	{    
		// Id del token actualmente en uso
		id_token_in_use = -1;

		// Prioridad
		sum_priority = 0.0;
		priority = 0.0;
		allow_continue = 0;

		// Parametros de prioridad
		k_distance = par("k_distance").doubleValue();
		k_velocity = par("k_velocity").doubleValue();
		k_idle_time = par("k_idle_time").doubleValue();
		k_cars = par("k_cars").doubleValue();

		// Para el calculo de tiempo en detencion
		idling_time = 0.0;

		// Tabla: tokens para vehiculo que va hacia i, y despues de interseccion va a j.
		token_table = std::vector<std::vector<std::vector<int> > >(4, std::vector<std::vector<int>>(4, std::vector<int>()));
		token_vector.resize(17);
		setTokens();

		// Radios de zonas
		share_data_radio = par("share_data_radio").doubleValue();
		token_selection_radio = par("token_selection_radio").doubleValue();

		// Cantidad de intervalos entre mensajes de vehiculo
		intervals_per_selfmsg = 2;
		intervals_counting = 0;// rand() % intervals_per_selfmsg;//intervals_per_selfmsg - 1;

		first_msg = false;

        break;
	}
    default:
        break;
    }
}

void Tokens::finish()
{
    Base::finish();
	recordScalar("FinishPriority", priority);
}


void Tokens::handleSelfMsg(cMessage *msg){
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

	// Calculando tiempo en espera
	if(axis_speed <= 0.01)
		idling_time += ping_interval.dbl();

	// Recalculando prioridad con informacion actual.
	if(allow_continue != 2 || priority >= 0.0)
		calculateIndividualPriority();

	// Determinar tokens que se utilizaran
	if(tokens_list.size() == 0)
		getTokens();

	EV << "    sum_priority = " << sum_priority << "\n";
	EV << "    priority = " << priority << "\n";


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
	// Verificacion de bloqueo de tokens
	/////////////////////////////////////////////////////////////////
	if(distance_to_junction <= token_selection_radio)
	{
		EV << ">>> Zona de reparto y bloqueo de tokens\n";
		double dist_x = std::abs(position.x - traci->junction("1").getPosition().x);
		double dist_y = std::abs(position.y - traci->junction("1").getPosition().y);

		// Vehiculo esta dentro de la interseccion
		if(startId == "1" && dist_x <= 11.4 && dist_y <= 11.4)
		{
			EV << ">>> Zona de cruce <<<\n";
			crossing = true;
			Base::registerInOfJunction();

			// Actualizando indice de tokens que se esta ocupando actualmente.
			tokenInUse();
			
			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("blue"));

		}
		else
		{
			// Vehiculo salio de la interseccion
			if(crossing)
			{
				EV << ">>> Salida de interseccion <<<\n";
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
			// Vehiculo esta entrando a la interseccion.
			else
			{
				EV << ">>> Comparacion de prioridades <<<\n";
				// Solo comparamos primeros
				bool is_first = true;

				for(auto it = carTable[direction_junction].begin(); it!= carTable[direction_junction].end(); it++)
					if(it->second.distance_to_junction < distance_to_junction && !it->second.crossing)
						is_first = false;

				EV << "    is_first: " << is_first << "\n";

				// Comparacion de prioridades con vehiculos registrados.
				if(better_priority_cars.size() > 0 || !is_first)
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
						EV << ">>> Continuando con viaje, reservando prioridad maxima\n";

						priority = -1.0;
						traciVehicle->setColor(Veins::TraCIColor::fromTkColor("green"));
						Base::continueTravel();

						prepareMsgData(data, 0);
						info_message->setData(data);
						sendWSM((WaveShortMessage*) info_message->dup());

						scheduleAt(simTime() + ping_interval, self_beacon);

						return;

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
	if(distance_to_junction <= share_data_radio)
	{
		EV << ">>> Shared data zone <<<\n";
		prepareMsgData(data, 0);
		info_message->setData(data);
		
		if(!first_msg || intervals_counting % intervals_per_selfmsg == 0)
		{
			sendWSM((WaveShortMessage*) info_message->dup());
			first_msg = true;

			intervals_counting = 0;
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


void Tokens::onData(WaveShortMessage *wsm)
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

	std::vector<int> sender_tokens = token_table[sender_in][sender_out];
	double sender_priority = data.priority;
	int sender_token_in_use = data.id_token_in_use;

	EV << "    tipe: " << tipe << "\n";
	EV << "    sender: " << sender << "\n";
	EV << "    recipient: " << recipient << "\n";
	EV << "    time: " << data.time_to_junction << "\n";
	EV << "    sender_in: " << sender_in << "\n";
	EV << "    sender_out: " << sender_out << "\n";
	EV << "    sender_priority: " << sender_priority << "\n";
	EV << "    sender_cell_in_use: " << sender_token_in_use << "\n";


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

	// En caso de un vehiculo en la misma pista, revisar si aporta a nuestra prioridad y actualizar
	if(sender_in == direction_junction && sender_dist > distance_to_junction)
	{
		if(carTable[sender_in].count(sender) == 0)
			sum_priority += data.priority;
		else
			sum_priority += (data.priority - carTable[sender_in][sender].priority);
	}
		

	carTable[sender_in][sender] = data;

	// Antes de compara, actualizar informacion
	Base::getBasicParameters();
	if(allow_continue != 2 || priority >= 0.0)
		calculateIndividualPriority();

	EV << ">>> Sender data <<<\n";
	EV << "    time to junction: " << data.time_to_junction << "\n";
	EV << "    distance to junction: " << sender_dist << "\n";

	EV << ">>> Celdas y prioridad\n";
	EV << "    sender prioridad = " << data.priority << "\n";
	EV << "    propia proridad = " << priority << "\n";

	// Revisar colisiones con mensajes enviados y guardar aquellos vehiculos con mayor prioridad
	bool priority_comp = comparePriority(sender_priority, sender);

	if(sender_in == direction_junction)
	{
		// Vehiculos que esten atras se ignoran
		if(sender_dist > distance_to_junction)
			priority_comp = false;

		// Vehiculos que esten adelante se consideran
		if(sender_dist < distance_to_junction)
			priority_comp = true;

	}
	else
	{
		// Solo considerar informacion de los vehiculos que van primeros en pista o estan cruzando.
		for(auto it=carTable[sender_in].begin(); it!=carTable[sender_in].end(); it++)
		{
			if(it->first == sender)
				continue;

			if(it->second.distance_to_junction < sender_dist && !it->second.crossing)
				priority_comp = false;
		}
	}

	// Siempre considerar vehiculos que cruzan
	if(data.crossing)
		priority_comp = true;

	if(priority_comp)
	{
		EV << ">>> Comparando posibilidad de colision\n";
		if(compareTokens(sender_in, sender_out, sender_token_in_use))
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
		EV << "Tengo mejor prioridad, ignorando y eliminando del registro\n";
		better_priority_cars.erase(sender);
	}
}


void Tokens::onBeacon(WaveShortMessage *wsm)
{
}


/**
 * Funcion que prepara contenido de mensaje enviado por los auto.
 */
void Tokens::prepareMsgData(vehicleData& data, int msgTipe)
{
	Base::prepareMsgData(data, msgTipe);

	data.priority = priority;
	data.id_token_in_use = id_token_in_use;
}


/**
 * Inicializa la tabla de tokens seguns origen-destino y establece posicion de cada token.
 */
void Tokens::setTokens()
{
	token_table[0][0] = {1, 4, 8, 11, 15};
	token_table[0][1] = {1, 3};
	token_table[0][3] = {1, 5, 9, 14};

	token_table[1][0] = {7, 9, 12, 15};
	token_table[1][1] = {7, 6, 5, 4, 3};
	token_table[1][2] = {7, 2};

	token_table[2][1] = {16, 12, 8, 3};
	token_table[2][2] = {16, 13, 9, 6, 2};
	token_table[2][3] = {16, 14};

	token_table[3][0] = {10, 15};
	token_table[3][2] = {10, 8, 5, 2};
	token_table[3][3] = {10, 11, 12, 13, 14};


	Coord junction = traci->junction("1").getPosition();

	double width_street = 3.7;
	token_vector[1].create(1, junction + Coord(-width_street/2, -width_street));
	token_vector[2].create(2, junction + Coord(width_street/2, -width_street));
	token_vector[3].create(3, junction + Coord(-width_street, -width_street/2));
	token_vector[4].create(4, junction + Coord(-width_street/2, -width_street/2));
	token_vector[5].create(5, junction + Coord(0.0, -width_street/2));
	token_vector[6].create(6, junction + Coord(width_street/2, -width_street/2));
	token_vector[7].create(7, junction + Coord(width_street, -width_street/2));
	token_vector[8].create(8, junction + Coord(-width_street/2, 0.0));

	token_vector[9].create(9, junction + Coord(width_street/2, 0.0));
	token_vector[10].create(3, junction + Coord(-width_street, width_street/2));
	token_vector[11].create(4, junction + Coord(-width_street/2, width_street/2));
	token_vector[12].create(5, junction + Coord(0.0, width_street/2));
	token_vector[13].create(6, junction + Coord(width_street/2, width_street/2));
	token_vector[14].create(7, junction + Coord(width_street, width_street/2));
	token_vector[15].create(1, junction + Coord(-width_street/2, width_street));
	token_vector[16].create(2, junction + Coord(width_street/2, width_street));

}


/**
 * Funcion que obtiene la lista de tokens a usar, dados el origen y destino del vehiculo
 */
void Tokens::getTokens()
{
	tokens_list = token_table[direction_junction][direction_out];
	id_token_in_use = 0;
}


/**
 * Funcion de calculo de prioridad
 */
void Tokens::calculateIndividualPriority()
{
	double d = std::max(0.0, share_data_radio - distance_to_junction);
	priority = (k_distance * d) + (k_velocity * axis_speed) + (k_idle_time * idling_time) + (k_cars * sum_priority);
}

/**
 * Funcion que determina si prioridad de otro vehiculo es mejor que la del vehiculo actual.
 */
bool Tokens::comparePriority(double vhc_priority, int sender_id)
{
	double priority_delta = 1.5;

	if(priority < 0.0)
		return false;
	else
		return (vhc_priority < 0.0 || priority < vhc_priority - priority_delta || (std::abs(priority - vhc_priority) < priority_delta && sender_id < myId));
}


/**
 * Determina cual token se utilizara, dependiendo de la posicion del vehiculo
 */
void Tokens::tokenInUse()
{
	if(crossing)
	{
		EV << ">>> Comparando posicion vehiculo y posicion tokens\n";
		int id_near_token = -1;
		double dist_near_token = 1e8;

		for(int i=id_token_in_use; i<tokens_list.size(); i++)
		{
			Coord dist_vec = token_vector[tokens_list[i]].position_token - position;
			double dist = dist_vec.squareLength();
			EV << ">>> Comparando posicion respecto a token " << tokens_list[i] << "...\n";

			if(dist < dist_near_token)
			{
				EV << "    Cambiando token mas cercano: " << id_near_token << " a " << i << "\n";
				id_near_token = i;
				dist_near_token = dist;
			}
		}

		id_token_in_use = id_near_token;
		EV << "    Token " << id_token_in_use << " en uso\n";

		/*
		// TODO: posiblemente sea mejor cambiar el criterio de uso de tokens, vehiculo esta usando el token que se encuentre mas cerca
		for(int i=id_token_in_use; i<tokens_list.size(); i++)
		{
			// Booleanos de posicion de vehiculo respecto del i-esimo token en su lista.
			bool right = (token_vector[tokens_list[i]].position_token.x < position.x);
			bool down = (token_vector[tokens_list[i]].position_token.y < position.y);
			bool left = (token_vector[tokens_list[i]].position_token.x > position.x);
			bool up = (token_vector[tokens_list[i]].position_token.y > position.y);

			EV << ">>> Comparando posicion respecto a token " << tokens_list[i] << "...\n";
			// Booleano de correcta posicion de vehiculo respecto al i-esimo token en su lista.
			bool verificacion1 = (direction_junction == 0 && down) || (direction_junction == 1 && left) || (direction_junction == 2 && up) || (direction_junction == 3 && right);
			bool verificacion2 = (direction_out == 0 && down) || (direction_out == 1 && left) || (direction_out == 2 && up) || (direction_out == 3 && right);

			if(!verificacion1 || !verificacion2)
			{
				EV << "    Token " << tokens_list[i] << " aun en uso\n";
				id_token_in_use = i;
				break;
			}
		}
		*/
	}
}


bool Tokens::compareTokens(int in, int out, int token_in_use)
{
	bool colision = false;
	std::vector<int> other_token_list = token_table[in][out];

	EV << ">>> Tokens vehiculo a comparar (desde token " << other_token_list[token_in_use] << " id: " << token_in_use << "):\n";
	for(int i = token_in_use; i < other_token_list.size(); i++)
		EV << other_token_list[i] << " ";

	EV << "\n";

	EV << ">>> Tokens vehiculo propio:\n";
	for(int c : tokens_list)
		EV << c << " ";
	EV << "\n";


	for(int i = token_in_use; i < other_token_list.size(); i++)
	{
		int t = other_token_list[i];
		if(std::find(tokens_list.begin(), tokens_list.end(), t) != tokens_list.end())
		{
			colision = true;
			break;
		}
	}

	if(colision)
		EV << ">>> Existe colision\n";

	return colision;
}