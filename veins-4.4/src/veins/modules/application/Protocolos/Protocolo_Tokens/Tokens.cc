//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include <veins/modules/application/Protocolos/Protocolo_Tokens/Tokens.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
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
		// Tokens a usar y prioridad
		last_individual_priority = 0.0;
		individual_priority = 0.0;
		sum_priority = 0.0;

		k_distance = 1.0;
		k_velocity = 1.0;
		k_idle_time = 1.0;
		k_cars = 1.0;

		idling_time = 0.0;

		id_token_in_use = -1;


		// tabla: tokens para vehiculo que va hacia i, y despues de interseccion va a j.
		token_table = std::vector<std::vector<std::vector<int> > >(4, std::vector<std::vector<int>>(4, std::vector<int>()));
		token_vector.resize(17);
		setTokens();


        break;
	}
    default:
        break;
    }
}

void Tokens::finish()
{
    Base::finish();
}


void Tokens::handleSelfMsg(cMessage *msg){
	/////////////////////////////////////////////////////////////////
	// Obtencion de datos basicos.
	/////////////////////////////////////////////////////////////////
	Base::handleSelfMsg(msg);


	/////////////////////////////////////////////////////////////////
	// Calculo de prioridad
	/////////////////////////////////////////////////////////////////

	// Calculando tiempo en espera
	if(axis_speed <= 0.05)
		idling_time += ping_interval.dbl();

	last_individual_priority = individual_priority;
	calculateIndividualPriority();

	if(tokens_list.size() == 0)
		getTokens();

	EV << ">>> Tokens and priority\n";
	EV << "    last_priority = " << last_individual_priority << "\n";
	EV << "    priority = " << individual_priority << "\n";
	EV << "    global_priority = " << sum_priority << "\n";

	EV << ">>> Need token:\n";
	for(int t : tokens_list)
		EV << "    Token: " << t << "\n";

	EV << ">>> Reserved tokens:\n";
	for(int t : tokens_list)
	{
		EV << "    Token " << t << " " << token_vector[t].position_token << " reserved by:";
		for(int a : token_vector[t].car_id_reserve)
			EV << " car " << a;
		
		EV << "\n";
	}

	EV << ">>> Blocked tokens:\n";
	for(int t : tokens_list)
	{
		EV << "    Token " << t << " blocked by " << token_vector[t].car_id_block << "\n";
	}


	/////////////////////////////////////////////////////////////////
	// Preparar mensaje
	/////////////////////////////////////////////////////////////////
	info_message = prepareNIM("data", beaconLengthBits, type_CCH, beaconPriority, -1, -1);

	vehicleData data;
	prepareMsgData(data, 0);


	if(msg == sharedDataZoneMessage)
	{
		info_message->setData(data);
		sendWSM((WaveShortMessage*) info_message->dup());
		scheduleAt(simTime() + ping_interval, self_beacon);

		return;
	}


	/////////////////////////////////////////////////////////////////
	// Verificacion de bloqueo de tokens
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

			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("blue"));

			tokensUsed();
			prepareMsgData(data, 3);

		}
		// Vehiculo aun no llega a la interseccion o ya salio
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
			// Vehiculo esta entrando a la interseccion.
			else
			{
				// Comparacion de prioridades.
				bool can_block_token = true;
				bool alredy_blocked = true;
				for(int t : tokens_list)
				{
					if(token_vector[t].car_id_block != myId)
						alredy_blocked = false;

					if(token_vector[t].car_id_block != -1)
						can_block_token = false;

					if(!token_vector[t].car_id_reserve.empty())
						can_block_token = false;

				}

				// Detencion/Continuacion segun estado de los tokens.
				if(alredy_blocked)
				{
					traciVehicle->setColor(Veins::TraCIColor(0, 255, 0, 0));
					traciVehicle->setSpeed(-1.0);

					stoped = false;
					stoping = false;

					prepareMsgData(data, 2);

					for(int t : tokens_list)
						token_vector[t].block(myId);

				}
				else
				{
					if(can_block_token)
					{
						traciVehicle->setColor(Veins::TraCIColor(0, 255, 0, 0));
						traciVehicle->setSpeed(-1.0);

						stoped = false;
						stoping = false;

						prepareMsgData(data, 2);

						for(int t : tokens_list)
							token_vector[t].block(myId);

					}
					else
					{
						smartDetention();

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

		// Verificar si recien entro a zona de informacion compartida.
		// TODO: existe un ciclo de simulacion donde no se entrega informacion, hay que arreglarlo -> mensaje extra o booleano extra.
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


void Tokens::onData(WaveShortMessage *wsm)
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

	std::vector<int> sender_tokens = token_table[sender_in][sender_out];


	/////////////////////////////////////////////////////////////////
	// Tipos de mensaje
	/////////////////////////////////////////////////////////////////

	// Tipo de mensaje 1: Vehiculo ya salio de la interseccion
	if(tipe == 1)
	{
		EV << ">>> Delete car " << sender << " from table <<<\n";
		carTable[sender_in].erase(sender);

		// TODO: cambiar lo siguiente para tipe == 2
		for(int t : sender_tokens)
		{
			if(token_vector[t].car_id_block == sender)
				token_vector[t].unblock();

			token_vector[t].release(sender);
		}

		return ;
	}

	// Tipo de mensaje 2: vehiculo bloquea tokens
	if(tipe == 2)
	{
		for(int t : sender_tokens)
		{
			if(token_vector[t].car_id_block != sender && token_vector[t].car_id_block != -1)
			{
				EV << ">>> Parece haber un conflicto\n";
				EV << "    Vehiculo " << sender << " esta bloqueando token " << t << "\n";
				EV << "    Pero vehiculo " << token_vector[t].car_id_block << " ya lo tiene bloqueado\n";

				int id_conflict_car = token_vector[t].car_id_block;
				int direction_conflict_car = -1;
				double priority_conflict_car = 0.0;
				double distance_conflict_car = 0.0;

				for(int i=0; i<4; ++i)
				{
					if(carTable[i].count(id_conflict_car))
					{
						direction_conflict_car = carTable[i][id_conflict_car].direction_junction;
						priority_conflict_car = carTable[i][id_conflict_car].priority;
						distance_conflict_car = carTable[i][id_conflict_car].distance_to_junction;
					}
				}

				bool change_block = false;
				if(direction_conflict_car == data.direction_junction)
				{
					if(sender_dist < distance_conflict_car)
						change_block = true;
				}
				else
				{
					if(data.priority > priority_conflict_car || (data.priority == priority_conflict_car && sender > id_conflict_car))
						change_block = true;
				}

				if(change_block)
				{
					token_vector[t].block(sender);
					token_vector[t].release(sender);
				}

			}
			else
			{
				token_vector[t].block(sender);
				token_vector[t].release(sender);
			}
		}
	}

	// Tipo de mensaje 3: Vehiculo esta cruzando
	if(tipe == 3)
	{
		int id_token_use = data.id_token_in_use;
		for(int i=0; i<id_token_use; i++)
		{
			int t = sender_tokens[i];
			if(token_vector[t].car_id_block == sender)
			{
				EV << ">>> Token used, unblock token " << t << "\n";
				token_vector[t].unblock();
			}
		}

		for(int i=id_token_use; i < sender_tokens.size(); i++)
		{
			int t = sender_tokens[i];
			token_vector[t].block(sender);
		}
	}
		
	double added_priority = data.priority - carTable[sender_in][sender].priority;

	detectColision(data);
	carTable[sender_in][sender] = data;

	if(tipe == 2 || tipe == 3)
		return ;
	

	EV << ">>> Sender data <<<\n";
	EV << "    time to junction: " << data.time_to_junction << "\n";
	EV << "    distance to junction: " << sender_dist << "\n";

	EV << ">>> Tokens y prioridad\n";

	for(int t : sender_tokens)
		EV << "    Sender will use token: " << t << "\n";

	EV << "    sender priority = " << data.priority << "\n";
	EV << "    sender last_priority = " << data.last_priority << "\n";

	
	/////////////////////////////////////////////////////////////////
	// Comparacion de prioridades y reserva de tokens.
	/////////////////////////////////////////////////////////////////

	// Auto proviene de la misma pista y esta mas lejos de la interseccion -> Se sumam las prioridades
	if(sender_in == direction_junction && data.distance_to_junction > distance_to_junction)
	{
		EV << ">>> Same lane with major distance -> adding priority\n";
		sum_priority += added_priority;
	}
	// Auto proviene de otra pista/calle o esta mas serca de la interseccion -> Comparacion de prioridades
	else
	{
		EV << ">>> It is a rival -> comparing priorities\n";
		if(data.priority < individual_priority || (data.priority == individual_priority && myId > sender))
		{
			// Otro auto tiene menor prioridad, hay que eliminarlo del registro de tokens reservados
			EV << ">>> Minor priority -> deleting reserves\n";
			for(int t : sender_tokens)
			{
				token_vector[t].release(sender);
			}
		}
		
		if(data.priority > individual_priority || (data.priority == individual_priority && myId < sender))
		{
			// Otro auto tiene mayor prioridad, hay que recordar tokens que usara.
			EV << ">>> Major priority -> adding to reserves\n";
			for(int t : sender_tokens)
			{
				token_vector[t].reserve(sender);
			}
		}
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

	data.last_priority = last_individual_priority;
	data.priority = individual_priority;
	
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
	double d = std::max(0.0, shared_data_radio - distance_to_junction);
	individual_priority = k_distance * d + k_velocity * axis_speed + k_idle_time * idling_time + k_cars * sum_priority;
}


/**
 * Determina cual token se utilizara, dependiendo de la posicion del vehiculo
 */
void Tokens::tokensUsed()
{
	EV << ">>> Comparando posicion vehiculo y posicion tokens\n";
	for(int i=id_token_in_use; i<tokens_list.size(); i++)
	{
		bool right = (token_vector[tokens_list[i]].position_token.x < position.x);
		bool down = (token_vector[tokens_list[i]].position_token.y < position.y);
		bool left = (token_vector[tokens_list[i]].position_token.x > position.x);
		bool up = (token_vector[tokens_list[i]].position_token.y > position.y);

		EV << ">>> Comparando posicion respecto a token " << tokens_list[i] << "...\n";

		bool verificacion1 = (direction_junction == 0 && down) || (direction_junction == 1 && left) || (direction_junction == 2 && up) || (direction_junction == 3 && right);
		bool verificacion2 = (direction_out == 0 && down) || (direction_out == 1 && left) || (direction_out == 2 && up) || (direction_out == 3 && right);

		if(!verificacion1 || !verificacion2)
		{
			EV << "    Token " << tokens_list[i] << " aun en uso\n";
			id_token_in_use = i;
			break;
		}

		EV << "    Vehiculo paso token " << tokens_list[i] << "\n";
		if(token_vector[tokens_list[i]].car_id_block == myId)
		{
			EV << "    Desbloqueando token " << tokens_list[i] << "de la lista\n";
			token_vector[tokens_list[i]].unblock();
		}

	}
}