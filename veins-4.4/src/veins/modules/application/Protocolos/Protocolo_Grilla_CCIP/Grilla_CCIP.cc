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
	Base::getBasicParameters();
	

	/////////////////////////////////////////////////////////////////
	// Calculo de prioridad
	/////////////////////////////////////////////////////////////////
	if(!in_block_area)
		priority = simTime().dbl() + time_to_junction;

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
			priority = simTime().dbl();

		in_block_area = true;

		EV << ">>> Zona de reparto y bloqueo de celdas\n";
		double dist_x = std::abs(position.x - traci->junction("1").getPosition().x);
		double dist_y = std::abs(position.y - traci->junction("1").getPosition().y);

		// Vehiculo esta dentro de la interseccion
		if(startId == "1" && dist_x <= 11.4 && dist_y <= 11.4)
		{
			EV << ">>> Zona de cruce\n";
			crossing = true;

			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("blue"));

			//cellsUsed();
			prepareMsgData(data, 3);

		}
		// Vehiculo aun no llega a la interseccion o ya salio
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

				return;
			}
			// vehiculo entrando a la interseccion.
			else
			{
				// Comparacion de prioridades.
				bool can_block_cell = true;
				bool alredy_blocked = true;
				for(int t : cell_list)
				{
					EV << ">>> Cell " << t << ":\n";
					EV << "    Block by: " << cell_register[t].car_id_block << "\n";
					EV << "    Reserve by: ";
					for(int c : cell_register[t].car_id_reserve)
						EV << c << " ";

					EV << "\n";

					if(cell_register[t].car_id_block != myId)
					{
						alredy_blocked = false;
						
						if(cell_register[t].car_id_block != -1)
							can_block_cell = false;

						if(!cell_register[t].car_id_reserve.empty())
							can_block_cell = false;
					}
				}

				// Este vehiculo ya bloqueo las celdas
				if(alredy_blocked)
				{
					EV << ">>> I blocked cell\n";
					traciVehicle->setColor(Veins::TraCIColor(0, 255, 0, 0));
					traciVehicle->setSpeed(-1.0);

					stoped = false;
					stoping = false;

					prepareMsgData(data, 2);

					for(int t : cell_list)
						cell_register[t].block(myId);

				}
				else
				{
					// Vehiculo no bloqueo las celdas, pero tiene prioridad para hacerlo.
					if(can_block_cell)
					{
						EV << ">>> Can block cells\n";
						traciVehicle->setColor(Veins::TraCIColor(0, 255, 0, 0));
						traciVehicle->setSpeed(-1.0);

						stoped = false;
						stoping = false;

						prepareMsgData(data, 2);

						for(int t : cell_list)
							cell_register[t].block(myId);

					}
					// Vehiculo no tiene prioridad para reservar celdas, tiene que esperar
					else
					{
						EV << ">>> Cant block cells, stoping car\n";
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


	/////////////////////////////////////////////////////////////////
	// Tipos de mensaje
	/////////////////////////////////////////////////////////////////


	// Tipo de mensaje 1: Vehiculo ya salio de la interseccion
	if(tipe == 1)
	{
		EV << ">>> Delete car " << sender << " from table <<<\n";
		carTable[sender_in].erase(sender);

		// TODO: cambiar lo siguiente para tipe == 2
		for(int t : sender_cells)
		{
			if(cell_register[t].car_id_block == sender)
				cell_register[t].unblock();

			cell_register[t].release(sender);
		}

		return ;
	}

	// Tipo de mensaje 2: vehiculo bloquea celdas
	if(tipe == 2)
	{
		for(int t : sender_cells)
		{
			if(cell_register[t].car_id_block != sender && cell_register[t].car_id_block != -1)
			{
				EV << ">>> Parece haber un conflicto\n";
				EV << "    Vehiculo " << sender << " esta bloqueando celda " << t << "\n";
				EV << "    Pero vehiculo " << cell_register[t].car_id_block << " ya lo tiene bloqueado\n";

				// Este conflicto no puede ser debido a un cambio en la prioridad de uno de los vehiculos
				// Hay dos opciones: 1) ambos vehiculos bloquearon al mismo tiempo o 2) alguien esta bloqueando de forma forzada.

				int id_conflict_car = cell_register[t].car_id_block;
				int direction_conflict_car = -1;
				double priority_conflict_car = 0.0;
				double distance_conflict_car = 0.0;

				EV << ">>> Obteniendo datos vehiculo " << id_conflict_car << "\n";
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

				// Si ambos vehiculos tienen la misma direccion inicial, bloque el vehiculo mas cercano.
				if(direction_conflict_car == data.direction_junction)
				{
					if(sender_dist < distance_conflict_car)
						change_block = true;
				}
				// En caso contrario, bloque el vehiculo de mejor prioridad.
				else
				{
					if(data.priority < priority_conflict_car || (data.priority == priority_conflict_car && sender > id_conflict_car))
						change_block = true;
				}

				// En caso que el auto que envia mensaje es el que debe bloquear.
				if(change_block)
				{
					cell_register[t].block(sender);
					cell_register[t].release(sender);
				}

			}
			else
			{
				cell_register[t].block(sender);
				cell_register[t].release(sender);
			}
		}
	}


	// Tipo de mensaje 3: Vehiculo esta cruzando
	if(tipe == 3)
	{
		for(int i=0; i < sender_cells.size(); i++)
		{
			int t = sender_cells[i];
			cell_register[t].block(sender);
		}
	}
		
	detectColision(data);
	carTable[sender_in][sender] = data;


	if(tipe == 2 || tipe == 3)
		return ;
	

	EV << ">>> Sender data <<<\n";
	EV << "    time to junction: " << data.time_to_junction << "\n";
	EV << "    distance to junction: " << sender_dist << "\n";

	EV << ">>> Celdas y prioridad\n";

	for(int t : sender_cells)
		EV << "    Sender will use celda: " << t << "\n";

	EV << "    sender priority = " << data.priority << "\n";
	EV << "    sender last_priority = " << data.last_priority << "\n";

	
	/////////////////////////////////////////////////////////////////
	// Comparacion de prioridades y reserva de celdas.
	/////////////////////////////////////////////////////////////////


	// Auto proviene de otra pista/calle o esta mas serca de la interseccion -> Comparacion de prioridades
	if(sender_in != direction_junction)// || data.distance_to_junction < distance_to_junction)
	{
		EV << ">>> It is a rival -> comparing priorities\n";
		if(data.priority > priority || (data.priority == priority && myId > sender))
		{
			// Otro auto tiene menor prioridad, hay que eliminarlo del registro de celdas reservados
			EV << ">>> Minor priority -> deleting reserves\n";
			for(int t : sender_cells)
			{
				cell_register[t].release(sender);
			}
		}
		
		if(data.priority < priority || (data.priority == priority && myId < sender))
		{
			// Otro auto tiene mayor prioridad, hay que recordar celdas que usara.
			EV << ">>> Major priority -> adding to reserves\n";
			for(int t : sender_cells)
			{
				cell_register[t].reserve(sender);
			}
		}
	}
	else
	{
		if(data.distance_to_junction < distance_to_junction)
		{
			EV << ">>> Car is more close to junction -> adding to reserves\n";
			for(int t : sender_cells)
			{
				cell_register[t].reserve(sender);
			}
		}
		else
			EV << ">>> Info not important\n";
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
