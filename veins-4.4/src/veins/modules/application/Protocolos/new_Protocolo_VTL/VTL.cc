/*
Implementacion de protocolo de semaforos virtuales (new_VTL).
*/

#include <veins/modules/application/Protocolos/new_Protocolo_VTL/VTL.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <cstdlib>
#include <algorithm>


Define_Module(new_VTL);

void new_VTL::initialize(int stage)
{
    Base::initialize(stage);

    switch (stage)
    {
    case 0:
		is_lider = false;

		first_query = false;
		block_movement = false;

		last_lider = -1;

		direction_to_left = false;

		stop_time = -1.0;

		crossing_left = false;

		liders_list = std::vector<int>(4, -1);
		vehicles_to_wait = std::set<int>();

		tiempo_semaforo = par("tiempo_semaforo").doubleValue();

		shared_data_radio = par("shared_data_radio").doubleValue();
		lider_selection_radio = par("lider_selection_radio").doubleValue();
		
        break;
    default:
        break;
    }
}

void new_VTL::finish()
{
    Base::finish();
}


void new_VTL::handleSelfMsg(cMessage *msg){
	/////////////////////////////////////////////////////////////////
	// Obtencion de datos basicos.
	/////////////////////////////////////////////////////////////////
	Base::handleSelfMsg(msg);

	// Determinar si este auto dobla hacia la izquierda.
	direction_to_left = isGoingLeft();

	EV << "> stop_time: " << stop_time << "\n";
	EV << "> actual_time: " << simTime().dbl() << "\n";
	EV << "> lideres:\n";
	// Vehiculo verifica si existe algun lider
	for(int i=0; i<4; i++)
		EV << "  direccion " << i << ": " << liders_list[i] <<"\n";
			


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
	// Espera de vehiculo que faltan por salir de interseccion
	/////////////////////////////////////////////////////////////////
	if(is_lider && lider_extra_waiting)
	{
		EV << ">>> Tiempo de espera extra: busqueda de proximo lider\n";
		if(next_lider == -1)
		{
			searchNextLider();

			if(next_lider != -1)
			{
				prepareMsgData(data, 3);

				info_message->setData(data);
				sendWSM((WaveShortMessage*) info_message->dup());

				scheduleAt(simTime() + ping_interval, self_beacon);

				return ;
			}
		}

		// Vehiculo revisa si aun existen vehiculos a los que esperar
		if(vehicles_to_wait.empty())
		{
			EV << "> Lider no tiene que esperar vehiculos\n";

			// Eliminamos status de lider
			is_lider = false;
			liders_list[direction_junction] = -1;

			Base::continueTravel();

			// Mensaje fin tiempo de lider
			prepareMsgData(data, 2);

			info_message->setData(data);
			sendWSM((WaveShortMessage*) info_message->dup());

			scheduleAt(simTime() + ping_interval, self_beacon);

			return ;
		}
		
	}


	/////////////////////////////////////////////////////////////////
	// Fin de tiempo de semaforo y verde rapida
	/////////////////////////////////////////////////////////////////
	if(is_lider)
	{
		// Termino de semaforo
		bool end_time = ((simTime().dbl() - stop_time.dbl()) >= tiempo_semaforo);

		// Verde rapida 
		// TODO: adaptar a solo vehiculos en zona de colision
		bool no_more_cars = (carTable[(direction_junction + 1) % 4].empty() && carTable[(direction_junction + 3) % 4].empty());

		EV << ">>> Analizando termino de liderazgo\n";

		// Termino de liderazgo
		if(end_time || no_more_cars)
		{
			EV << "> Fin de tiempo de lider\n";

			EV << ">>> Busqueda de proximo lider\n";
			searchNextLider();

			// No existen vehiculos a esperar -> lider puede continuar con viaje
			if(vehicles_to_wait.empty())
			{
				EV << "> Lider no tiene que esperar vehiculos\n";

				// Eliminamos status de lider
				is_lider = false;
				liders_list[direction_junction] = -1;

				Base::continueTravel();

				// Mensaje fin tiempo de lider
				prepareMsgData(data, 2);

			}
			else
			{
				EV << "> Lider tiene que esperar vehiculos\n";

				// Agregar status de espera extra
				lider_extra_waiting = true;

				// Mensaje notificacion de inicio de tiempo extra de lider
				if(next_lider != -1)
					prepareMsgData(data, 3);
				else
					prepareMsgData(data, 0);
			}
			
			info_message->setData(data);
			sendWSM((WaveShortMessage*) info_message->dup());

			scheduleAt(simTime() + ping_interval, self_beacon);

			return ;
		}
		else
		{
			double delta_time = tiempo_semaforo - (simTime().dbl() - stop_time.dbl());
			EV << "> Aun existen vehiculos, tiempo restante de semaforo: " << delta_time << "\n";
		}
	}


	/////////////////////////////////////////////////////////////////
	// Area de seleccion de lider.
	/////////////////////////////////////////////////////////////////
	if(distance_to_junction <= lider_selection_radio)
	{
		EV << ">>> Zona de colision <<<\n";

		// Posicion relativa al centro de la interseccion
		double dist_x = std::abs(position.x - traci->junction("1").getPosition().x);
		double dist_y = std::abs(position.y - traci->junction("1").getPosition().y);

		// Vehiculo se encuentra dentro de la interseccion
		if(startId == "1" && dist_x <= 11.4 && dist_y <= 11.4)
		{
			EV << ">>> Zona de cruce <<<\n";

			// Estados de vehiculo
			crossing = true;

			// Registrar entrada en interseccion
			Base::registerInOfJunction();

			// Mensaje broadcast
			prepareMsgData(data, 0);
			info_message->setData(data);
			sendWSM((WaveShortMessage*) info_message->dup());

			scheduleAt(simTime() + ping_interval, self_beacon);

			return ;

		}
		// Vehiculo salio de la interseccion
		else if(crossing)
		{
			EV << ">>> Fuera de interseccion <<<\n";

			// Estados de vehiculo
			crossing = false;
			outJunction = true;

			// Registrar salida en interseccion
			Base::registerOutOfJunction();
			
			// Mensaje de salida de vehiculo
			prepareMsgData(data, 1);
			info_message->setData(data);
			sendWSM((WaveShortMessage*) info_message->dup());

			// Removemos de forma temprana el vehiculo
			Base::removeVehicle(0);

			// Vehiculo no vuelve a mandar mensajes
			return;
		}

		// Vehiculo verifica si existe algun lider
		bool exist_lider = false;
		for(int id : liders_list)
			if(id != -1)
				exist_lider = true;

		// Si no existe lider, verificamos que vehiculo no colisiona
		if(!exist_lider)
		{
			EV << ">>> No existe lider, calculando si debo ser lider\n";

			// Reviso si hay vehiculos de la otra calle en la zona de colision
			bool vehicle_other_street = false;
			for(int i=1; i<4; i+=2)
			{
				int d = (direction_junction + i) % 4;
				for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
				{
					vehicle_other_street = vehicle_other_street || (it->second.distance_to_junction < lider_selection_radio);
				}
			}

			// Si vehiculo puede llegar a colisionar, y tarda mas en llegar a interseccion, se vuelve lider
			if(vehicle_other_street && !first_query)
			{
				EV << "> Existen vehiculo en la otra calle, puede haber colision => Tomo liderazgo\n";

				Base::detention();

				// Asignacion de parametros de lider
				is_lider = true;
				liders_list[direction_junction] = myId;
				stop_time = simTime().dbl();

				// Solo se realiza una query de colision
				first_query = true;

				// Se manda mensaje especial para notificar liderazgo
				prepareMsgData(data, 0);
				info_message->setData(data);
				sendWSM((WaveShortMessage*) info_message->dup());

				scheduleAt(simTime() + ping_interval, self_beacon);

				return;
			}
		}

		// Solo se realiza una query de colision
		first_query = true;

		// Detencion de lider
		if(is_lider)
		{
			EV << "> Detencion de lider\n";

			Base::detention();
		}
		// Detencion de primer vehiculo en pista contraria a un lider, en caso de haber lider
		else
		{
			EV << ">>> Confirmando lider en otra pista\n";
			int l = liders_list[(direction_junction + 2) % 4];

			if(l != -1 && !block_movement)
			{
				EV << "> Existe lider en la otra pista, hay que detenerse\n";
				Base::detention();
			}
			else
			{
				if(stoping || stoped)
					Base::continueTravel();
			}
			
		}
	}


	/////////////////////////////////////////////////////////////////
	// Area de envio de infomacion
	/////////////////////////////////////////////////////////////////
	if(distance_to_junction <= shared_data_radio)
	{
		EV << ">>> Zona de informacion <<<\n";

		// Mensaje broadcast
		prepareMsgData(data, 0);
		info_message->setData(data);
		sendWSM((WaveShortMessage*) info_message->dup());

		// Verificar si recien entro a zona de informacion compartida.
		if(!inSharedDataZone)
		{
			EV << ">>> Nuevo vehiculo en zona de informacion, realizando ciclo de espera...\n";
			inSharedDataZone = true;
			scheduleAt(simTime() + ping_interval, sharedDataZoneMessage);
		} 
		else
			scheduleAt(simTime() + ping_interval, self_beacon);

		return;

	}

	scheduleAt(simTime() + ping_interval, self_beacon);

}

void new_VTL::onData(WaveShortMessage *wsm)
{
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


	EV << ">>> Data message <<<\n";
	NodeInfoMessage* msg = check_and_cast<NodeInfoMessage*>(wsm);
	vehicleData data = msg->getData();

	int tipe = data.msg_type;
	int sender = wsm->getSenderAddress();
	//int recipient = wsm->getRecipientAddress();

	int sender_in = data.direction_junction;
	int sender_out = data.direction_out;

	bool sender_lider = data.isLider;
	double sender_dist = data.distance_to_junction;
	double sender_stop_time = data.stop_time;


	EV << "    tipe: " << tipe << "\n";
	EV << "    send: " << sender << "\n";
	EV << "    time: " << data.time_to_junction << "\n";

	EV << "    sender_in: " << sender_in << "\n";
	EV << "    sender_out: " << sender_out << "\n";
	EV << "    sender_stop_time: " << sender_stop_time << "\n";


	// Auto ha salido de la interseccion
	if(tipe == 1)
	{
		EV << ">>> Delete car " << sender << " from table <<<\n";

		carTable[data.direction_junction].erase(sender);

		// Eliminar elemento en caso que lider se encuentre en tiempo de espera extra
		if(is_lider && lider_extra_waiting)
			vehicles_to_wait.erase(sender);

		return ;
	}

	// Termino tiempo de lider
	if(tipe == 2)
	{
		EV << ">>> Lider en direccion " << sender_in << " termino como semaforo\n";

		liders_list[sender_in] = -1;
	}

	// Lider actual selecciona su heredero
	if(tipe == 3)
	{
		// Lider entrega identificador del siguiente lider
		int id_next_lider = data.next_lider;
		EV << ">>> Lider en tiempo de espera extra, vehiculo " << id_next_lider << " es el siguiente lider\n";

		// Busqueda de direccion de siguiente lider
		int dir_next_lider = -1;
		for(int i=0; i<4; i++)
		{
			if(carTable[i].count(id_next_lider) != 0)
				dir_next_lider = -1;
		}

		// Memorizar siguiente lider
		if(dir_next_lider != -1)
		{
			liders_list[dir_next_lider] = id_next_lider
		}

		if(myId == id_next_lider)
		{
			is_lider = true;
		}
	}

	carTable[sender_in][sender] = data;

	Base::getBasicParameters();

	// En caso de tener un lider incorrecto
	if(!data.isLider && liders_list[sender_in] == sender)
		liders_list[sender_in] = -1;

	// Reaccion a mensaje de un vehiculo lider
	if(data.isLider && !crossing)
	{
		EV << ">>> Existe lider en la interseccion\n";

		// Lider esta en la misma pista
		if(sender_in == direction_junction)
		{
			EV << ">>> Mensaje de lider en mi pista\n";

			// Verificar si creo ser lider tambien
			if(is_lider)
			{
				EV << "> Existe conflicto de liderazgo\n";

				// Elegir el lider mas cercano
				if(sender_dist < distance_to_junction)
				{
					EV << "> Soy el falso lider\n";

					// Reseteando parametros de lider
					is_lider = false;
					last_lider = -1;
					liders_list[sender_in] = sender;

					last_lider = -1;
					stop_time = -1.0;

					if(stoping || stoped)
						Base::continueTravel();
				}
			}
			else
				liders_list[sender_in] = sender;

		}
		// Lider esta en la misma calle
		else if(sender_in % 2 == direction_junction)
		{
			EV << ">>> Mensaje de lider en otra pista\n";

			if(is_lider)
			{
				EV << "> Existe conflicto de liderazgo\n";

				// Elegir el que lleve mas tiempo como lider
				if(sender_dist < distance_to_junction)
				{
					EV << "> Soy el falso lider\n";

					// Reseteando parametros de lider
					is_lider = false;
					last_lider = -1;
					liders_list[sender_in] = sender;
					liders_list[direction_junction] = -1;

					last_lider = -1;
					stop_time = -1.0;

					if(stoping || stoped)
						Base::continueTravel();
				}
			}
			else
				liders_list[sender_in] = sender;
				
		}
		// Lider esta en la otra calle
		else
		{
			EV << ">>> Mensaje de lider en otra calle\n";
			// Si hay conflicto durante revision de colision, elegir como lider el vehiculo con menos vehiculos delante de el
			if(is_lider)
			{
				if(!(data.last_lider == myId || sender == last_lider))
				{
					EV << "> Liderazgo no fue heredado\n";
					if(stop_time > sender_stop_time)
					{
						is_lider = false;
						liders_list[sender_in] = sender;
						liders_list[direction_junction] = -1;
						stop_time = -1.0;

						if(stoping || stoped)
							Base::continueTravel();
					}
				}
				else
					liders_list[sender_in] = sender;

			}
			else
				liders_list[sender_in] = sender;
			
		}
	}
}

void new_VTL::onBeacon(WaveShortMessage *wsm)
{
}


/**
 * Funcion que prepara contenido de mensaje enviado por los auto.
 */
void new_VTL::prepareMsgData(vehicleData& data, int msgTipe)
{
	Base::prepareMsgData(data, msgTipe);

	data.isLider = is_lider;

	data.last_lider = last_lider;
	data.next_lider = next_lider;

	data.stop_time = stop_time.dbl();
	data.goingLeft = direction_to_left;
}


/**
 * Determina la direccion final de un auto
 */
bool new_VTL::isGoingLeft()
{
	bool left = ((direction_junction + 3) % 4 == direction_out);
	EV << "    arrival direction: " << direction_out << "\n";

	if(left) 
	{
		left = true;
		traciVehicle->setColor(Veins::TraCIColor::fromTkColor("orange"));
	}
	
	return left;
}


/**
 * 
 */
void new_VTL::searchNextLider()
{
	// Buscamos proximo lider
	next_lider = -1;
	double best_dist_to_junction = 1e8;

	for(int i=1; i<4; i+=2)
	{
		int d = (direction_junction + i) % 4;
		for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
		{
			double dist = it->second.distance_to_junction;
			if(dist < best_dist_to_junction && dist > lider_selection_radio)
			{
				next_lider = it->first;
				best_dist_to_junction = dist;
			}

			// Vehiculo con permiso para salir de interseccion
			if(dist <= lider_selection_radio)
			{
				EV << "> Esperando a vehiculo " << it->first << "\n";
				vehicles_to_wait.insert(it->first);
			}
		}
	}

	EV << "> Proximo lider: " << next_lider << "\n";
}