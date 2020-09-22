/*
Implementacion de protocolo de semaforos virtuales (new_VTL).
*/

#include <veins/modules/application/Protocolos/new_Protocolo_VTL/VTL.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <stdlib.h>
#include <cstdlib>
#include <algorithm>


Define_Module(new_VTL);

void new_VTL::initialize(int stage)
{
    Base::initialize(stage);

    switch (stage)
    {
    case 0:
		// Identificador de lider y sublider
		is_lider = false;
		is_sub_lider = false;
		sub_lider_id = -1;

		// Flag de vehiculo al entrar en zona de reparto de informacion
		first_query = false;

		// Flag de vehiculo al entrar en zona de seleccion de lider
		first_msg = false;

		// Identificador de lider siguiente o predecesor
		last_lider = -1;
		next_lider = -1;

		// Tiempo de simulacion cuando vehiculo toma liderazgo
		stop_time = -1.0;

		// Determina si vehiculo dobla a la izquierda
		direction_to_left = false;

		// Registro de lideres
		liders_list = std::vector<int>(4, -1);

		// Vehiculos a esperar en tiempo de espera extra
		vehicles_to_wait = std::set<int>();

		// Hiperparametros
		tiempo_semaforo = par("tiempo_semaforo").doubleValue();
		shared_data_radio = par("shared_data_radio").doubleValue();
		lider_selection_radio = par("lider_selection_radio").doubleValue();

		// Cantidad de intervalos entre mensajes de vehiculo
		intervals_per_selfmsg = 2;
		intervals_counting = 0;//rand() % intervals_per_selfsmg;
		
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

	if(outJunction)
	{
		Base::removeVehicle(0);
		return;
	}

	intervals_counting++;

	// Determinar si este auto dobla hacia la izquierda.
	direction_to_left = isGoingLeft();

	EV << ">>> is lider: " << is_lider << "\n";
	EV << "    is sublider: " << is_sub_lider << "\n";
	EV << "    stop_time: " << stop_time << "\n";
	EV << "    actual_time: " << simTime().dbl() << "\n";
	EV << "    Lider siguiente: " << next_lider << "\n";
	EV << "    Lider anterior: " << last_lider << "\n";

	// Vehiculo verifica si existe algun lider
	EV << "    Lideres:\n";
	for(int i=0; i<4; i++)
		EV << "    > direccion " << i << ": " << liders_list[i] <<"\n";

	for(int i=0; i<4; i++)
	{
		EV << "    Vehiculos en direccion " << i << ":";
		for(auto it=carTable[i].begin(); it!=carTable[i].end(); it++)
		{
			EV << " " << it->first;
		}
		EV << "\n";
	}

	if(is_lider)
		traciVehicle->setColor(Veins::TraCIColor::fromTkColor("red"));

	if(is_sub_lider)
		traciVehicle->setColor(Veins::TraCIColor::fromTkColor("orange"));


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
	// Vehiculo lider busca un sublider
	/////////////////////////////////////////////////////////////////
	if(is_lider && sub_lider_id == -1)
	{
		EV << "    No hay sublider, buscando...\n";

		int d = (direction_junction + 2) % 4;
		int id_best = -1;
		double dist_best = 1e8;

		for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
		{
			if(it->second.distance_to_junction < dist_best && it->second.distance_to_junction > lider_selection_radio)
			{
				id_best = it->first;
				dist_best = it->second.distance_to_junction;
			}
		}

		// Se encontro sublider
		if(id_best != -1 && sub_lider_id == -1)
		{
			EV << "    Sublider encontrado: " << sub_lider_id << "\n";

			sub_lider_id = id_best;

			prepareMsgData(data, 0);
		}
	}


	/////////////////////////////////////////////////////////////////
	// Espera de vehiculo que faltan por salir de interseccion
	/////////////////////////////////////////////////////////////////
	if(is_lider && lider_extra_waiting)
	{
		EV << ">>> Tiempo de espera extra: busqueda de proximo lider\n";
		if(distance_to_junction < lider_selection_radio)
			Base::detention();

		if(next_lider == -1)
		{
			EV << "    Buscando siguiente lider\n";
			// Se busca siguiente lider y lider de referencia
			searchNextLider();

			if(next_lider != -1)
			{
				EV << "    Siguiente lider encontrado: " << next_lider << "\n";

				// Mensaje para notificar lider heredero
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
			EV << "    Lider no tiene que esperar mas vehiculos\n";
			EV << "    Soltando liderazgo\n";

			// Eliminamos status de lider
			is_lider = false;
			liders_list[direction_junction] = -1;

			Base::continueTravel();
			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("green"));

			// Mensaje fin tiempo de lider
			prepareMsgData(data, 2);

			info_message->setData(data);
			sendWSM((WaveShortMessage*) info_message->dup());

			scheduleAt(simTime() + ping_interval, self_beacon);

			return ;
		}
		else
		{
			EV << "    Lider espera salida de vehiculos:\n";
			for(int id : vehicles_to_wait)
				EV << "    > " << id << "\n";
		}
	}


	/////////////////////////////////////////////////////////////////
	// Fin de tiempo de semaforo y verde rapida
	/////////////////////////////////////////////////////////////////
	if(is_lider && !lider_extra_waiting)
	{
		EV << ">>> Lider en espera\n";
		if(distance_to_junction < lider_selection_radio)
			Base::detention();

		// Solo considerar lideres que no hereden, o que heredan de un lider que ya se fue
		int d = (direction_junction + 1) % 4;
		
		// En caso que lider heredero y lider original estan al mismo tiempo
		if(last_lider != -1 && (last_lider == liders_list[d] || last_lider == liders_list[(d+2) % 4]))
		{
			EV << "    Vehiculo aun tiene que esperar que lider anterior se retire\n";
			EV << "    No se puede analizar termino de liderazgo\n";
			stop_time = simTime().dbl();

		}
		else
		{
			EV << ">>> Analizando termino de liderazgo\n";

			// Termino de semaforo
			bool end_time = ((simTime().dbl() - stop_time.dbl()) >= tiempo_semaforo);

			// Verde rapida 
			// TODO: adaptar a solo vehiculos en zona de colision
			bool no_more_cars = (carTable[(direction_junction + 1) % 4].empty() && carTable[(direction_junction + 3) % 4].empty());

			// Termino de liderazgo
			if(end_time || no_more_cars)
			{
				EV << "    Fin de tiempo de lider\n";

				EV << ">>> Busqueda de proximo lider\n";
				searchNextLider();

				EV << "    Entrando en fase de tiempo extra\n";
				lider_extra_waiting = true;

				if(next_lider != -1)
					prepareMsgData(data, 3);
				else
					prepareMsgData(data, 0);
				
				info_message->setData(data);
				sendWSM((WaveShortMessage*) info_message->dup());

				scheduleAt(simTime() + ping_interval, self_beacon);

				return ;
			}
			else
			{
				double delta_time = tiempo_semaforo - (simTime().dbl() - stop_time.dbl());
				EV << "    Aun existen vehiculos, tiempo restante de semaforo: " << delta_time << "\n";
			}
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

			// Iniciando estado de cruce
			crossing = true;

			// Registrar entrada en interseccion
			Base::registerInOfJunction();

			// Mensaje broadcast a vehiculos
			prepareMsgData(data, 0);
			info_message->setData(data);

			if(intervals_counting % intervals_per_selfmsg == 0)
				sendWSM((WaveShortMessage*) info_message->dup());

			scheduleAt(simTime() + ping_interval, self_beacon);

			return ;

		}
		// Vehiculo salio de la interseccion
		else if(crossing)
		{
			EV << ">>> Fuera de interseccion <<<\n";
			Base::registerOutOfJunction();

			// Estados de vehiculo
			crossing = false;
			outJunction = true;
			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("purple"));
			
			// Mensaje de salida de vehiculo
			prepareMsgData(data, 1);
			info_message->setData(data);
			sendWSM((WaveShortMessage*) info_message->dup());

			scheduleAt(simTime() + ping_interval, self_beacon);

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

			// Se reviso si existe colision
			bool vehicle_other_street = false;
			for(int i=1; i<4; i+=2)
			{
				int d = (direction_junction + i) % 4;
				for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
				{
					vehicle_other_street = vehicle_other_street || (it->second.distance_to_junction <= lider_selection_radio);
				}
			}


			// Si vehiculo puede llegar a colisionar, y tarda mas en llegar a interseccion, se vuelve lider
			if(vehicle_other_street && !first_query)
			{
				EV << "    Existen vehiculo en la otra calle, puede haber colision => Tomo liderazgo\n";

				// Detencion de vehiculo
				Base::detention();

				// Solo se realiza una query de colision
				first_query = true;

				// Asignacion de parametros de lider
				is_lider = true;
				liders_list[direction_junction] = myId;
				stop_time = simTime().dbl();

				// Buscar sublider
				int d = (direction_junction + 2) % 4;
				int id_best = -1;
				double dist_best = 1e8;

				for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
				{
					if(it->second.distance_to_junction < dist_best && it->second.distance_to_junction > lider_selection_radio)
					{
						id_best = it->first;
						dist_best = it->second.distance_to_junction;
					}
				}

				if(id_best != -1 && sub_lider_id == -1)
					sub_lider_id = id_best;

				EV << "    Sublider es: " << sub_lider_id << "\n";

				// Se manda mensaje para notificar liderazgo
				prepareMsgData(data, 0);
				info_message->setData(data);
				sendWSM((WaveShortMessage*) info_message->dup());

				scheduleAt(simTime() + ping_interval, self_beacon);

				return;
			}
		}


		// En caso de que no se envie msg de colision, se envia msg para informar entrada en zona de colision
		if(!first_query)
		{
			// Se envia msg al entrar en zona de colision
			first_query = true;

			// Envio de msg
			prepareMsgData(data, 0);
			info_message->setData(data);
			sendWSM((WaveShortMessage*) info_message->dup());

			scheduleAt(simTime() + ping_interval, self_beacon);

			return;
		}
		

		// Detencion de lider y sublider
		if(is_lider || is_sub_lider)
		{
			EV << ">>> Detencion de lider y sublider\n";
			EV << "    sublider: " << sub_lider_id << "\n";

			// Detencion de vehiculo
			Base::detention();

			// En caso de que no hay sublider, lider revisa si hay algun candidato
			if(is_lider && sub_lider_id == -1)
			{
				EV << "    No hay sublider, buscando...\n";

				int d = (direction_junction + 2) % 4;
				int id_best = -1;
				double dist_best = 1e8;

				for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
				{
					if(it->second.distance_to_junction < dist_best && it->second.distance_to_junction > lider_selection_radio)
					{
						id_best = it->first;
						dist_best = it->second.distance_to_junction;
					}
				}

				// Se encontro sublider
				if(id_best != -1 && sub_lider_id == -1)
				{
					EV << "    Sublider encontrado: " << sub_lider_id << "\n";

					sub_lider_id = id_best;

					prepareMsgData(data, 0);
					info_message->setData(data);
					sendWSM((WaveShortMessage*) info_message->dup());

					scheduleAt(simTime() + ping_interval, self_beacon);

					return;
				}
			}
		}
	}


	/////////////////////////////////////////////////////////////////
	// Area de envio de infomacion
	/////////////////////////////////////////////////////////////////
	if(distance_to_junction <= shared_data_radio)
	{
		EV << ">>> Zona de informacion <<<\n";

		// Prepara msg
		prepareMsgData(data, 0);
		info_message->setData(data);

		// Mensaje generico de informacion
		if(!first_msg || intervals_counting % intervals_per_selfmsg == 0 || is_lider)
		{
			sendWSM((WaveShortMessage*) info_message->dup());
			first_msg = true;

			intervals_counting = 0;
		}

		// Verificar si recien entro a zona de informacion compartida. 
		if(!inSharedDataZone)
		{
			EV << ">>> Nuevo vehiculo en zona de informacion, realizando ciclo de espera...\n";
			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("yellow"));
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

	/////////////////////////////////////////////////////////////////
	// Manejo especifico de mensajes.
	/////////////////////////////////////////////////////////////////

	// Auto ha salido de la interseccion
	if(tipe == 1)
	{
		EV << ">>> Eliminar vehiculo " << sender << " de la tabla <<<\n";

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
		if(is_lider && sender == last_lider)
		{
			stop_time = simTime().dbl();
		}

		if(is_sub_lider && direction_junction % 2 == sender_in % 2)
		{
			EV << "    Dejo de ser sublider\n";
			is_sub_lider = false;

			Base::continueTravel();
			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("green"));
		}

		return;
	}


	// Lider actual selecciona su heredero
	if(tipe == 3)
	{
		// Lider entrega identificador del siguiente lider
		int id_next_lider = data.next_lider;
		int id_sub_next_lider = data.sub_next_lider;
		EV << ">>> Lider entrando a tiempo de espera extra, vehiculo " << id_next_lider << " es el siguiente lider\n";
		EV << "    Lider entrando a tiempo de espera extra, vehiculo " << id_sub_next_lider << " es el siguiente sublider\n";

		// Asignamos el siguiente lider
		if(myId == id_next_lider)
		{
			EV << "    Soy heredero del lider actual\n";

			is_lider = true;
			sub_lider_id = id_sub_next_lider;
			liders_list[direction_junction] = id_next_lider;

			last_lider = sender;
			stop_time = simTime().dbl();
		}

		if(myId == id_sub_next_lider)
		{
			EV << "    Soy sublider del lider heredero\n";

			is_sub_lider = true;
		}

		if(id_next_lider != -1)
		{
			for(int i=0; i<4; ++i)
			{
				for(auto it=carTable[i].begin(); it!=carTable[i].end(); it++)
				{
					if(id_next_lider == it->first)
					{
						liders_list[i] = id_next_lider;
						break;
					}
				}
			}
		}
	}


	/////////////////////////////////////////////////////////////////
	// Manejo general de mensajes.
	/////////////////////////////////////////////////////////////////

	// Guardar informacion de vehiculo
	carTable[sender_in][sender] = data;

	// Actualizar datos de vehiculo
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
					
					stop_time = -1.0;

					if(stoping || stoped)
					{
						Base::continueTravel();
						traciVehicle->setColor(Veins::TraCIColor::fromTkColor("purple"));
					}
				}
			}
			else
				liders_list[sender_in] = sender;

		}
		// Lider esta en la misma calle
		else if(sender_in % 2 == direction_junction % 2)
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
					
					stop_time = -1.0;

					if(stoping || stoped)
					{
						Base::continueTravel();
						traciVehicle->setColor(Veins::TraCIColor::fromTkColor("purple"));
					}
				}
			}
			else
				liders_list[sender_in] = sender;

			if(data.sub_lider == myId)
				is_sub_lider = true;
				
		}
		// Lider esta en la otra calle
		else
		{
			EV << ">>> Mensaje de lider en otra calle\n";

			if(is_lider)
			{
				if(!(sender == next_lider || sender == last_lider))
				{
					EV << "> Liderazgo no fue heredado\n";
					if(stop_time.dbl() > sender_stop_time)
					{
						is_lider = false;
						liders_list[sender_in] = sender;
						liders_list[direction_junction] = -1;
						stop_time = -1.0;

						if(stoping || stoped)
						{
							Base::continueTravel();
							traciVehicle->setColor(Veins::TraCIColor::fromTkColor("purple"));
						}
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
	data.sub_lider = sub_lider_id;

	data.last_lider = last_lider;
	data.next_lider = next_lider;
	data.sub_next_lider = sub_next_lider;

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
// TODO: Seleccionar sublider como referencia para elegir los vehiculos que ignoran lideres
void new_VTL::searchNextLider()
{
	// Identificadores y distancias
	int lider1 = -1, lider2 = -1;
	double dist_lider1 = 1e8, dist_lider2 = 1e8;

	// Buscando posible lider en primera direccion
	int d1 = (direction_junction + 1) % 4;
	for(auto it = carTable[d1].begin(); it != carTable[d1].end(); it++)
	{
		double dist = it->second.distance_to_junction;

		// Proximo lider tiene que estar fuera de area de interseccion
		if(dist < dist_lider1 && dist > lider_selection_radio)
		{
			lider1 = it->first;
			dist_lider1 = dist;
		}

		// Vehiculos con permiso para salir de interseccion
		if(dist <= lider_selection_radio)
		{
			EV << "    Esperando a vehiculo " << it->first << "\n";
			vehicles_to_wait.insert(it->first);
		}
	}

	// Buscando posible lider en segunda direccion
	int d2 = (direction_junction + 3) % 4;
	for(auto it = carTable[d2].begin(); it != carTable[d2].end(); it++)
	{
		double dist = it->second.distance_to_junction;

		// Proximo lider tiene que estar fuera de area de interseccion
		if(dist < dist_lider2 && dist > lider_selection_radio)
		{
			lider2 = it->first;
			dist_lider2 = dist;
		}

		// Vehiculos con permiso para salir de interseccion
		if(dist <= lider_selection_radio)
		{
			EV << "    Esperando a vehiculo " << it->first << "\n";
			vehicles_to_wait.insert(it->first);
		}
	}

	if(dist_lider1 > dist_lider2)
	{
		next_lider = lider2;
		sub_next_lider = lider1;
	}
	else
	{
		next_lider = lider1;
		sub_next_lider = lider2;
	}

	EV << "    Proximo lider: " << next_lider << ", y sub lider: " << sub_next_lider << "\n";

}