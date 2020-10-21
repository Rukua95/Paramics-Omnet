/*
Implementacion de protocolo de semaforos virtuales.
*/

#include <veins/modules/application/Protocolos/Protocolo_VTL/VTL.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <stdlib.h>
#include <cstdlib>
#include <algorithm>


Define_Module(VTL);

void VTL::initialize(int stage)
{
    Base::initialize(stage);

    switch (stage)
    {
    case 0:
		// Identificador de lider y sublider
		is_lider = false;

		is_sub_lider = false;
		sub_lider_id = -1;

		// Flag de tiempo de espera extra
		lider_extra_waiting = false;

		// Flag de vehiculo al entrar en zona de seleccion de lider
		first_msg = false;

		// Identificador de lider siguiente o predecesor
		last_lider = -1;
		next_lider = -1;

		// Tiempo de simulacion cuando vehiculo toma liderazgo
		stop_time = -1.0;
		enter_conflict_zone_time = -1.0;

		// Determina si vehiculo dobla a la izquierda
		direction_to_left = false;
		crossing_left = false;

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
		intervals_counting = 0;		//rand() % intervals_per_selfsmg;
		
        break;
    default:
        break;
    }
}

void VTL::finish()
{
    Base::finish();
}


void VTL::handleSelfMsg(cMessage *msg){
	/////////////////////////////////////////////////////////////////
	// Obtencion de datos basicos.
	/////////////////////////////////////////////////////////////////
	Base::handleSelfMsg(msg);

	if(outJunction)
	{
		Base::removeVehicle(0);
		return;
	}

	if(distance_to_junction <= lider_selection_radio && enter_conflict_zone_time < 0.0)
		enter_conflict_zone_time = simTime().dbl();

	intervals_counting++;

	// Determinar si este auto dobla hacia la izquierda.
	direction_to_left = isGoingLeft();

	EV << ">>> is lider: " << is_lider << "\n";
	EV << "    is sublider: " << is_sub_lider << "\n";
	EV << "    Lider siguiente: " << next_lider << "\n";
	EV << "    Lider anterior: " << last_lider << "\n";

	EV << "    Viraje izquierdo: " << direction_to_left << "\n";
	EV << "    Cruzando a izquierda: " << crossing_left << "\n";

	EV << "    stop_time: " << stop_time << "\n";
	EV << "    actual_time: " << simTime().dbl() << "\n";
	EV << "    Enter time conflict zone: " << enter_conflict_zone_time << "\n";

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
	// Acciones de lider
	/////////////////////////////////////////////////////////////////
	if(is_lider)
	{
		traciVehicle->setColor(Veins::TraCIColor::fromTkColor("red"));
		if(crossing_left)
			crossing_left = false;

		// Detencion de lider
		if(distance_to_junction <= lider_selection_radio)
			Base::detention();

		// Verificar existencia de sub lider
		if(sub_lider_id == -1)
		{
			EV << "    No hay sublider, buscando...\n";

			int d = (direction_junction + 2) % 4;
			int id_best = -1;
			double dist_best = 1e8;

			// Iterando sobre los vehiculos en calle opuesta
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

		// Lider se encuentra en tiempo de espera extra
		if(lider_extra_waiting)
		{
			EV << ">>> Tiempo de espera extra: busqueda de proximo lider\n";

			// Aun no se ha encontrado lider heredero
			if(next_lider == -1)
			{
				EV << "    Buscando siguiente lider\n";

				// Se busca siguiente lider y lider de referencia
				searchNextLider();

				// Se encontro siguiente lider
				if(next_lider != -1)
				{
					EV << "    Siguiente lider encontrado: " << next_lider << "\n";

					// Mensaje para notificar lider heredero
					prepareMsgData(data, 3);

					info_message->setData(data);
					sendWSM((WaveShortMessage*) info_message->dup());
					intervals_counting = 0;

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
				lider_extra_waiting = false;
				liders_list[direction_junction] = -1;

				Base::continueTravel();

				// Mensaje fin tiempo de lider
				prepareMsgData(data, 2);

				info_message->setData(data);
				sendWSM((WaveShortMessage*) info_message->dup());
				intervals_counting = 0;

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
		// Lider aun esta esperando por tiempo de semaforo
		else
		{
			EV << ">>> Lider en espera\n";

			int d = (direction_junction + 1) % 4;
			
			// En caso que lider heredero y lider original estan al mismo tiempo
			if(last_lider != -1 && (last_lider == liders_list[d] || last_lider == liders_list[(d+2) % 4]))
			{
				EV << "    Vehiculo aun tiene que esperar que lider anterior se retire\n";
				EV << "    No se puede analizar termino de liderazgo\n";
				EV << "    Reiniciando tiempo de detencion\n";
				stop_time = simTime().dbl();

			}
			else
			{
				EV << ">>> Analizando termino de liderazgo\n";

				// Termino de semaforo
				bool end_time = ((simTime().dbl() - stop_time) >= tiempo_semaforo && stop_time > 0.0);

				// Verde rapida 
				bool no_more_cars = (carTable[(direction_junction + 1) % 4].empty() && carTable[(direction_junction + 3) % 4].empty());

				// Termino de liderazgo
				if(end_time || no_more_cars)
				{
					/*
					if(end_time && no_more_cars)
						traciVehicle->setColor(Veins::TraCIColor(0, 255, 255, 255)); // cyan
					else if(end_time)
						traciVehicle->setColor(Veins::TraCIColor(0, 0, 139, 255)); // dark blue
					else
						traciVehicle->setColor(Veins::TraCIColor(104, 0, 139, 255)); // light blue
					*/


					EV << "    Fin de tiempo de lider\n";
					EV << "     > end_time: " << end_time << "\n";
					EV << "     > no_more_cars: " << no_more_cars << "\n";

					EV << ">>> Busqueda de proximo lider\n";
					searchNextLider();

					EV << "    Entrando en fase de tiempo extra\n";
					lider_extra_waiting = true;

					// Aviso si hay lider heredero
					if(next_lider != -1)
						prepareMsgData(data, 3);
					else
						prepareMsgData(data, 0);
					
					info_message->setData(data);
					sendWSM((WaveShortMessage*) info_message->dup());
					intervals_counting = 0;

					scheduleAt(simTime() + ping_interval, self_beacon);

					return ;
				}
				else
				{
					double delta_time = tiempo_semaforo - (simTime().dbl() - stop_time);
					EV << "    Aun existen vehiculos, tiempo restante de semaforo: " << delta_time << "\n";
				}
			}
		}
	}


	/////////////////////////////////////////////////////////////////
	// Acciones de sub-lider
	/////////////////////////////////////////////////////////////////
	if(is_sub_lider)
	{
		traciVehicle->setColor(Veins::TraCIColor(1, 1, 1, 255));

		if(crossing_left)
			crossing_left = false;

		if(distance_to_junction <= lider_selection_radio)
			Base::detention();
			
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

			// Registrar entrada en interseccion
			Base::registerInOfJunction();

			// Mensaje broadcast a vehiculos
			prepareMsgData(data, 0);
			info_message->setData(data);

			if(!crossing || intervals_counting % intervals_per_selfmsg == 0)
			{
				// Iniciando estado de cruce
				crossing = true;
				sendWSM((WaveShortMessage*) info_message->dup());
			}

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
			EV << ">>> No existe lider, calculando si es necesario\n";

			// Determinamos si hay vehiculos calle perpendicular
			bool conflict = false;
			for(int i=1; i<4; i+=2)
			{
				int d = (direction_junction + i) % 4;
				for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
				{
					EV << "    id: " << it->first << "; enter tiem: " << it->second.enter_conflict_zone_time << "\n";
					if(it->second.enter_conflict_zone_time > 0.0)
						conflict = true;
				}
			}

			// En caso de conflicto, vehiculo verifica si es lider
			if(conflict)
			{
				EV << ">>> Existe conflicto, determinando si soy lider\n";

				// Si hay conflicto, se elige lider
				bool is_posible_lider = true;

				// Verificar si este vehiculo es el primero en su pista
				int d = direction_junction;
				for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
				{
					//if(distance_to_junction > it->second.distance_to_junction)
					//	is_posible_lider = false;

					if(enter_conflict_zone_time > it->second.enter_conflict_zone_time && it->second.enter_conflict_zone_time > 0.0)
						is_posible_lider = false;
				}
				EV << "    Soy primero en pista: " << is_posible_lider << "\n";


				// Verificar si es el primero en la calle
				d = (direction_junction + 2) % 4;
				double best_distance = 1e8;
				for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
				{
					//if(best_distance > it->second.distance_to_junction && !crossing)
					//	best_distance = it->second.distance_to_junction;

					if(best_distance > it->second.enter_conflict_zone_time && it->second.enter_conflict_zone_time > 0.0)
						best_distance = it->second.enter_conflict_zone_time;
				}

				//if(best_distance < distance_to_junction)
				//	is_posible_lider = false;
				
				if(best_distance < enter_conflict_zone_time)
					is_posible_lider = false;

				EV << "    Soy primero en calle: " << is_posible_lider << "\n";
				

				// Verificar si este vehiculo es ultimo comparado con los primeros
				best_distance = 1e8;
				for(int i=1; i<=3; i+=2)
				{
					d = (direction_junction + i) % 4;
					for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
					{
						//if(best_distance > it->second.distance_to_junction && !crossing)
						//	best_distance = it->second.distance_to_junction;

						if(best_distance > it->second.enter_conflict_zone_time && it->second.enter_conflict_zone_time > 0.0)
							best_distance = it->second.enter_conflict_zone_time;
					}
				}

				//if(best_distance > distance_to_junction && best_distance <= lider_selection_radio)
				//	is_posible_lider = false;

				if(best_distance > enter_conflict_zone_time && best_distance < 1e7)
					is_posible_lider = false;

				EV << "    Soy ultimo de los primeros: " << is_posible_lider << "\n";

				if(is_posible_lider)
				{
					EV << ">>> Puedo ser el lider\n";

					// Detencion de vehiculo
					traciVehicle->setColor(Veins::TraCIColor::fromTkColor("red"));
					Base::detention();
					
					// Asignacion de parametros de lider
					is_lider = true;
					liders_list[direction_junction] = myId;
					stop_time = simTime().dbl();


					EV << "    Eligiendo sublider...\n";

					int d = (direction_junction + 2) % 4;
					int id_best = -1;
					double dist_best = 1e8;

					// Iterando sobre los vehiculos en calle opuesta
					for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
					{
						if(it->second.distance_to_junction < dist_best)
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


					// Se manda mensaje para notificar liderazgo
					prepareMsgData(data, 0);
					info_message->setData(data);
					sendWSM((WaveShortMessage*) info_message->dup());
					intervals_counting = 0;

					scheduleAt(simTime() + ping_interval, self_beacon);

					return;
				}
			}
		}


		// Procesamos posible viraje a izquierda del vehiculo
		if(direction_to_left)
		{
			EV << ">>> Verificando viraje a izquierda\n";

			// Revisando si soy primer vehiculo en esta direccion
			bool is_first = true;
			int id_first_no_left = -1;
			double time_first_no_left = 1e8;

			for(auto it = carTable[direction_junction].begin(); it!=carTable[direction_junction].end(); it++)
			{
				EV << "    Vehiculo en pista " << it->first << "\n";
				if(it->second.enter_conflict_zone_time < enter_conflict_zone_time && it->second.enter_conflict_zone_time > 0.0 && !it->second.crossing)
				{
					EV << "    Vehiculo " << it->first << " esta adelante, no se puede cruzar\n";
					is_first = false;
				}

				if(it->second.time_to_junction > time_to_junction && time_first_no_left > it->second.time_to_junction && !it->second.goingLeft)
				{
					time_first_no_left = it->second.time_to_junction;
				}
			}


			// Revisando condicion de primer vehiculo en direccion opuesta
			bool no_oposition = true;

			int d = (direction_junction + 2) % 4;
			int id_near = -1, id_near_no_left = -1;
			double enter_time = -1.0, enter_time_no_left = -1.0;

			for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
			{
				// No hay colision con vehiculos opuestos que giran a la izquierda
				EV << "    Comparando con vehiculo " << it->first << "\n";
				if(it->second.goingLeft)
					EV << "    -> vehiculo vira a izquierda\n";

				// Vehiculo opuesto mas cercano a interseccion
				if(it->second.time_to_junction < enter_time || enter_time < 0.0)
				{
					id_near = it->first;
					enter_time = it->second.time_to_junction;
				}

				if((it->second.time_to_junction < enter_time_no_left || enter_time_no_left < 0.0) && !it->second.goingLeft)
				{
					id_near_no_left = it->first;
					enter_time_no_left = it->second.time_to_junction;
				}
			}

			// Chocara con un vehiculo opuesto
			EV << "    Vehiculo opuesto mas cercano " << id_near << "\n";
			EV << "    Tiempo " << enter_time << "\n";
			if(id_near != -1)
			{
				bool oposite_is_lider = carTable[d][id_near].isLider;
				bool oposite_is_sub_lider = carTable[d][id_near].is_sub_lider;
				bool oposite_going_left = carTable[d][id_near].goingLeft;

				double oposite_enter_time = carTable[d][id_near].enter_conflict_zone_time;
				double oposite_time_to_junction = carTable[d][id_near].time_to_junction;

				// Vehiculos lideres y sublideres siempre se detienen
				if(oposite_is_lider || (oposite_is_sub_lider && id_near != sub_lider_id))
				{
					no_oposition = true;
				}
				else
				{
					double delta_tiempo = 0.3;

					// Caso especial en que primer vehiculo opuesto se mueve a la izquierda
					if(oposite_going_left)
					{
						EV << ">>> Ambos vehiculos viran a izquierda\n";

						if(oposite_enter_time < enter_conflict_zone_time)
						{
							no_oposition = false;
						}
						else
						{
							no_oposition = true;
						}
					}
					// Caso general
					else
					{
						if(oposite_time_to_junction - delta_tiempo - time_to_junction > 2.5)
							no_oposition = true;
						else
						{
							no_oposition = false;
						}
						
					}
				}
			}


			EV << "    no_oposicion: " << no_oposition << "\n";
			EV << "    is_first: " << is_first << "\n";

			if(!crossing_left)
			{
				if(no_oposition && is_first && !is_sub_lider && !is_lider)
				{
					EV << ">>> Es posible virar\n";
					Base::continueTravel();
					crossing_left = true;
				}
				else
				{
					EV << ">>> No es posible virar\n";
					Base::detention();
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
			//traciVehicle->setColor(Veins::TraCIColor::fromTkColor("yellow"));
			inSharedDataZone = true;
			scheduleAt(simTime() + ping_interval, sharedDataZoneMessage);
		} 
		else
			scheduleAt(simTime() + ping_interval, self_beacon);

		return;

	}

	scheduleAt(simTime() + ping_interval, self_beacon);

}

void VTL::onData(WaveShortMessage *wsm)
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
		}
	}


	// Lider actual selecciona su heredero
	if(tipe == 3)
	{
		// Lider entrega identificador del siguiente lider
		int id_next_lider = data.next_lider;
		int id_sub_next_lider = data.sub_next_lider;
		EV << ">>> Lider entrando a tiempo de espera extra, vehiculo " << id_next_lider << " es el siguiente lider\n";
		EV << "    Lider entrando a tiempo de espera extra, vehiculo " << id_sub_next_lider << " es el siguiente sublider\n";

		// Este vehiculo es el lider
		if(myId == id_next_lider)
		{
			EV << "    Soy heredero del lider actual\n";

			is_lider = true;
			sub_lider_id = id_sub_next_lider;
			liders_list[direction_junction] = id_next_lider;

			last_lider = sender;
			stop_time = -1.0;

			traciVehicle->setColor(Veins::TraCIColor(255, 0, 150, 0));
		}

		// Este vehiculo es el sub-lider
		if(myId == id_sub_next_lider)
		{
			EV << "    Soy sublider del lider heredero\n";

			is_sub_lider = true;
		}

		// Busqueda de posicion de lider para poder clasificarlo
		if(id_next_lider != -1)
		{
			for(int i=0; i<4; ++i)
			{
				if(carTable[i].count(id_next_lider) != 0)
				{
					liders_list[i] = id_next_lider;
					break;
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

	// En caso recordar un lider incorrecto
	if(!data.isLider && liders_list[sender_in] == sender)
		liders_list[sender_in] = -1;

	// Reaccion a mensaje de un vehiculo lider
	if(data.isLider && !crossing)
	{
		EV << ">>> Existe lider en la interseccion\n";

		// Lider esta en la misma calle
		if(sender_in % 2 == direction_junction % 2)
		{
			EV << ">>> Mensaje de lider en mi pista o pista opuesta\n";

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
					next_lider = -1;

					liders_list[sender_in] = sender;
					if(sender_in % 2 == direction_junction % 2)
						liders_list[direction_junction] = -1;

					stop_time = -1.0;

					Base::continueTravel();
					traciVehicle->setColor(Veins::TraCIColor(255, 255, 255, 0));
				}
			}
			else
				liders_list[sender_in] = sender;

			if(data.sub_lider == myId)
				is_sub_lider = true;
			else
				is_sub_lider = false;
				
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
					if(stop_time > sender_stop_time)
					{
						is_lider = false;
						liders_list[sender_in] = sender;
						liders_list[direction_junction] = -1;
						stop_time = -1.0;
						
						Base::continueTravel();
						traciVehicle->setColor(Veins::TraCIColor::fromTkColor("purple"));
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

void VTL::onBeacon(WaveShortMessage *wsm)
{
}


/**
 * Funcion que prepara contenido de mensaje enviado por los auto.
 */
void VTL::prepareMsgData(vehicleData& data, int msgTipe)
{
	Base::prepareMsgData(data, msgTipe);

	data.isLider = is_lider;
	data.sub_lider = sub_lider_id;

	data.is_sub_lider = is_sub_lider;

	data.last_lider = last_lider;
	data.next_lider = next_lider;
	data.sub_next_lider = sub_next_lider;

	data.stop_time = stop_time;
	data.goingLeft = direction_to_left;

	data.enter_conflict_zone_time = enter_conflict_zone_time;
}


/**
 * Determina la direccion final de un auto
 */
bool VTL::isGoingLeft()
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
void VTL::searchNextLider()
{
	// Identificadores y distancias
	std::vector<std::pair<double, int> > posible_next_liders(2, std::pair<double, int>(1e8, -1));
	for(int i=0; i<2; i++)
	{
		int d = (direction_junction + 1 + 2*i) % 4;

		for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
		{
			double dist = it->second.distance_to_junction;
			double best_dist = posible_next_liders[i].first;

			// Proximo posible lider tiene que estar fuera de area de interseccion
			if(dist < best_dist && dist > lider_selection_radio && dist < shared_data_radio - 10)
			{
				posible_next_liders[i].second = it->first;
				posible_next_liders[i].first = dist;
			}

			// Vehiculos a esperar que crucen
			if(dist <= lider_selection_radio)
			{
				EV << "    Esperando a vehiculo " << it->first << "\n";
				vehicles_to_wait.insert(it->first);
			}
		}
	}

	std::sort(posible_next_liders.begin(), posible_next_liders.end());

	next_lider = posible_next_liders[0].second;
	sub_next_lider = posible_next_liders[1].second;

	EV << "    Proximo lider: " << next_lider << ", y sub lider: " << sub_next_lider << "\n";

}