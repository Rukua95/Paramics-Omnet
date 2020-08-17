/*
Implementacion de protocolo de semaforos virtuales (VTL) con tiempo de espera.
*/

#include <veins/modules/application/Protocolos/Protocolo_VTL/VTL.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <cstdlib>
#include <algorithm>


Define_Module(VTL);

void VTL::initialize(int stage)
{
    Base::initialize(stage);

    switch (stage)
    {
    case 0:
		is_lider = false;
		is_lane_lider = false;
		exist_lider = false;
		exist_lane_lider = false;
		is_new_lider = false;

		extraWaitingTime = -1.0;
		isExtraWaitingTime = false;

		direction_to_left = false;
		crossing_left = false;

		stop_time = -1.0;

		firstCar = std::vector<int>(4, -1);

		tiempo_semaforo = par("tiempo_semaforo").doubleValue();
		shared_data_radio = par("shared_data_radio").doubleValue();
		lider_selection_radio = par("lider_selection_radio").doubleValue();
		
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

	// Determinar si este auto dobla hacia la izquierda.
	direction_to_left = isGoingLeft();


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
	// Termino de periodo de tiempo extra.
	/////////////////////////////////////////////////////////////////
	if(is_lider && isExtraWaitingTime && simTime() - stop_time >= tiempo_semaforo + extraWaitingTime + 1.0)
	{
		// Revisar existencia de posible lider en la otra calle.
		existNextLider(false);

		// Resetear variables relacionadas a lider.
		is_lider = false;
		is_lane_lider = false;
		exist_lider = is_new_lider;
		exist_lane_lider = false;
		stop_time = -1.0;

		if(!direction_to_left)
			Base::continueTravel();

		isExtraWaitingTime = false;
		extraWaitingTime = -1.0;

		// Enviar mensajes sobre termino de tiempo de espera.
		prepareMsgData(data, 2);
		info_message->setData(data);
		sendWSM((WaveShortMessage*) info_message->dup());

		is_new_lider = false;

		scheduleAt(simTime() + ping_interval, self_beacon);

		return ;
	}


	/////////////////////////////////////////////////////////////////
	// Durante periodo de tiempo de espera extra.
	/////////////////////////////////////////////////////////////////
	if(isExtraWaitingTime)
	{
		// Revisar existencia de posible lider en la otra calle.
		existNextLider(false);

		// Enviar mensajes sobre tiempo de espera.
		prepareMsgData(data, 3);
		info_message->setData(data);
		sendWSM((WaveShortMessage*) info_message->dup());

		scheduleAt(simTime() + ping_interval, self_beacon);

		return;
	}


	/////////////////////////////////////////////////////////////////
	// Termino de semaforo para lider.
	/////////////////////////////////////////////////////////////////
	if(is_lider && simTime() - stop_time >= tiempo_semaforo + 1.0)
	{
		EV << ">>> Time-out stop time: lider car " << myId << " can continue\n";
		EV << ">>> Calculating extra waiting time...\n";
		
		// Revisar existencia de posible lider en la otra calle.
		existNextLider(true);

		// Caso: hay que esperar autos rezagados.
		EV << ">>> Extra waiting time: " << extraWaitingTime << "\n";
		if(isExtraWaitingTime)
		{
			prepareMsgData(data, 3);
			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("yellow"));
		}
		// Caso: no hay que esperar autos rezagados.
		else
		{
			is_lider = false;
			is_lane_lider = false;
			exist_lider = false;
			exist_lane_lider = false;
			stop_time = -1.0;

			if(!direction_to_left)
				Base::continueTravel();

			prepareMsgData(data, 2);

		}

		info_message->setData(data);
		sendWSM((WaveShortMessage*) info_message->dup());

		scheduleAt(simTime() + ping_interval, self_beacon);

		return;
		
	}

	// Lider deja semaforo en caso que no se acerquen vehiculos por la otra calle.
	if(is_lider)
	{
		int d1 = (direction_junction + 1) % 4;
		int d2 = (d1 + 2) % 4;

		if(carTable[d1].empty() && carTable[d2].empty())
		{
			is_lider = false;
			is_lane_lider = false;
			exist_lider = false;
			exist_lane_lider = false;
			stop_time = -1.0;

			if(!direction_to_left)
				Base::continueTravel();

			prepareMsgData(data, 2);

			info_message->setData(data);
			sendWSM((WaveShortMessage*) info_message->dup());

			scheduleAt(simTime() + ping_interval, self_beacon);

			return ;

		}
	}


	/////////////////////////////////////////////////////////////////
	// Area de seleccion de lider.
	/////////////////////////////////////////////////////////////////
	if(distance_to_junction <= lider_selection_radio)
	{
		EV << ">>> Lider selection zone <<<\n";

		EV << ">>> Zona de reparto y bloqueo de celdas\n";
		double dist_x = std::abs(position.x - traci->junction("1").getPosition().x);
		double dist_y = std::abs(position.y - traci->junction("1").getPosition().y);

		// Vehiculo esta dentro de la interseccion
		if(startId == "1" && dist_x <= 11.4 && dist_y <= 11.4)
		{
			EV << ">>> Zona de cruce\n";
			crossing = true;
			Base::registerInOfJunction();

			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("blue"));

			//cellsUsed();
			prepareMsgData(data, 0);
			info_message->setData(data);
			sendWSM((WaveShortMessage*) info_message->dup());

			scheduleAt(simTime() + ping_interval, self_beacon);

			return ;

		}
		// Vehiculo aun no llega a la interseccion o ya salio
		else if(crossing)
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


		if(!exist_lider)
		{
			// Se decide el lider segun distancia y timepo de llegada estimada a interseccion.
			bool selected_lider = false;

			for(int i=0; i<2; i++)
			{
				int d = (direction_junction + 1 + 2*i) % 4;
				for(auto it1 = carTable[d].begin(); it1 != carTable[d].end(); it1++)
				{
					double time1 = it1->second.time_to_junction;
					double dist1 = it1->second.distance_to_junction;

					EV << "    time: " << time1 << "\n";
					EV << "    dist: " << dist1 << "\n";
					EV << "    delta t: " << std::abs(time1 - time_to_junction) << "\n";
					EV << ">>>> <<<<\n";

					// Comparar direccion y tiempos de llegada a interseccion para determinar posible colision.
					if(std::abs(time1 - time_to_junction) < 3.0)// && time_to_junction > time1)
					{
						EV << ">>> I'm gonna crash!! <<<\n";
						selected_lider = selected_lider | (dist1 < distance_to_junction);
					}
				}
			}

			if(selected_lider)
			{
				EV << ">>> I think i'm the lider\n";
				detention();
				crossing_left = false;

				exist_lider = true;
				exist_lane_lider = true;
				
				is_lider = true;
				is_lane_lider = true;
				stop_time = simTime();

				prepareMsgData(data, 0);

				info_message->setData(data);
				sendWSM((WaveShortMessage*) info_message->dup());

				scheduleAt(simTime() + ping_interval, self_beacon);

				return;
			}
		} 
		// Caso: auto es un lider de pista.
		else if(is_lane_lider)
		{
			EV << ">>> Stoping lane lider...\n";
			detention();
			crossing_left = false;

			prepareMsgData(data, 0);

			info_message->setData(data);
			sendWSM((WaveShortMessage*) info_message->dup());

			scheduleAt(simTime() + ping_interval, self_beacon);

			return ;
		}
		

		// Caso: auto va a doblar a la izquierda.
		if(direction_to_left)
		{
			EV << ">>> Going left, waiting window to pass.\n";
			// Determinamos auto que va primero en la pista contraria.

			bool isFirst = true;
			bool someone_crossing = false;

			// Determinar si este vehiculo es primero
			int d = direction_junction;
			for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
			{
				// Vehiculo actual no es el primero
				if(it->second.distance_to_junction < distance_to_junction)
				{
					isFirst = false;
				}

				/*
				if(it->second.crossing)
				{
					someone_crossing = true;
				}
				*/
			}

			double secondary_best_time = 1e18;
			double secondary_best_dist = 1e18;
			double best_time = 1e18;
			double best_distance = 1e18;
			double best_speed = -1;
			int name_best = -1;
			bool toLeft = true;

			// Determinar cual vehiculo es primero en pista contraria
			d = (d+2) % 4;
			for(auto it = carTable[d].begin(); it != carTable[d].end() && isFirst; it++)
			{
				if(best_distance > it->second.distance_to_junction)
				{
					if(secondary_best_dist > best_distance)
					{
						secondary_best_time = best_time;
						secondary_best_dist = best_distance;
					}

					name_best = it->first;
					best_distance = it->second.distance_to_junction;
					best_time = it->second.time_to_junction;

					best_speed = it->second.axis_speed;
					toLeft = it->second.goingLeft;
				}
				else if(secondary_best_dist > it->second.distance_to_junction)
				{
					secondary_best_dist = it->second.distance_to_junction;
					secondary_best_time = it->second.time_to_junction;
				}

				if(it->second.crossing)
				{
					if(!it->second.goingLeft)
						someone_crossing = true;
				}
			}

			// Revisar si vehiculos en otra calle estan cruzando la interseccion
			for(int i = 1; i <= 3; i+=2)
			{
				d = (d + i) % 4;
				for(auto it = carTable[d].begin(); it != carTable[d].end() && isFirst; it++)
				{
					if(it->second.crossing)
					{
						someone_crossing = true;
					}
				}
			}
			
			
			EV << ">>> Data de auto primer auto en pista contraria\n";
			EV << "    best_time: " << best_time << "\n";
			EV << "    best_speed: " << best_speed << "\n";
			EV << "    best_distance: " << best_distance << "\n";
			EV << "    toLeft: " << toLeft << "\n";
			EV << "    crossing_left: " << crossing_left << "\n";
			
			// Decidir si doblar a la izquierda o no.
			if(!crossing_left)
			{
				// Tiempo de llegada considerando velocidad y aceleracion maxima

				double t_real1 = time_to_junction;
				double t_real2 = best_time;
				double t_real3 = secondary_best_time;

				EV << ">>> Tiempos\n";
				EV << "    t_real de este auto: " << t_real1 << "\n";
				EV << "    t_real de otro auto: " << t_real2 << "\n";
				EV << "    t_real de 2do auto: " << t_real3 << "\n";
				EV << "    t_ventaja: " << t_real2 - t_real1 << "\n";

				//////////////////////////////////////////////////////

				if(someone_crossing)
				{
					EV << ">>> Alguien esta cruzando\n";
					detention();
				}
				else if(t_real2 - t_real1 <= 4.0)
				{
					EV << ">>> No hay tiempo para doblar\n";
					if(toLeft && std::abs(t_real2 - t_real1) <= 4.5)
					{
						//if((std::abs(t_real1 - t_real2) <= 0.5 && secondary_best >= 3.0) || t_real1 < t_real2)
						if(t_real3 - t_real1 >= 4.0)
						{
							if(isFirst)
							{
								Base::continueTravel();
								crossing_left = true;
							}
							else
								detention();
						}
						else
						{
							//if(isFirst)
							if(myId < name_best && isFirst)
							{
								Base::continueTravel();
								crossing_left = true;
							}
							else
								detention();
						}
					}
					else
						//if(isFirst)
						detention();
				}
				else
				{
					if(isFirst)
					{
						Base::continueTravel();
						crossing_left = true;
					}
					else
						detention();
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
		if(!inSharedDataZone)
		{
			EV << ">>> New car in shared data zone, waiting one cicle of simulation...\n";
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
		distance_to_junction = -1;
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
	Base::getBasicParameters();
	NodeInfoMessage* msg = check_and_cast<NodeInfoMessage*>(wsm);
	vehicleData data = msg->getData();

	int tipe = data.msg_type;
	int sender = wsm->getSenderAddress();
	int recipient = wsm->getRecipientAddress();

	EV << "    tipe: " << tipe << "\n";
	EV << "    send: " << sender << "\n";
	EV << "    rcp: " << recipient << "\n";
	EV << "    time: " << data.time_to_junction << "\n";


	// Auto ha salido de la interseccion
	if(tipe == 1)
	{
		EV << ">>> Delete car " << sender << " from table <<<\n";
		carTable[data.direction_junction].erase(sender);

		return ;
	}

	// Se acabo tiempo de espera de auto lider:
	// -> tipo 2: auto lider termino tiempo de espera
	// -> tipo 3: auto lider esta en tiempo extra de espera
	if(tipe == 2 || tipe == 3)
	{
		carTable[data.direction_junction][sender] = data;

		if(tipe == 2)
		{
			EV << ">>> Car " << sender << " lider time is ending <<<\n";
			if(is_lane_lider && direction_junction % 2 == data.direction_junction % 2)
			{
				is_lane_lider = false;

				Base::continueTravel();

			}

			if(!data.isNewLider)
			{
				exist_lider = false;
				exist_lane_lider = false;

				return ;
			}
		}
		
		// Estoy fuera de zona de detencion y en calle perpendicular
		bool ver = (canBeLider(axis_speed, distance_to_junction)) && (direction_junction % 2 != data.direction_junction % 2);

		EV << "    Calculating if i am the next lider...\n";
		for(auto it = carTable[direction_junction].begin(); it != carTable[direction_junction].end() && ver; it++)
		{
			EV << "    Comparing with car " << it->first << "\n";

			int direction = it->second.direction_junction;
			double dist = it->second.distance_to_junction;
			double time = it->second.time_to_junction;
			double vel = it->second.axis_speed;

			EV << "    > direction: " << direction << "\n";
			EV << "    > distance to junction: " << dist << "\n";
			EV << "    > time to junction: " << time << "\n";
			
			// Auto con el que me comparo esta dentro de la zona de eleccion de lider -> no puede ser el siguiente lider.
			if(!canBeLider(vel, dist))
				continue;

			// Auto con el que me comparo es mejor opcion que yo -> no puedo ser lider
			if(direction_junction % 2 == direction % 2 && dist < distance_to_junction)
				ver = false;

		}
		
		if(ver)
		{
			EV << "    Soy el proximo lider!!!\n";
			is_lider = true;
			is_lane_lider = true;

			exist_lider = true;
			exist_lane_lider = true;

			traciVehicle->setColor(Veins::TraCIColor::fromTkColor("blue"));
			stop_time = simTime();
		}

		return ;
	}

	carTable[data.direction_junction][sender] = data;

	if(data.crossing)
		return ;

	EV << ">>> Sender data <<<\n";
	EV << "    isLider " << data.isLider << "\n";
	EV << "    isLaneLider: " << data.isLaneLider << "\n";
	EV << "    time to junction: " << data.time_to_junction << "\n";
	EV << "    distance to junction: " << data.distance_to_junction << "\n";
	EV << "    stop time: " << data.stop_time << "\n";
	
	if(data.stop_time > 0.0)
		EV << "    delta time: " << simTime() - data.stop_time << "\n";


	/////////////////////////////////////////////////
	// Mensaje viene del lider de una pista
	/////////////////////////////////////////////////
	if(data.isLaneLider)
	{
		EV << ">>> Message from lane lider <<<\n";
		EV << "    data direction: " << data.direction_junction << "\n";
		EV << "    my direction: " << direction_junction << "\n";
		// Auto esta en la misma pista que el lider de pista.
		if(data.direction_junction == direction_junction)
		{
			EV << "    Message from mine lane lider <<<\n";
			exist_lane_lider = true;

			// Verificar que no hay lider de pista extra.
			if(is_lane_lider)
			{
				if(data.distance_to_junction < distance_to_junction)
				{
					is_lane_lider = false;
					traciVehicle->setColor(Veins::TraCIColor::fromTkColor("grey"));
				}
			}
		}
	}


	/////////////////////////////////////////////////
	// Mensaje del lider de una calle
	/////////////////////////////////////////////////
	if(data.isLider)
	{
		EV << ">>> Message from street lider, direction: " << data.direction_junction << " <<<\n";
		exist_lider = true;

		// Auto esta en la misma calle que el lider de calle.
		if(data.direction_junction % 2 == direction_junction % 2)
		{
			// Verificar que no mas de un lider de calle en la misma calle.
			if(is_lider && data.distance_to_junction < distance_to_junction)
			{
				EV << "    I'm a false lider!!!\n";
				is_lider = false;
				traciVehicle->setColor(Veins::TraCIColor::fromTkColor("grey"));

				return;
			}

			EV << "    Estoy en la misma calle que el lider, pero en otra pista\n";
			// Verificar hay sublider en la pista.
			if(!exist_lane_lider && direction_junction != data.direction_junction)
			{
				bool ver = (canBeLider(axis_speed, distance_to_junction));
				exist_lane_lider = true;
				for(auto it = carTable[direction_junction].begin(); it != carTable[direction_junction].end() && ver; it++)
				{
					int direction = it->second.direction_junction;
					double distance = it->second.distance_to_junction;

					if(direction_junction == direction && distance < distance_to_junction)
						ver = false;

				}
				

				if(ver)
				{
					EV << "    Soy el proximo lider de pista!!!\n";
					is_lane_lider = true;
					traciVehicle->setColor(Veins::TraCIColor::fromTkColor("blue"));
				}
			}
		}
	}
}

void VTL::onBeacon(WaveShortMessage *wsm){

}


/**
 * Funcion que prepara contenido de mensaje enviado por los auto.
 */
void VTL::prepareMsgData(vehicleData& data, int msgTipe)
{
	Base::prepareMsgData(data, msgTipe);


	data.isLaneLider = is_lane_lider;
	data.isLider = is_lider;
	data.isNewLider = is_new_lider;

	data.isExtraTime = isExtraWaitingTime;
	data.stop_time = stop_time.dbl();

	data.goingLeft = direction_to_left;
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
 * Funcion que determina si en otra calle puede existir un lider, ademas de calcular el tiempo de espera extra si getWaitingTime es true
 */
void VTL::existNextLider(bool getWaitingTime)
{
	// Revisar existencia de posible lider en la otra calle.
	EV << ">>> Searching for next lider.\n";
	for(int i=0; i<=2; i+=2)
	{
		int d = (direction_junction + 1 + i) % 4;
		for(auto it = carTable[d].begin(); it != carTable[d].end(); it++)
		{
			int direction = it->second.direction_junction;
			double dist = it->second.distance_to_junction;
			double time = it->second.time_to_junction;
			double vel = it->second.axis_speed;

			EV << "    > direction: " << direction << "\n";
			EV << "    > distance to junction: " << dist << "\n";
			EV << "    > time to junction: " << time << "\n";

			if(canBeLider(vel, dist))
			{
				is_new_lider = true;
				continue;
			}
			else
			{
				if(extraWaitingTime < time && getWaitingTime)
				{
					isExtraWaitingTime = true;
					extraWaitingTime = time;
				}
			}
		}
	}
}


/**
 * Funcion que determina si una auto puede ser lider, dada la velocidad y distancia a interseccion.
 */
bool VTL::canBeLider(double velocity, double distance)
{
	double d = distance - 17.0;
	double v = velocity;
	double a = -4.5;

	EV << "    dist: " << d << "\n";
	EV << "    vel: " << v << "\n";
	EV << "    a_min: " << a << "\n";

	double t_detention = (-v) / a;
	double d_detention = v*t_detention + (1.0/2.0)*a*t_detention*t_detention;

	EV << "    time_det: " << t_detention << "\n";
	EV << "    dist_det: " << d_detention << "\n";

	// Velocidad es negativa(?)
	if(t_detention < 0)
	{
		EV << "    Cant be lider\n";
		return false;
	}

	// Esta dentro de la interseccion.
	if(d < 0)
	{
		EV << "    Cant be lider\n";
		return false;
	}

	// Si frena, se detiene despues de la interseccion.
	if(d_detention > d)
	{
		EV << "    Cant be lider\n";
		return false;
	}

	return true;
	
}