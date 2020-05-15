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

#include <veins/modules/application/Protocolos/Protocolo_Dummy/Dummy.h>
#include <veins/modules/application/ExtTraCIScenarioManagerLaunchd/ExtTraCIScenarioManagerLaunchd.h>
#include <veins/modules/mobility/traci/TraCIColor.h>
#include <veins/modules/mobility/traci/TraCIScenarioManager.h>
#include <cstdlib>
#include <algorithm>


Define_Module(Dummy);


void Dummy::initialize(int stage)
{
    Base::initialize(stage);
}

void Dummy::finish()
{
    Base::finish();
}


void Dummy::handleSelfMsg(cMessage *msg){
	/////////////////////////////////////////////////////////////////
	// Obtencion de datos basicos.
	/////////////////////////////////////////////////////////////////
	Base::handleSelfMsg(msg);


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
				
				outJunction = true;
				traciVehicle->setColor(Veins::TraCIColor::fromTkColor("purple"));
				
				prepareMsgData(data, 1);
				info_message->setData(data);
				sendWSM((WaveShortMessage*) info_message->dup());

				return;
			}
		}
	}


	/////////////////////////////////////////////////////////////////
	// Area de envio de infomacion
	/////////////////////////////////////////////////////////////////
	if(distance_to_junction <= shared_data_radio)
	{
		EV << ">>> Shared data zone <<<\n";
		Base::removeVehicle(0);
		return ;

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


void Dummy::onData(WaveShortMessage *wsm)
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


	/////////////////////////////////////////////////////////////////
	// Tipos de mensaje
	/////////////////////////////////////////////////////////////////


	// Tipo de mensaje 1: Vehiculo ya salio de la interseccion
	if(tipe == 1)
	{
		EV << ">>> Delete car " << sender << " from table <<<\n";
		carTable[sender_in].erase(sender);

		return ;
	}
		
	detectColision(data);
	carTable[sender_in][sender] = data;

}


void Dummy::onBeacon(WaveShortMessage *wsm)
{
}


/**
 * Funcion que prepara contenido de mensaje enviado por los auto.
 */
void Dummy::prepareMsgData(vehicleData& data, int msgTipe)
{
	Base::prepareMsgData(data, msgTipe);
}
