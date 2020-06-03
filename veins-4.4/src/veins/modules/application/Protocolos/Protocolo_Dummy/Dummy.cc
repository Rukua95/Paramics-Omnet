/*
Protocolo dummy. Tiene funcionalidad para calcular flujo y tiempo promedio
*/

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

	// Vehiculos mandan mensajes durante el ciclo de espera 
	if(msg == sharedDataZoneMessage)
	{
		info_message->setData(data);
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
				info_message->setData(data);

				return;
			}
		}
	}

	scheduleAt(simTime() + ping_interval, self_beacon);

}


void Dummy::onData(WaveShortMessage *wsm)
{
}


void Dummy::onBeacon(WaveShortMessage *wsm)
{
}
