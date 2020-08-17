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

	}
	else
	{
		// Vehiculo salio de la interseccion
		if(crossing)
		{
			EV << ">>> Out of junction <<<\n";
			Base::registerOutOfJunction();
			Base::removeVehicle(0);

			return;

		}
	}

	scheduleAt(simTime() + ping_interval, self_beacon);

}


void Dummy::onData(WaveShortMessage *wsm)
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
}


void Dummy::onBeacon(WaveShortMessage *wsm)
{
}
