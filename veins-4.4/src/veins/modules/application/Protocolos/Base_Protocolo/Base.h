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

#ifndef SRC_VEINS_MODULES_APPLICATION_PVEINS_Base_H_
#define SRC_VEINS_MODULES_APPLICATION_PVEINS_Base_H_

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/messages/NodeInfoMessage_m.h"
#include <mutex>

class Base : public BaseWaveApplLayer
{
protected:
    Veins::TraCIMobility* mobility;
    Veins::TraCICommandInterface *traci;
    Veins::TraCICommandInterface::Vehicle *traciVehicle;

    // Mensaje de informacion.
	NodeInfoMessage* info_message;
	
	// Self message
    cMessage *self_beacon;
	cMessage *sharedDataZoneMessage;

	// Tiempo entre intervalos
    SimTime ping_interval;

	// Identificadores de nodos y links correspondientes a la calle.
	std::string laneId;
	std::string roadId;
	std::string startId;
	std::string endId;


	// Parametros para determinar acciones segun posicion.

	// Identificadores para saber si el auto lo estamos deteneniendo o si ya lo detubimos.
	bool stoping;
	bool stoped;

	// Booleanos de estado de posicion de vehiculos
	bool outJunction;
	bool inSharedDataZone;
	bool crossing;

	// Direccion de entrada y salida a interseccion
	int direction_junction;
	int direction_out;

	// Distancia y tiempo hacia interseccion
	double distance_to_junction;
	double time_to_junction;

	// Radios de distintas zonas.
	double shared_data_radio;
	double lider_select_radio;


	// Cinematica vehiculos.

	// Constantes cinematica
	double max_accel;
	double min_accel;
	double max_velocidad;

	// Aceleracion (y variables extras utilizadas para calcular) y velocidad de vehiculo
	double last_vel;
	double acceleration;
	bool got_last_accel;

	double axis_speed;

	// Vectores de velocidad y posicion
	Coord position;
	Coord velocity;
	Coord intersection;

	// Parametros de simulacion.
	double sim_update_interval;
	double sim_ping_interval;

	double time_in_junction;
	double intersection_exit_time;
	double time_in_wait;

	bool vehicle_removed = false;


	// Tablas de informacion

	// Direcciones iniciales y finales
	std::map<std::string, int> directionMap, arrivalMap;
	std::map<int, std::string> direction_name;

	// Informacion de autos
	std::vector<std::map<int, vehicleData> > carTable;

	// Car lenght & width
	double car_lenght;
	double car_width;

	// Colision list: car id, simulation time
	std::map<int, double> colision_list;

	// Metodos
    void initialize(int stage);
    void finish();
    void handleSelfMsg(cMessage *msg);
    void onData(WaveShortMessage *wsm);
    void onBeacon(WaveShortMessage *wsm);
	void updateEstate();

	NodeInfoMessage* prepareNIM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, int serial);
	
	int finalDirection();
	void prepareMsgData(vehicleData& data, int msgTipe);
	void smartDetention();
	void timeToJunction();
	void getBasicParameters();
	void detectColision(vehicleData data);
	void getCarPoint(std::vector<Coord> &lim, double theta);
	void registerOutOfJunction();

};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_CrossRoad_H_ */
