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

#ifndef SRC_VEINS_MODULES_APPLICATION_PVEINS_VTL_H_
#define SRC_VEINS_MODULES_APPLICATION_PVEINS_VTL_H_

#include "veins/modules/application/Protocolos/Base_Protocolo/Base.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
//#include "veins/modules/application/Protocolos/new_Protocolo_VTL/VTL_msg_m.h"
#include "veins/modules/messages/NodeInfoMessage_m.h"
#include <mutex>

class VTL : public Base
{
protected:

	// Identificador de lider.
	int lider_id;
	int sub_lider_id;
	bool is_lider;
	bool is_sub_lider;

	// Identificadores para tiempo extra de espera
	bool lider_extra_waiting;
	bool fast_exit;

	// Identificador de viraje a izquierda
	bool direction_to_left;
	bool crossing_left;

	// Identificador de primera entrada en zona de colision
	bool first_query;

	// Estado que permite ignorar existencia de lider, y continuar con el viaje
	bool block_movement;

	// Identificador de lider anterior y heredero
	int last_lider;
	int next_lider;
	int sub_next_lider;

	// Tiempo en que comienza lider
	SimTime stop_time;
	SimTime lider_start_time;

	double enter_conflict_zone_time;

	// Lista de lideres activos
	std::vector<int> liders_list;
	std::set<int> vehicles_to_wait;

	// Tiempo de semaforo
	double tiempo_semaforo;
	
	// Radios de zonas
	double shared_data_radio;
	double lider_selection_radio;


	// Metodos
    void initialize(int stage);
    void finish();
    void handleSelfMsg(cMessage *msg);
    void onData(WaveShortMessage *wsm);
    void onBeacon(WaveShortMessage *wsm);
	void updateEstate();

	void searchNextLider();
	void selectIgnorants();


	void prepareMsgData(vehicleData& data, int msgTipe);
	bool isGoingLeft();
	void existNextLider(bool getWaitingTime);
	void searchSubLider();

};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_CrossRoad_H_ */
