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
#include "veins/modules/messages/NodeInfoMessage_m.h"
#include <mutex>

class VTL : public Base
{
protected:

	// Identificadores de lideres.
	bool is_lider;
	bool is_lane_lider;
	bool exist_lider;
	bool exist_lane_lider;
	bool is_new_lider;


	// Parametros para saber si el lider esta en tiempo extra.
	bool isExtraWaitingTime;
	double extraWaitingTime;

	bool direction_to_left;
	bool crossing_left;

	double tiempo_semaforo;

	SimTime stop_time;

	std::vector<int> firstCar;

	// Metodos
    void initialize(int stage);
    void finish();
    void handleSelfMsg(cMessage *msg);
    void onData(WaveShortMessage *wsm);
    void onBeacon(WaveShortMessage *wsm);
	void updateEstate();

	void prepareMsgData(vehicleData& data, int msgTipe);
	bool isGoingLeft();
	bool canBeLider(double velocity, double distance);
	void existNextLider(bool getWaitingTime);

};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_CrossRoad_H_ */
