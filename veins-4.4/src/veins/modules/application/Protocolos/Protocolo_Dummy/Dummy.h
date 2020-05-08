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

#ifndef SRC_VEINS_MODULES_APPLICATION_PVEINS_Dummy_H_
#define SRC_VEINS_MODULES_APPLICATION_PVEINS_Dummy_H_

#include "veins/modules/application/Protocolos/Base_Protocolo/Base.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/messages/NodeInfoMessage_m.h"
#include <mutex>

class Dummy : public Base
{
protected:
	// Metodos
    void initialize(int stage);
    void finish();
    void handleSelfMsg(cMessage *msg);
    void onData(WaveShortMessage *wsm);
    void onBeacon(WaveShortMessage *wsm);
	void updateEstate();

	void prepareMsgData(vehicleData& data, int msgTipe);

public:
	std::string test = "nada";
};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_CrossRoad_H_ */
