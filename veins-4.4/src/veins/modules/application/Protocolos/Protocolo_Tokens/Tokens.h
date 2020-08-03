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

#ifndef SRC_VEINS_MODULES_APPLICATION_PVEINS_Tokens_H_
#define SRC_VEINS_MODULES_APPLICATION_PVEINS_Tokens_H_

#include "veins/modules/application/Protocolos/Base_Protocolo/Base.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/messages/NodeInfoMessage_m.h"
#include <mutex>

class Tokens : public Base
{
protected:
	class Token
	{
		public:
			Coord position_token;
			int id_token;

			void create(int id, Coord position)
			{
				id_token = id;
				position_token = position;
			}
			
	};

	// Parametros de protocolo

	// Lista de Token que se pueden usar.
	std::vector<Token> token_vector;

	// Lista de tokens a usar por el vehiculo (lista de enteros)
	std::vector<int> tokens_list;
	int id_token_in_use;

	// Lista de tokens a usar segun origen y destino
	std::vector<std::vector<std::vector<int> > > token_table;

	// Lista de vehiculos con mejor prioridad
	std::set<int> better_priority_cars;

	// Prioridad
	double sum_priority;
	double priority;
	int allow_continue;
	double block_time;

	// Tiempo en espera
	double idling_time;

	// Constantes de prioridad
	double k_distance;
	double k_velocity;
	double k_idle_time;
	double k_cars;

	double share_data_radio;
	double token_selection_radio;


	// Metodos
    void initialize(int stage);
    void finish();
    void handleSelfMsg(cMessage *msg);
    void onData(WaveShortMessage *wsm);
    void onBeacon(WaveShortMessage *wsm);
	void updateEstate();

	void prepareMsgData(vehicleData& data, int msgTipe);
	void setTokens();
	void getTokens();
	void calculateIndividualPriority();
	void tokenInUse();

	bool comparePriority(double vhc_priority, int sender_id);
	bool compareTokens(int sender_in, int sender_out, int sender_token_in_use);

};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_CrossRoad_H_ */
