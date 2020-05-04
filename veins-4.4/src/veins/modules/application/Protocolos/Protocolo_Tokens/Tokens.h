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

			std::set<int> car_id_reserve;
			int car_id_block;

			void create(int id, Coord position)
			{
				id_token = id;
				position_token = position;

				car_id_block = -1;
			}

			void reserve(int id_car)
			{
				car_id_reserve.insert(id_car);
			}

			void release(int id_car)
			{
				car_id_reserve.erase(id_car);
			}

			void block(int id_car)
			{
				car_id_block = id_car;
			}

			void unblock()
			{
				car_id_block = -1;
			}
			
	};

	// Parametros de protocolo

	// Tokens

	// Lista de Token que se pueden usar.
	std::vector<Token> token_vector;

	// Lista de tokens a usar por el vehiculo (lista de enteros)
	std::vector<int> tokens_list;
	int id_token_in_use;

	// Lista de tokens a usar segun origen y destino
	std::vector<std::vector<std::vector<int> > > token_table;

	// Prioridad
	double last_individual_priority;
	double individual_priority;
	double sum_priority;

	// Tiempo en espera
	double idling_time;

	// Constantes de prioridad
	double k_distance;
	double k_velocity;
	double k_idle_time;
	double k_cars;


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
	void tokensUsed();

};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_CrossRoad_H_ */
