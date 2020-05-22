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

#ifndef SRC_VEINS_MODULES_APPLICATION_PVEINS_Grilla_MPIP_H_
#define SRC_VEINS_MODULES_APPLICATION_PVEINS_Grilla_MPIP_H_

#include "veins/modules/application/Protocolos/Base_Protocolo/Base.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/messages/NodeInfoMessage_m.h"
#include <mutex>

class Grilla_MPIP : public Base
{
protected:
	class Cell
	{
		public:
			Coord p1, p2;
			int id_cell;

			std::set<int> car_id_reserve;
			std::queue<int> car_id_block_queue;
			int car_id_block;

			void create(int id, Coord position1, Coord position2)
			{
				id_cell = id;
				p1 = position1;
				p2 = position2;

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

	// Registro de celdas que se pueden usar
	std::vector<Cell> cell_register;

	// Lista de celdas a usar por el vehiculo (lista de enteros)
	std::vector<int> cell_list;

	int id_cell_begin;
	int id_cell_end;

	bool in_block_area;

	// Lista de celdas a usar segun origen y destino
	std::vector<std::vector<std::vector<int> > > cells_table;

	// Prioridad
	double priority;

	// Variables para determinar bloqueo por Paramics
	double detention_time;


	// Metodos
    void initialize(int stage);
    void finish();
    void handleSelfMsg(cMessage *msg);
    void onData(WaveShortMessage *wsm);
    void onBeacon(WaveShortMessage *wsm);
	void updateEstate();

	void prepareMsgData(vehicleData& data, int msgTipe);

	void setCells();
	void getCells();
	void calculateIndividualPriority();
	void cellsUsed();
	void detentionLastCell();

	void removeVehicle(int reason);

public:
	std::string test = "nada";
};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_CrossRoad_H_ */
