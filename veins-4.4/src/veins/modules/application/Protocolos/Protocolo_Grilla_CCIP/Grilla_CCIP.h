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

#ifndef SRC_VEINS_MODULES_APPLICATION_PVEINS_Grilla_CCIP_H_
#define SRC_VEINS_MODULES_APPLICATION_PVEINS_Grilla_CCIP_H_

#include "veins/modules/application/Protocolos/Base_Protocolo/Base.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/messages/NodeInfoMessage_m.h"
#include <mutex>

class Grilla_CCIP : public Base
{
protected:
	class Cell
	{
		public:
			Coord p1, p2;
			int id_cell;

			std::set<int> car_id_reserve;
			int car_id_block;

			void create(int id, Coord position1, Coord position2)
			{
				id_cell = id;
				p1 = position1;
				p2 = position2;

				car_id_block = -1;
			}

	};

	// Parametros de protocolo

	// Registro de celdas que se pueden usar
	std::vector<Cell> cell_register;

	// Lista de celdas a usar por el vehiculo (lista de enteros)
	std::vector<int> cell_list;
	int id_cell_in_use;

	// Lista de celdas a usar segun origen y destino
	std::vector<std::vector<std::vector<int> > > cells_table;

	// Lista de vehiculos con mejor prioridad
	std::set<int> better_priority_cars;

	// Prioridad
	double priority;
	int allow_continue;

	// Radios
	double shared_data_radio;
	double cell_selection_radio;


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
	void cellInUse();
	bool compareCells(int in, int out, int cell_in_use);


public:
	std::string test = "nada";
};

#endif /* SRC_VEINS_MODULES_APPLICATION_PVEINS_CrossRoad_H_ */
