cplusplus {{
#include "veins/modules/messages/WaveShortMessage_m.h"
#include "veins/base/utils/Coord.h"
}}

class noncobject Coord;

struct vehicleDataToken {
	
	int id_cell_begin;
	int id_cell_end;
	int in_bloqued_queue;

};

class WaveShortMessage;

packet Token_message extends WaveShortMessage {
    vehicleDataToken data;
}