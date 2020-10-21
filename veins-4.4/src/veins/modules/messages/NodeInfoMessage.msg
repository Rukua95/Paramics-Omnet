cplusplus {{
#include "veins/modules/messages/WaveShortMessage_m.h"
#include "veins/base/utils/Coord.h"
}}

class noncobject Coord;

struct vehicleData {
	int msg_type = 0;

	int vehicle_id = 0;
	int direction_junction = -1;
	int direction_out = -1;

	Coord position;
	Coord speed;
	float axis_speed;
	float acceleration;

	float time_to_junction = -1.0;
	float time_in_wait = -1.0;
	float distance_to_junction = -1.0;

	bool stoping = false;
	bool stoped = false;



	bool isLider = false;
	bool is_sub_lider = false;

	int last_lider = -1;
	int next_lider = -1;
	int sub_next_lider = -1;
	int sub_lider = -1;

	float stop_time = -1.0;
	float enter_conflict_zone_time = -1.0;
	bool goingLeft = false;

	bool isLaneLider = false;
	bool isNewLider = false;
	bool isExtraTime = false;
	


	int tokens;
	int tokens_direction;

	int id_token_in_use;

	float last_priority;
	float priority;

	bool crossing;



	int id_cell_begin;
	int id_cell_end;
	int in_bloqued_queue;

};

class WaveShortMessage;

packet NodeInfoMessage extends WaveShortMessage {
    vehicleData data;
}