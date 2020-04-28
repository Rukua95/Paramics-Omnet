#pragma once
#ifdef _DEBUG
#define DEBUG true
#else
#define DEBUG false
#endif

#include <stdint.h>

//Server definitions
#define TRACIAPI_VERSION 10
#define PVEINS_VERSION "0.5rc1"
#define PVEINS_COPYRIGHT "Copyright 2017 Manuel Olguín <molguin@dcc.uchile.cl>"
#define PVEINS_LICENSE "This plugin is distributed under the BSD 3-Clause license."

#define POSITION_EPS 0.1f
#define NUMERICAL_EPS 0.001f

#define RECALCULATE_DELAY_MINS 5


namespace traci_api
{
    //variable types
    static const uint8_t VTYPE_UBYTE = 0x07;
    static const uint8_t VTYPE_BYTE = 0x08;
    static const uint8_t VTYPE_INT = 0x09;
    static const uint8_t VTYPE_FLOAT = 0x0a;
    static const uint8_t VTYPE_DOUBLE = 0x0b;
    static const uint8_t VTYPE_STR = 0x0c;
    static const uint8_t VTYPE_STRLST = 0x0e;
    static const uint8_t VTYPE_COMPOUND = 0x0f;

    static const uint8_t VTYPE_POSITION = 0x01;
    static const uint8_t VTYPE_POSITION3D = 0x03;
    static const uint8_t VTYPE_ROADMAPPOS = 0x04;
    static const uint8_t VTYPE_LONLAT = 0x00;
    static const uint8_t VTYPE_LONLATALT = 0x02;
    static const uint8_t VTYPE_BOUNDBOX = 0x05;
    static const uint8_t VTYPE_POLYGON = 0x06;
    static const uint8_t VTYPE_TLIGHTPHASELST = 0x0d;
    static const uint8_t VTYPE_COLOR = 0x11;


    //Status responses:
    static const uint8_t STATUS_OK = 0x00;
    static const uint8_t STATUS_ERROR = 0x0ff;
    static const uint8_t STATUS_NIMPL = 0x01;

    // standard variable fetch types
    static const uint8_t VARLST = 0x00;
    static const uint8_t VARCNT = 0x01;

    //Command definitions:
    //Simulation control:
    static const uint8_t CMD_GETVERSION = 0x00;
    static const uint8_t CMD_SIMSTEP = 0x02;
    static const uint8_t CMD_SHUTDOWN = 0x7f;
    //------------------
    //Value retrieval:
    static const uint8_t CMD_GETSIMVAR = 0xab;
    static const uint8_t RES_GETSIMVAR = 0xbb;

    static const uint8_t CMD_GETVHCVAR = 0xa4;
    static const uint8_t RES_GETVHCVAR = 0xb4;

    static const uint8_t CMD_GETVTPVAR = 0xa5;
    static const uint8_t RES_GETVTPVAR = 0xb5;

    static const uint8_t CMD_GETLNKVAR = 0xaa;
    static const uint8_t RES_GETLNKVAR = 0xba;
    static const uint8_t CMD_GETNDEVAR = 0xa9;
    static const uint8_t RES_GETNDEVAR = 0xb9;

    static const uint8_t CMD_GETRTEVAR = 0xa6;
    static const uint8_t RES_GETRTEVAR = 0xb6;

    // polygons
    static const uint8_t CMD_GETPOLVAR = 0xa8;
    static const uint8_t RES_GETPOLVAR = 0xb8;

    //Change State:
    static const uint8_t CMD_SETVHCSTATE = 0xc4;


    // subscription commands and responses:
    static const uint8_t CMD_SUB_INDVAR = 0xd0;
    static const uint8_t CMD_SUB_MULTVAR = 0xd1;
    static const uint8_t CMD_SUB_TLIGHTVAR = 0xd2;
    static const uint8_t CMD_SUB_LANEVAR = 0xd3;
    static const uint8_t CMD_SUB_VHCVAR = 0xd4;
    static const uint8_t CMD_SUB_VHCTYPEVAR = 0xd5;
    static const uint8_t CMD_SUB_RTEVAR = 0xd6;
    static const uint8_t CMD_SUB_POIVAR = 0xd7;
    static const uint8_t CMD_SUB_POLVAR = 0xd8;
    static const uint8_t CMD_SUB_JUNCTVAR = 0xd9;
    static const uint8_t CMD_SUB_EDGEVAR = 0xda;
    static const uint8_t CMD_SUB_SIMVAR = 0xdb;

    static const uint8_t RES_SUB_INDVAR = 0xe0;
    static const uint8_t RES_SUB_MULTVAR = 0xe1;
    static const uint8_t RES_SUB_TLIGHTVAR = 0xe2;
    static const uint8_t RES_SUB_LANEVAR = 0xe3;
    static const uint8_t RES_SUB_VHCVAR = 0xe4;
    static const uint8_t RES_SUB_VHCTYPEVAR = 0xe5;
    static const uint8_t RES_SUB_RTEVAR = 0xe6;
    static const uint8_t RES_SUB_POIVAR = 0xe7;
    static const uint8_t RES_SUB_POLVAR = 0xe8;
    static const uint8_t RES_SUB_JUNCTVAR = 0xe9;
    static const uint8_t RES_SUB_EDGEVAR = 0xea;
    static const uint8_t RES_SUB_SIMVAR = 0xeb;
}
