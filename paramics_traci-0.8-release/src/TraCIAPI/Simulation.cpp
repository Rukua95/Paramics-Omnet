#include "Simulation.h"

/*
 * This class abstracts a Paramics traffic simulation, providing helper methods for
 * interacting with the simulator itself.
 */

traci_api::Simulation* traci_api::Simulation::instance = nullptr;

traci_api::Simulation::Simulation() : stepcnt(0)
{
}

traci_api::Simulation::~Simulation()
{
}

float traci_api::Simulation::getCurrentTimeSeconds()
{
    return qpg_CFG_simulationTime();
}

int traci_api::Simulation::getCurrentTimeMilliseconds()
{
    return static_cast<int>(this->getCurrentTimeSeconds() * 1000);
}

int traci_api::Simulation::getTimeStepSizeMilliseconds()
{
    return static_cast<int>(qpg_CFG_timeStep() * 1000.0f);
}

void traci_api::Simulation::getRealNetworkBounds(double& llx, double& lly, double& urx, double& ury)
{
    /*
     * Paramics qpg_POS_network() function, which should return the network bounds, does not make sense.
     * It returns coordinates which leave basically the whole network outside of its own bounds. 
     * 
     * Thus, we'll have to "bruteforce" the positional data for the network bounds.
     */

    // get all relevant elements in the network, and all their coordinates

    std::vector<float> x;
    std::vector<float> y;

    int node_count = qpg_NET_nodes();
    int link_count = qpg_NET_links();
    int zone_count = qpg_NET_zones();

    float tempX, tempY, tempZ;

    for (int i = 1; i <= node_count; i++)
    {
        NODE* node = qpg_NET_nodeByIndex(i);
        qpg_POS_node(node, &tempX, &tempY, &tempZ);

        x.push_back(tempX);
        y.push_back(tempY);
    }

    for (int i = 1; i <= zone_count; i++)
    {
        ZONE* zone = qpg_NET_zone(i);
        int vertices = qpg_ZNE_vertices(zone);
        for (int j = 1; j <= vertices; j++)
        {
            qpg_POS_zoneVertex(zone, j, &tempX, &tempY, &tempZ);

            x.push_back(tempX);
            y.push_back(tempY);
        }
    }

    for (int i = 1; i <= link_count; i++)
    {
        // links are always connected to zones or nodes, so we only need
        // to get position data from those that are curved

        LINK* lnk = qpg_NET_linkByIndex(i);
        if (!qpg_LNK_arc(lnk) && !qpg_LNK_arcLeft(lnk))
            continue;

        // arc are perfect sections of circles, thus we only need the start, end and middle point (for all lanes)
        float len = qpg_LNK_length(lnk);
        int lanes = qpg_LNK_lanes(lnk);

        float g, b;

        for (int j = 1; j <= lanes; j++)
        {
            // start points
            qpg_POS_link(lnk, j, 0, &tempX, &tempY, &tempZ, &b, &g);

            x.push_back(tempX);
            y.push_back(tempY);

            // middle points
            qpg_POS_link(lnk, j, len / 2.0, &tempX, &tempY, &tempZ, &b, &g);

            x.push_back(tempX);
            y.push_back(tempY);

            // end points
            qpg_POS_link(lnk, j, len, &tempX, &tempY, &tempZ, &b, &g);

            x.push_back(tempX);
            y.push_back(tempY);
        }
    }


    // we have all the coordinates, now get maximums and minimums
    // add some wiggle room as well, just in case
    urx = *std::max_element(x.begin(), x.end()) + 100;
    llx = *std::min_element(x.begin(), x.end()) - 100;
    ury = *std::max_element(y.begin(), y.end()) + 100;
    lly = *std::min_element(y.begin(), y.end()) - 100;
}

traci_api::Simulation* traci_api::Simulation::getInstance()
{
    if (!instance)
        instance = new Simulation();

    return instance;
}

void traci_api::Simulation::deleteInstance()
{
    if (instance != nullptr)
        delete(instance);

    instance = nullptr;
}

bool traci_api::Simulation::packSimulationVariable(uint8_t varID, tcpip::Storage& result_store)
{
    debugPrint("Fetching SIMVAR " + std::to_string(varID));

    result_store.writeUnsignedByte(RES_GETSIMVAR);
    result_store.writeUnsignedByte(varID);
    result_store.writeString("");
    try
    {
        getSimulationVariable(varID, result_store);
    }
    catch (...)
    {
        return false;
    }
    return true;
}

void traci_api::Simulation::getSimulationVariable(uint8_t varID, tcpip::Storage& result)
{
    VehicleManager* vhcman = traci_api::VehicleManager::getInstance();

    switch (varID)
    {
    case VAR_SIMTIME:
        result.writeUnsignedByte(VTYPE_INT);
        result.writeInt(this->getCurrentTimeMilliseconds());
        break;
    case VAR_DEPARTEDVHC_CNT:
        result.writeUnsignedByte(VTYPE_INT);
        result.writeInt(vhcman->getDepartedVehicleCount());
        break;
    case VAR_DEPARTEDVHC_LST:
        result.writeUnsignedByte(VTYPE_STRLST);
        result.writeStringList(vhcman->getDepartedVehicles());
        break;
    case VAR_ARRIVEDVHC_CNT:
        result.writeUnsignedByte(VTYPE_INT);
        result.writeInt(vhcman->getArrivedVehicleCount());
        break;
    case VAR_ARRIVEDVHC_LST:
        result.writeUnsignedByte(VTYPE_STRLST);
        result.writeStringList(vhcman->getArrivedVehicles());
        break;
    case VAR_TIMESTEPSZ:
        result.writeUnsignedByte(VTYPE_INT);
        result.writeInt(getTimeStepSizeMilliseconds());
        break;
    case VAR_NETWORKBNDS:
        result.writeUnsignedByte(VTYPE_BOUNDBOX);
        {
            double llx, lly, urx, ury;
            this->getRealNetworkBounds(llx, lly, urx, ury);

            result.writeDouble(llx);
            result.writeDouble(lly);
            result.writeDouble(urx);
            result.writeDouble(ury);
        }
        break;
        // we don't have teleporting vehicles in Paramics, nor parking (temporarily at least)
    case VAR_VHCENDTELEPORT_CNT:
    case VAR_VHCSTARTTELEPORT_CNT:
    case VAR_VHCSTARTPARK_CNT:
    case VAR_VHCENDPARK_CNT:
        result.writeUnsignedByte(VTYPE_INT);
        result.writeInt(0);
        break;

    case VAR_VHCENDTELEPORT_LST:
    case VAR_VHCSTARTTELEPORT_LST:
    case VAR_VHCSTARTPARK_LST:
    case VAR_VHCENDPARK_LST:
        result.writeUnsignedByte(VTYPE_STRLST);
        result.writeStringList(std::vector<std::string>());
        break;
    default:
        throw std::runtime_error("Unimplemented variable " + std::to_string(varID));
    }
}
