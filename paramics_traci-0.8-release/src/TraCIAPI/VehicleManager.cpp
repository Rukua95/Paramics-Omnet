#include "VehicleManager.h"

/* null singleton */
traci_api::VehicleManager* traci_api::VehicleManager::instance = nullptr;

traci_api::VehicleManager* traci_api::VehicleManager::getInstance()
{
    if (traci_api::VehicleManager::instance == nullptr)
        traci_api::VehicleManager::instance = new VehicleManager();

    return traci_api::VehicleManager::instance;
}

void traci_api::VehicleManager::deleteInstance()
{
    if (traci_api::VehicleManager::instance != nullptr)
        delete(traci_api::VehicleManager::instance);

    traci_api::VehicleManager::instance = nullptr;
}

/**
 * \brief Resets the internal temporary vectors for a new simulation timestep
 */
void traci_api::VehicleManager::reset()
{
    // clear temporary lists for a new timestep
    departed_vehicles.clear();
    arrived_vehicles.clear();
}

void traci_api::VehicleManager::packVehicleVariable(tcpip::Storage& input, tcpip::Storage& output) throw(NotImplementedError, std::runtime_error, NoSuchObjectError)
{
    uint8_t varID = input.readUnsignedByte();
    std::string s_vid = input.readString();

    //tcpip::Storage output;
    output.writeUnsignedByte(RES_GETVHCVAR);
    output.writeUnsignedByte(varID);
    output.writeString(s_vid);

    getVehicleVariable(s_vid, varID, output);

    //return output;
}

void traci_api::VehicleManager::getVehicleVariable(std::string vid, uint8_t varID, tcpip::Storage& output) throw(NotImplementedError, std::runtime_error, NoSuchObjectError)
{
    switch (varID)
    {
    case VAR_VHC_LIST:
        output.writeUnsignedByte(VTYPE_STRLST);
        output.writeStringList(getVehiclesInSim());
        break;

    case VAR_VHC_COUNT:
        output.writeUnsignedByte(VTYPE_INT);
        output.writeInt(currentVehicleCount());
        break;

    case VAR_VHC_SPEED:
        output.writeUnsignedByte(VTYPE_DOUBLE);
        output.writeDouble(getSpeed(vid));
        break;

    case VAR_VHC_POS:
        output.writeUnsignedByte(VTYPE_POSITION);
        {
            PositionalData pos = getPosition(vid);
            output.writeDouble(pos.x);
            output.writeDouble(pos.y);
        }
        break;

    case VAR_VHC_POS3D:
        output.writeUnsignedByte(VTYPE_POSITION3D);
        {
            PositionalData pos = getPosition(vid);
            output.writeDouble(pos.x);
            output.writeDouble(pos.y);
            output.writeDouble(pos.z);
        }
        break;

    case VAR_VHC_ANGLE:
        output.writeUnsignedByte(VTYPE_DOUBLE);
        output.writeDouble(getPosition(vid).bearing);
        break;

    case VAR_VHC_ROAD:
        output.writeUnsignedByte(VTYPE_STR);
        output.writeString(getRoadID(vid));
        break;

    case VAR_VHC_LANE:
        output.writeUnsignedByte(VTYPE_STR);
        output.writeString(getLaneID(vid));
        break;

    case VAR_VHC_LANEIDX:
        output.writeUnsignedByte(VTYPE_INT);
        output.writeInt(getLaneIndex(vid));
        break;

    case VAR_VHC_TYPE:
        output.writeUnsignedByte(VTYPE_STR);
        output.writeString(getVehicleType(vid));
        break;

    case VAR_VHC_LENGTH:
        output.writeUnsignedByte(VTYPE_DOUBLE);
        output.writeDouble(getDimensions(vid).length);
        break;

    case VAR_VHC_WIDTH:
        output.writeUnsignedByte(VTYPE_DOUBLE);
        output.writeDouble(getDimensions(vid).width);
        break;

    case VAR_VHC_HEIGHT:
        output.writeUnsignedByte(VTYPE_DOUBLE);
        output.writeDouble(getDimensions(vid).height);
        break;

    case VAR_VHC_SLOPE:
        output.writeUnsignedByte(VTYPE_DOUBLE);
        output.writeDouble(getPosition(vid).gradient);
        break;

    case VAR_VHC_SIGNALST:
        // the only signal state we can obtain from Paramics is brake light */
        {
            VEHICLE* v = findVehicle(vid);
            output.writeUnsignedByte(VTYPE_INT);
            if (qpg_VHC_braking(v))
                output.writeInt(0x08);
            else
                output.writeInt(0x00);
        }
        break;

    case VAR_VHC_LANEPOS:
        output.writeUnsignedByte(VTYPE_DOUBLE);
        {
            // Client asks about position from start of the lane
            // we know length of lane and distance from end, thus
            // position = length - distance_to_end

            VEHICLE* vhc = findVehicle(vid);
            LINK* current_link = qpg_VHC_link(vhc);
            float length = qpg_LNK_length(current_link);
            float distance = qpg_VHC_distance(vhc);
            output.writeDouble(length - distance);
        }
        break;

    case VAR_VHC_EDGES:
        output.writeUnsignedByte(VTYPE_STRLST);
        output.writeStringList(getRouteEdges(vid));
        break;

        /* not implemented yet*/
    case VAR_VHC_ROUTE:
    case VAR_VHC_ROUTEIDX:
    case VAR_VHC_COLOR:
    case VAR_VHC_DIST:
    case VAR_VHC_CO2:
    case VAR_VHC_CO:
    case VAR_VHC_HC:
    case VAR_VHC_PMX:
    case VAR_VHC_NOX:
    case VAR_VHC_FUELCONS:
    case VAR_VHC_NOISE:
    case VAR_VHC_ELECCONS:
    case VAR_VHC_BESTLANES:
    case VAR_VHC_STOPSTATE:
    case VAR_VHC_VMAX:
    case VAR_VHC_ACCEL:
    case VAR_VHC_DECEL:
    case VAR_VHC_TAU:
    case VAR_VHC_SIGMA:
    case VAR_VHC_SPDFACTOR:
    case VAR_VHC_SPEEDDEV:
    case VAR_VHC_VCLASS:
    case VAR_VHC_EMSCLASS:
    case VAR_VHC_SHAPE:
    case VAR_VHC_MINGAP:
    case VAR_VHC_WAITTIME:
    case VAR_VHC_NEXTTLS:
    case VAR_VHC_SPEEDMODE:
    case VAR_VHC_ALLOWEDSPD:
    case VAR_VHC_LINE:
    case VAR_VHC_PNUMBER:
    case VAR_VHC_VIAEDGES:
    case VAR_VHC_NONTRACISPD:
    case VAR_VHC_VALIDROUTE:
        throw NotImplementedError("Vehicle Variable not implemented: " + std::to_string(varID));
    default:
        throw std::runtime_error("No such variable: " + std::to_string(varID));
    }
}

void traci_api::VehicleManager::setVehicleState(tcpip::Storage& input)
{
    uint8_t varID = input.readUnsignedByte();

    switch (varID)
    {
    case STA_VHC_CHANGELANE:
        changeLane(input);
        break;
    case STA_VHC_SLOWDWN:
        slowDown(input);
        break;
    case STA_VHC_COLOUR:
        changeColour(input);
        break;
    case STA_VHC_SPEED:
        setSpeed(input);
        break;

    case STA_VHC_MAXSPEED:
        setMaxSpeed(input);
        break;

    case STA_VHC_CHANGEROUTE:
        setRoute(input);
        break;

    case STA_VHC_STOP:
    case STA_VHC_RESUME:
    case STA_VHC_CHANGETARGET:
    case STA_VHC_CHANGEROUTEID:
    case STA_VHC_CHANGEEDGETTIME:
    case STA_VHC_SIGNALSTATES:
    case STA_VHC_MOVETO:
    case STA_VHC_MOVETOXY:
    case STA_VHC_REROUTE:
    case STA_VHC_SPEEDMODE:
    case STA_VHC_SPEEDFACTOR:
    case STA_VHC_CHANGELANEMODE:
    case STA_VHC_ADD:
    case STA_VHC_ADDFULL:
    case STA_VHC_REMOVE:
    case STA_VHC_LENGTH:
    case STA_VHC_VHCCLASS:
    case STA_VHC_EMSCLASS:
    case STA_VHC_WIDTH:
    case STA_VHC_HEIGHT:
    case STA_VHC_MINGAP:
    case STA_VHC_SHAPECLASS:
    case STA_VHC_ACC:
    case STA_VHC_DEC:
    case STA_VHC_IMPERFECTION:
    case STA_VHC_TAU:
    case STA_VHC_TYPE:
    case STA_VHC_VIA:
        throw NotImplementedError("Vehicle State change not implemented: " + std::to_string(varID));
    default:
        throw std::runtime_error("No such State change: " + std::to_string(varID));
    }
}

/**
 * \brief Searchs the internal list of vehicles for a specific ID.
 * \param vid The vehicle ID to find.
 * \return A pointer to the corresponding Paramics Vehicle.
 */
VEHICLE* traci_api::VehicleManager::findVehicle(int vid) throw(NoSuchObjectError)
{
    //std::lock_guard<std::mutex> lock(vhc_lists_mutex);
    auto iterator = vehicles_in_sim.find(vid);
    if (iterator == vehicles_in_sim.end())
        throw NoSuchObjectError(std::to_string(vid));

    return iterator->second;
}

VEHICLE* traci_api::VehicleManager::findVehicle(std::string vid) throw(NoSuchObjectError)
{
    return findVehicle(std::stoi(vid));
}

void traci_api::VehicleManager::packVhcTypesVariable(tcpip::Storage& input, tcpip::Storage& output) throw(std::runtime_error, NotImplementedError)
{
    uint8_t varID = input.readUnsignedByte();
    std::string type_id = input.readString();

    output.writeUnsignedByte(RES_GETVTPVAR);
    output.writeUnsignedByte(varID);
    output.writeString(type_id);

    int type_index = -1;
    if (varID != VARLST && varID != VARCNT)
    {
        try
        {
            type_index = types_index_map.at(type_id);
        }
        // ReSharper disable once CppEntityNeverUsed
        catch (std::out_of_range& e)
        {
            throw std::runtime_error("No such type: " + type_id);
        }
    }

    getVhcTypesVariable(type_index, varID, output);
}

void traci_api::VehicleManager::getVhcTypesVariable(int type_id, uint8_t varID, tcpip::Storage& output) throw(std::runtime_error, NotImplementedError)
{
    if ((type_id < 0 || type_id > types_index_map.size()) && varID != VAR_VHC_LIST && varID != VAR_VHC_COUNT)
        throw std::runtime_error("Invalid type ID " + std::to_string(type_id) + " for variable " + std::to_string(varID));

    switch (varID)
    {
    case VAR_VHC_LIST:
        {
            std::vector<std::string> type_names;
            for (auto kv : types_index_map)
                type_names.push_back(kv.first);
            output.writeUnsignedByte(VTYPE_STRLST);
            output.writeStringList(type_names);
        }
        break;
    case VAR_VHC_COUNT:
        output.writeUnsignedByte(VTYPE_INT);
        output.writeInt(types_index_map.size());
        break;
    case VAR_VHC_LENGTH:
        output.writeUnsignedByte(VTYPE_FLOAT);
        output.writeDouble(qpg_VTP_length(type_id));
        break;
    case VAR_VHC_WIDTH:
        output.writeUnsignedByte(VTYPE_FLOAT);
        output.writeDouble(qpg_VTP_width(type_id));
        break;
    case VAR_VHC_HEIGHT:
        output.writeUnsignedByte(VTYPE_FLOAT);
        output.writeDouble(qpg_VTP_height(type_id));
        break;
    case VAR_VHC_ACCEL:
        output.writeUnsignedByte(VTYPE_FLOAT);
        output.writeDouble(qpg_VTP_acceleration(type_id));
        break;
    case VAR_VHC_VMAX:
        output.writeUnsignedByte(VTYPE_FLOAT);
        output.writeDouble(qpg_VTP_maxSpeed(type_id));
        break;
    case VAR_VHC_DECEL:
        output.writeUnsignedByte(VTYPE_FLOAT);
        output.writeDouble(qpg_VTP_deceleration(type_id));
        break;

    case VAR_VHC_TAU:
    case VAR_VHC_SIGMA:
    case VAR_VHC_SPDFACTOR:
    case VAR_VHC_SPEEDDEV:
    case VAR_VHC_VCLASS:
    case VAR_VHC_EMSCLASS:
    case VAR_VHC_SHAPE:
    case VAR_VHC_MINGAP:
    case VAR_VHC_COLOR:
    case VAR_VHC_MAXLATSPD:
    case VAR_VHC_LATGAP:
    case VAR_VHC_LATALIGN:
        throw NotImplementedError("Variable not implemented: " + std::to_string(varID));
    default:
        throw std::runtime_error("No such variable (" + std::to_string(varID) + ")");
    }
}

int traci_api::VehicleManager::rerouteVehicle(VEHICLE* vhc, LINK* lnk)
{
    if (0 == qpg_VHC_uniqueID(vhc)) // dummy vhc
        return 0;

    // check if the vehicle has a special route
    std::unordered_map<LINK*, int>* exit_map;
    try
    {
        exit_map = vhc_routes.at(vhc);
    }
    // ReSharper disable once CppEntityNeverUsed
    catch (std::out_of_range& e)
    {
        // no special route, return default
        return 0;
    }

    int next_exit = 0;
    try
    {
        next_exit = exit_map->at(lnk);
    }
    // ReSharper disable once CppEntityNeverUsed
    catch (std::out_of_range& e)
    {
        // outside route, clear 
        exit_map->clear();
        delete exit_map;
        vhc_routes.erase(vhc);
    }

    return next_exit;
}

void traci_api::VehicleManager::routeReEval(VEHICLE* vhc)
{
    try
    {
        // if car has a custom route, we need to re-eval
        // otherwise do nothing
        vhc_routes.at(vhc);
        qps_VHC_destination(vhc, qpg_VHC_destination(vhc), 0);
    }
    // ReSharper disable once CppEntityNeverUsed
    catch (std::out_of_range& e)
    {
    }
}

/**
 * \brief Checks if a vehicle has a custom speed controller set (for instance, in the
 * case of SetSpeed commands). If the vehicle does have a custom controller, this method
 * returns true, and sets the new speed value on the speed parameter. Otherwise, the method
 * returns false and writes nothing on the speed parameter.
 * This method also removes finished controllers after triggering them.
 * \param vhc The vehicle to check for custom speed controllers.
 * \param speed Return variable for next timesteps speed value.
 * \return True if the vehicle has a controller, false if it doesn't.
 */
bool traci_api::VehicleManager::speedControlOverride(VEHICLE* vhc, float& speed)
{
    BaseSpeedController* controller;
    try
    {
        controller = speed_controllers.at(vhc);
        speed = controller->nextTimeStep();

        if (!controller->repeat())
        {
            speed_controllers.erase(vhc);
            delete controller;
        }

        return true;
    }
    catch (std::out_of_range& e)
    {
        return false;
    }
}

traci_api::VehicleManager::VehicleManager()
{
    int type_n = qpg_NET_vehicleTypes();
    for (int i = 1; i <= type_n; i++)
        types_index_map[qpg_VTP_name(i)] = i;
}


/**
 * \brief Finds the exit corresponding to a specific link in a node.
 * \param node The node to exit from.
 * \param exit_link The link whose exit index we want.
 * \return The exit index for the specified link.
 */
int traci_api::VehicleManager::findExit(NODE* node, LINK* exit_link) throw(NoSuchObjectError)
{
    int n_exits = qpg_NDE_exitLinks(node);
    for (int i = 1; i <= n_exits; i++)
    {
        LINK* current_exit = qpg_NDE_link(node, i);
        if (current_exit == exit_link) return i;
    }

    throw NoSuchObjectError("Could not find the specified link in the given node");
}

/**
 * \brief Handles delayed time_triggers and repetitive timestep triggers. For example, changing back to the original lane 
 * after a set time after a lane change command.
 */
void traci_api::VehicleManager::handleDelayedTriggers()
{
    // handle lane set triggers
    debugPrint("Handling vehicle triggers: lane set triggers");
    for (auto kv = lane_set_triggers.begin(); kv != lane_set_triggers.end();)
    {
        kv->second->handleTrigger();

        /* check if need repeating */
        if (!kv->second->repeat())
        {
            delete kv->second;
            kv = lane_set_triggers.erase(kv);
        }
        else
            ++kv;
    }

    debugPrint("Handling vehicle triggers: done");
}


/**
 * \brief Signals the departure of a vehicle into the network.
 * \param vehicle A pointer to the Paramics Vehicle.
 */
void traci_api::VehicleManager::vehicleDepart(VEHICLE* vehicle)
{
    //std::lock_guard<std::mutex> lock(vhc_lists_mutex);
    departed_vehicles.push_back(vehicle);
    vehicles_in_sim[qpg_VHC_uniqueID(vehicle)] = vehicle;
}

/**
 * \brief Signals the arrival of a vehicle from the network.
 * \param vehicle A pointer to the Paramics Vehicle.
 */
void traci_api::VehicleManager::vehicleArrive(VEHICLE* vehicle)
{
    //std::lock_guard<std::mutex> lock(vhc_lists_mutex);
    arrived_vehicles.push_back(vehicle);
    vehicles_in_sim.erase(qpg_VHC_uniqueID(vehicle));
    speed_controllers.erase(vehicle);
}

/**
 * \brief Requests the list of recently departed vehicles.
 * \return A vector of strings containing every vehicle that has departed in the last timestep.
 */
std::vector<std::string> traci_api::VehicleManager::getDepartedVehicles()
{
    std::vector<std::string> ids;
    //std::lock_guard<std::mutex> lock(vhc_lists_mutex);
    for (VEHICLE* v : departed_vehicles)
        ids.push_back(std::to_string(qpg_VHC_uniqueID(v)));

    return ids;
}

/**
* \brief Requests the list of recently arrived vehicles.
* \return A vector of strings containing every vehicle that has arrived in the last timestep.
*/
std::vector<std::string> traci_api::VehicleManager::getArrivedVehicles()
{
    std::vector<std::string> ids;
    //std::lock_guard<std::mutex> lock(vhc_lists_mutex);
    for (VEHICLE* v : arrived_vehicles)
        ids.push_back(std::to_string(qpg_VHC_uniqueID(v)));

    return ids;
}

int traci_api::VehicleManager::getDepartedVehicleCount() const
{
    //std::lock_guard<std::mutex> lock(vhc_lists_mutex);
    return departed_vehicles.size();
}

int traci_api::VehicleManager::getArrivedVehicleCount() const
{
    //std::lock_guard<std::mutex> lock(vhc_lists_mutex);
    return arrived_vehicles.size();
}

int traci_api::VehicleManager::currentVehicleCount() const
{
    //std::lock_guard<std::mutex> lock(vhc_lists_mutex);
    return vehicles_in_sim.size();
}

std::vector<std::string> traci_api::VehicleManager::getVehiclesInSim()
{
    std::vector<std::string> ids;
    //std::lock_guard<std::mutex> lock(vhc_lists_mutex);
    for (auto iterator : vehicles_in_sim)
        ids.push_back(std::to_string(iterator.first));

    return ids;
}

/**
 * \brief Requests the speed of a specific vehicle.
 * \param vid The ID of the vehicle.
 * \return The current speed in m/s.
 */
float traci_api::VehicleManager::getSpeed(std::string vid) throw(NoSuchObjectError)
{
    //double mph = qpg_VHC_speed(this->findVehicle(vid)) * qpg_UTL_toExternalSpeed();
    //return KPH2MS(mph);
    return qpg_VHC_speed(findVehicle(vid));
}

/**
 * \brief Requests the 3-dimensional position of the vehicle in the simulation.
 * \param vid The ID of the vehicle.
 * \return A Vector3D object representing the position of the vehicle.
 */
PositionalData traci_api::VehicleManager::getPosition(std::string vid) throw(NoSuchObjectError)
{
    float x;
    float y;
    float z;
    float b;
    float g;

    VEHICLE* vhc = this->findVehicle(vid);
    LINK* lnk = qpg_VHC_link(vhc);

    qpg_POS_vehicle(vhc, lnk, &x, &y, &z, &b, &g);

    return PositionalData(x, y, z, b, g);
}

DimensionalData traci_api::VehicleManager::getDimensions(std::string vid) throw(NoSuchObjectError)
{
    VEHICLE* vhc = this->findVehicle(vid);
    return DimensionalData(qpg_VHC_height(vhc), qpg_VHC_length(vhc), qpg_VHC_width(vhc));
}

std::string traci_api::VehicleManager::getRoadID(std::string vid) throw(NoSuchObjectError)
{
    VEHICLE* vhc = this->findVehicle(vid);
    LINK* lnk = qpg_VHC_link(vhc);

    return qpg_LNK_name(lnk);
}

std::string traci_api::VehicleManager::getLaneID(std::string vid) throw(NoSuchObjectError)
{
    VEHICLE* vhc = this->findVehicle(vid);
    LINK* lnk = qpg_VHC_link(vhc);

    return std::string(qpg_LNK_name(lnk)) + "." + std::to_string(qpg_VHC_lane(vhc));
}

int traci_api::VehicleManager::getLaneIndex(std::string vid) throw(NoSuchObjectError)
{
    return qpg_VHC_lane(this->findVehicle(vid));
}

std::vector<std::string> traci_api::VehicleManager::getRouteEdges(std::string vid) throw(NoSuchObjectError)
{
    // get current and next two edges
    // also include destination zone in message

    VEHICLE* vhc = findVehicle(vid);
    std::vector<std::string> result;
    LINK* current_link = qpg_VHC_link(vhc);

    result.push_back(qpg_LNK_name(current_link));

    int next_exit = qpg_VHC_nextExit(vhc);
    LINK* next_link = qpg_NDE_link(qpg_LNK_nodeEnd(current_link), next_exit);

    if (next_link)
    {
        result.push_back(qpg_LNK_name(next_link));

        int next_next_exit = qpg_VHC_nextNextExit(vhc);
        LINK* next_next_link = qpg_NDE_link(qpg_LNK_nodeEnd(next_link), next_next_exit);

        if (next_next_link)
            result.push_back(qpg_LNK_name(next_next_link));
    }

    int dest_index = qpg_VHC_destination(vhc);
    result.push_back(std::to_string(dest_index));

    return result;
}

std::string traci_api::VehicleManager::getVehicleType(std::string vid) throw(NoSuchObjectError)
{
    return std::to_string(qpg_VHC_type(this->findVehicle(vid)));
}

void traci_api::VehicleManager::changeLane(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error)
{
    /* change lane message format
    *
    * | type: compound	| ubyte
    * | items: 2		| int
    * ------------------
    * | type: byte		| ubyte
    * | lane id			| ubyte
    * ------------------
    * | type: int		| ubyte
    * | duration		| int
    */

    std::string vhcid = input.readString();


    debugPrint("Vehicle " + vhcid + " changing lanes.");

    if (input.readUnsignedByte() != VTYPE_COMPOUND || input.readInt() != 2)
        throw std::runtime_error("Malformed TraCI message");

    int8_t new_lane = 0;
    if (!readTypeCheckingByte(input, new_lane))
        throw std::runtime_error("Malformed TraCI message");

    int duration = 0;
    if (!readTypeCheckingInt(input, duration))
        throw std::runtime_error("Malformed TraCI message");

    VEHICLE* vhc = findVehicle(std::stoi(vhcid));

    // first, check if there already exists a lane change trigger
    // if there is, delete it

    try
    {
        LaneSetTrigger* trigger = lane_set_triggers.at(vhc);
        delete trigger;
        lane_set_triggers.erase(vhc);
    }
    // ReSharper disable once CppEntityNeverUsed
    catch (std::out_of_range& e) { /* no previous trigger, ok */ }

    if (duration < 0)
    // negative duration: remove previous lane set command, we cant just return now
        return;
    else if (duration == 0)
    // "infinite" duration
        duration = INT32_MAX - Simulation::getInstance()->getCurrentTimeMilliseconds();

    lane_set_triggers[vhc] = new LaneSetTrigger(vhc, new_lane, duration);
    lane_set_triggers[vhc]->handleTrigger();
}

void traci_api::VehicleManager::slowDown(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error)
{
    /* slow down message format
    *
    * | type: compound	| ubyte
    * | items: 2		| int
    * ------------------
    * | type: double	| ubyte
    * | speed			| ubyte
    * ------------------
    * | type: int		| ubyte
    * | duration		| int
    */

    std::string vhcid = input.readString();
    VEHICLE* vhc = findVehicle(std::stoi(vhcid));

    debugPrint("Vehicle " + vhcid + " slowing down.");

    if (input.readUnsignedByte() != VTYPE_COMPOUND || input.readInt() != 2)
        throw std::runtime_error("Malformed TraCI message");

    double target_speed = 0;
    if (!readTypeCheckingDouble(input, target_speed))
        throw std::runtime_error("Malformed TraCI message");

    int duration = 0;
    if (!readTypeCheckingInt(input, duration))
        throw std::runtime_error("Malformed TraCI message");

    if (duration <= 0 && target_speed >= 0)
        throw std::runtime_error("Malformed TraCI message");

    try
    {
        delete speed_controllers.at(vhc);
    }
    catch (std::out_of_range& e)
    {
    }

    if (target_speed >= 0)
        speed_controllers[vhc] = new LinearSpeedChangeController(vhc, target_speed, duration);
    else
    {
        speed_controllers.erase(vhc);
        if (qpg_VHC_stopped(vhc))
            qps_VHC_stopped(vhc, PFALSE);
    }
}

void traci_api::VehicleManager::changeColour(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error)
{
    /* colour change message format
     * 
     * | string | ubyte | ubyte | ubyte | ubyte | ubyte |
     *	 vhc_id	  Type		R		G		B		A
    */

    std::string vhcid = input.readString();
    VEHICLE* vhc = findVehicle(std::stoi(vhcid));

    uint32_t hex = 0x000000;
    if (!readTypeCheckingColor(input, hex))
        throw std::runtime_error("Malformed TraCI message");

    /* change vehicle color */
    qps_DRW_vehicleColour(vhc, hex);
}

void traci_api::VehicleManager::setSpeed(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error)
{
    /* set speed message format
     * | string | ubyte | double |
     *   vhc_id	  Type	  speed
     */

    std::string vhcid = input.readString();
    VEHICLE* vhc = findVehicle(std::stoi(vhcid));

    double speed = 0;
    if (!readTypeCheckingDouble(input, speed))
        throw std::runtime_error("Malformed TraCI message");

    debugPrint("Setting speed " + std::to_string(speed) + " for vehicle " + vhcid);

    try
    {
        delete speed_controllers.at(vhc);
    }
    catch (std::out_of_range& e)
    {
    }

    if (speed >= 0)
        speed_controllers[vhc] = new HoldSpeedController(vhc, speed);
    else
    {
        speed_controllers.erase(vhc);
        if (qpg_VHC_stopped(vhc))
            qps_VHC_stopped(vhc, PFALSE);
    }
}

void traci_api::VehicleManager::setMaxSpeed(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error)
{
    /* set maxspeed message format
    * | string | ubyte | double |
    *   vhc_id	  Type	  speed
    */

    std::string vhcid = input.readString();
    VEHICLE* vhc = findVehicle(std::stoi(vhcid));

    double speed = 0;
    if (!readTypeCheckingDouble(input, speed))
        throw std::runtime_error("Malformed TraCI message");

    /* speed is in m/s */
    qps_VHC_maxSpeed(vhc, speed);
}

void traci_api::VehicleManager::setRoute(tcpip::Storage& input) throw(NoSuchObjectError, std::runtime_error)
{
    /* set new route message format
     * | string | ubyte | strlist |
     *   vhc_id | Type  | edges   |
     */

    std::string vhcid = input.readString();
    VEHICLE* vhc = findVehicle(vhcid);

    debugPrint("Changing Vehicle route");

    std::vector<std::string> edges;
    if (!readTypeCheckingStringList(input, edges))
        throw std::runtime_error("Malformed TraCI message");

    /* first, find current edge for the vehicle, and find it in the given list
     * (list needs to come in order) */

    /* this call only works if we are not on a junction */
    /*if (qpg_VHC_onNode(vhc))
        throw std::runtime_error("Can't change route while on a junction"); */

    std::string current_edge = qpg_LNK_name(qpg_VHC_link(vhc));
    for (auto i = edges.begin(); i != edges.end();)
    {
        if (*i != current_edge)
            i = edges.erase(i);
        else
            break;
    }

    if (edges.size() == 0)
    {
        debugPrint("Invalid route: could not find current edge");
        throw std::runtime_error("Invalid route: could not find current edge");
    }


    /* for each edge, find the next exit corresponding to the next edge */
    std::unordered_map<LINK*, int>* exit_map = new std::unordered_map<LINK*, int>;
    for (int i = 0; i < edges.size() - 1; i++)
    {
        std::string edge_a = edges[i];
        std::string edge_b = edges[i + 1];
        LINK* link_a = qpg_NET_link(&edge_a[0u]);
        LINK* link_b = qpg_NET_link(&edge_b[0u]);

        int exit_i = -1;

        int n_links = qpg_LNK_links(link_a);
        for (int j = 1; j <= n_links; j++)
        {
            if (link_b == qpg_LNK_link(link_a, j))
                exit_i = j;
        }

        if (exit_i < 0)
            throw std::runtime_error("Non-contigous route.");

        /* map A to the corresponding exit */
        (*exit_map)[link_a] = exit_i;
    }

    /* program the route */
    vhc_routes[vhc] = exit_map;

    /* tell paramics to reevaluate route */
    qps_VHC_destination(vhc, qpg_VHC_destination(vhc), 0);
}
