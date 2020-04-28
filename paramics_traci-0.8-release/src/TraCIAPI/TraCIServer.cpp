#include "TraCIServer.h"

/*
 * This class abstracts a server for the TraCI protocol.
 * It binds to and listens on a port for incoming TraCI commands, and replies with the appropiate data.
 */


/**
 * \brief Standard constructor.
 * \param port The port on which the server should listen for incoming requests.
 */
traci_api::TraCIServer::TraCIServer(int port): ssocket(port), incoming_size(0), running(false), port(port), multiple_timestep(false), target_time(0)
{
    ssocket.set_blocking(true);
}


traci_api::TraCIServer::~TraCIServer()
{
}

/**
 * \brief Starts this instance, binding it to a port and awaiting connections.
 */
void traci_api::TraCIServer::waitForConnection()
{
    running = true;
    infoPrint("Awaiting connections on port " + std::to_string(port));

    {
        std::lock_guard<std::mutex> lock(socket_lock);
        ssocket.accept();
    }

    infoPrint("Accepted connection");
}

/**
 * \brief Closes the socket, severing all connections.
 */
void traci_api::TraCIServer::close()
{
    debugPrint("Closing connections");
    running = false;
    ssocket.close();

    VehicleManager::deleteInstance();
    for (auto i : subs)
        delete i;

    debugPrint("Server succesfully shut down");
}


void traci_api::TraCIServer::preStep()
{
    std::lock_guard<std::mutex> lock(socket_lock);
    if (multiple_timestep && Simulation::getInstance()->getCurrentTimeMilliseconds() < target_time)
    {
        VehicleManager::getInstance()->reset();
        return;
    }

    multiple_timestep = false;
    target_time = 0;

    
    tcpip::Storage cmdStore; // individual commands in the message

    debugPrint("Waiting for incoming commands from the TraCI client...");

    // receive and parse messages until we get a simulation step command
    while (running && ssocket.receiveExact(incoming))
    {
        incoming_size = incoming.size();

        debugPrint("Got message of length " + std::to_string(incoming_size));
        //debugPrint("Incoming: " + incoming.hexDump());


        /* Multiple commands may arrive at once in one message,
        * divide them into multiple storages for easy handling */
        while (incoming_size > 0 && incoming.valid_pos())
        {
            uint8_t cmdlen = incoming.readUnsignedByte();
            cmdStore.writeUnsignedByte(cmdlen);

            debugPrint("Got command of length " + std::to_string(cmdlen));


            for (uint8_t i = 0; i < cmdlen - 1; i++)
                cmdStore.writeUnsignedByte(incoming.readUnsignedByte());

            bool simstep = this->parseCommand(cmdStore);
            cmdStore.reset();

            // if the received command was a simulation step command, return so that 
            // Paramics can do its thing.
            if (simstep)
            {
                VehicleManager::getInstance()->reset();
                return;
            }
        }

        this->sendResponse();
        incoming.reset();
        outgoing.reset();
    }
}

void traci_api::TraCIServer::postStep()
{
    // after each step, have VehicleManager update its internal state
    VehicleManager::getInstance()->handleDelayedTriggers();

    if (multiple_timestep && Simulation::getInstance()->getCurrentTimeMilliseconds() < target_time)
        return;

    // after a finishing a simulation step command (completely), collect subscription results and 
    // check if there are commands remaining in the incoming storage
    this->writeStatusResponse(CMD_SIMSTEP, STATUS_OK, "");

    // handle subscriptions after simstep command
    tcpip::Storage subscriptions;
    this->processSubscriptions(subscriptions);
    outgoing.writeStorage(subscriptions);

    // finish parsing the message we got before the simstep command
    tcpip::Storage cmdStore;
    /* Multiple commands may arrive at once in one message,
    * divide them into multiple storages for easy handling */
    while (incoming_size > 0 && incoming.valid_pos())
    {
        uint8_t cmdlen = incoming.readUnsignedByte();
        cmdStore.writeUnsignedByte(cmdlen);

        debugPrint("Got command of length " + std::to_string(cmdlen));


        for (uint8_t i = 0; i < cmdlen - 1; i++)
            cmdStore.writeUnsignedByte(incoming.readUnsignedByte());

        bool simstep = this->parseCommand(cmdStore);
        cmdStore.reset();

        // weird, two simstep commands in one message?
        if (simstep)
        {
            if(!multiple_timestep)
            {
                multiple_timestep = true;
                Simulation* sim = Simulation::getInstance();
                target_time = sim->getCurrentTimeMilliseconds() + sim->getTimeStepSizeMilliseconds();
            }
            VehicleManager::getInstance()->reset();
            return;
        }
    }

    // finally, send response and return
    std::lock_guard<std::mutex> lock(socket_lock);
    if (!running)
    {
        // got shutdown command, gg bai bai
        qps_SIM_close();
        return;
    }

    this->sendResponse();
    incoming.reset();
    outgoing.reset();
}

/**
 * \brief Parses an incoming command according to the TraCI protocol specifications.
 * \param storage A tcpip::Storage object which contains a single TraCI command.
 */
bool traci_api::TraCIServer::parseCommand(tcpip::Storage& storage)
{
    debugPrint("Parsing command");

    uint8_t cmdLen = storage.readUnsignedByte();
    uint8_t cmdId = storage.readUnsignedByte();
    tcpip::Storage state;

    debugPrint("Command length: " + std::to_string(cmdLen));
    debugPrint("Command ID: " + std::to_string(cmdId));

    if (cmdId >= CMD_SUB_INDVAR && cmdId <= CMD_SUB_SIMVAR)
    {
        // subscription
        // | begin Time | end Time | Object ID | Variable Number | The list of variables to return

        debugPrint("Subscribing to " + std::to_string(cmdId));


        int btime = storage.readInt();
        debugPrint("Start time: " + std::to_string(btime));

        int etime = storage.readInt();
        debugPrint("End time: " + std::to_string(etime));

        std::string oID = storage.readString();
        debugPrint("Object ID: " + oID);

        int varN = storage.readUnsignedByte();
        debugPrint("N Vars: " + std::to_string(varN));

        std::vector<uint8_t> vars;
        std::string vars_s = "";

        for (int i = 0; i < varN; i++)
        {
            uint8_t vid = storage.readUnsignedByte();
            vars.push_back(vid);
            vars_s = vars_s + std::to_string(vid) + " ";
        }

        debugPrint("Vars: " + vars_s);
        addSubscription(cmdId, oID, btime, etime, vars);
    }
    else
    {
        switch (cmdId)
        {
        case CMD_GETVERSION:

            debugPrint("Got CMD_GETVERSION");
            this->writeVersion();
            break;

        case CMD_SIMSTEP:

            debugPrint("Got CMD_SIMSTEP");
            {
                int ttime = storage.readInt();
                if (ttime != 0 && ttime > Simulation::getInstance()->getCurrentTimeMilliseconds())
                {
                    this->multiple_timestep = true;
                    this->target_time = ttime;
                }
            }
            return true;

        case CMD_SHUTDOWN:

            debugPrint("Got CMD_SHUTDOWN");
            this->cmdShutDown();
            break;

        case CMD_GETSIMVAR:

            debugPrint("Got CMD_GETSIMVAR");
            this->cmdGetSimVar(storage.readUnsignedByte());
            break;

        case CMD_SETVHCSTATE:

            debugPrint("Got CMD_SETVHCSTATE");
            this->cmdSetVhcState(storage);
            break;

        case CMD_GETVHCVAR:

            debugPrint("Got CMD_GETVHCVAR");
            this->cmdGetVhcVar(storage);
            break;

        case CMD_GETRTEVAR:
        case CMD_GETLNKVAR:
        case CMD_GETNDEVAR:

            debugPrint("Got CMD_GETLNKVAR/CMD_GETNDEVAR/CMD_GETRTEVAR");
            this->cmdGetNetworkVar(storage, cmdId);
            break;

        case CMD_GETPOLVAR:
            debugPrint("Got CMD_GETPOLVAR");
            this->cmdGetPolygonVar(storage);
            break;

        case CMD_GETVTPVAR:
            debugPrint("Got CMD_GETVTPVAR");
            this->cmdGetVtpVar(storage);
            break;


        default:

            debugPrint("Command not implemented!");

            writeStatusResponse(cmdId, STATUS_NIMPL, "Method not implemented.");
        }
    }

    return false;
}

/**
 * \brief Writes a status respond to a specific command in the outgoing TraCI message to be sent back to the client.
 * \param cmdId The command to respond to.
 * \param cmdStatus The status response.
 * \param description A std::string describing the result.
 */
void traci_api::TraCIServer::writeStatusResponse(uint8_t cmdId, uint8_t cmdStatus, std::string description)
{
    debugPrint("Writing status response " + std::to_string(cmdStatus) + " for command " + std::to_string(cmdId));
    outgoing.writeUnsignedByte(1 + 1 + 1 + 4 + static_cast<int>(description.length())); // command length
    outgoing.writeUnsignedByte(cmdId); // command type
    outgoing.writeUnsignedByte(cmdStatus); // status
    outgoing.writeString(description); // description
}

/**
 * \brief Writes a server version information message response on the outgoing tcpip::Storage.
 */
void traci_api::TraCIServer::writeVersion()
{
    debugPrint("Writing version information");

    this->writeStatusResponse(CMD_GETVERSION, STATUS_OK, "");

    tcpip::Storage answerTmp;
    answerTmp.writeInt(TRACIAPI_VERSION);
    answerTmp.writeString("Paramics TraCI plugin v" + std::string(PVEINS_VERSION) + " on Paramics v" + std::to_string(qpg_UTL_parentProductVersion()));

    outgoing.writeUnsignedByte(1 + 1 + static_cast<int>(answerTmp.size()));
    outgoing.writeUnsignedByte(CMD_GETVERSION);
    outgoing.writeStorage(answerTmp);
}

/**
 * \brief Sends the internally stored outgoing TraCI message to the client. 
 * Should only be called by the server itself on waitForCommands()
 */
void traci_api::TraCIServer::sendResponse()
{
    debugPrint("Sending response to TraCI client");
    //debugPrint("Outgoing data: " + outgoing.hexDump());

    ssocket.sendExact(outgoing);
}

void traci_api::TraCIServer::writeToOutputWithSize(tcpip::Storage& storage, bool force_extended)
{
    this->writeToStorageWithSize(storage, outgoing, force_extended);
}

void traci_api::TraCIServer::writeToStorageWithSize(tcpip::Storage& src, tcpip::Storage& dest, bool force_extended)
{
    uint32_t size = 1 + src.size();
    if (size > 255 || force_extended)
    {
        // extended-length message
        dest.writeUnsignedByte(0);
        dest.writeInt(size + 4);
    }
    else
        dest.writeUnsignedByte(size);
    dest.writeStorage(src);
}

void traci_api::TraCIServer::addSubscription(uint8_t sub_type, std::string object_id, int start_time, int end_time, std::vector<uint8_t> variables)
{
    std::string errors;
    tcpip::Storage temp;

    // first check if this corresponds to an update for an existing subscription
    for (auto it = subs.begin(); it != subs.end(); ++it)
    {
        uint8_t result = (*it)->updateSubscription(sub_type, object_id, start_time, end_time, variables, temp, errors);

        switch (result)
        {
        case VariableSubscription::STATUS_OK:
            // update ok, return now
            debugPrint("Updated subscription");
            writeStatusResponse(sub_type, STATUS_OK, "");
            writeToOutputWithSize(temp, true);
            return;
        case VariableSubscription::STATUS_UNSUB:
            // unsubscribe command, remove the subscription
            debugPrint("Unsubscribing...");
            delete *it;
            it = subs.erase(it);
            // we don't care about the deleted iterator, since we return from the loop here
            writeStatusResponse(sub_type, STATUS_OK, "");
            return;
        case VariableSubscription::STATUS_ERROR:
            // error when updating
            debugPrint("Error updating subscription.");
            writeStatusResponse(sub_type, STATUS_ERROR, errors);
            break;
        case VariableSubscription::STATUS_NOUPD:
            // no update, try next subscription
            continue;
        default:
            throw std::runtime_error("Received unexpected result " + std::to_string(result) + " when trying to update subscription.");
        }
    }

    // if we reach here, it means we need to add a new subscription.
    // note: it could also mean it's an unsubscribe command for a car that reached its
    // destination. Check number of variables and do nothing if it's 0.

    if (variables.size() == 0)
    {
        // unsub command that didn't match any of the currently running subscriptions, so just
        // tell the client it's ok, everything's alright

        debugPrint("Unsub from subscription already removed.");
        writeStatusResponse(sub_type, STATUS_OK, "");
        return;
    }


    debugPrint("No update. Adding new subscription.");
    VariableSubscription* sub;

    switch (sub_type)
    {
    case CMD_SUB_VHCVAR:

        debugPrint("Adding VHC subscription.");

        sub = new VehicleVariableSubscription(object_id, start_time, end_time, variables);
        break;

    case CMD_SUB_SIMVAR:

        debugPrint("Adding SIM subscription.");

        sub = new SimulationVariableSubscription(object_id, start_time, end_time, variables);
        break;

    default:
        writeStatusResponse(sub_type, STATUS_NIMPL, "Subscription type not implemented: " + std::to_string(sub_type));
        return;
    }

    uint8_t result = sub->handleSubscription(temp, true, errors); // validate

    if (result == VariableSubscription::STATUS_EXPIRED)
    {
        debugPrint("Expired subscription.");

        writeStatusResponse(sub_type, STATUS_ERROR, "Expired subscription.");
        return;
    }
    else if (result != VariableSubscription::STATUS_OK)
    {
        debugPrint("Error adding subscription.");

        writeStatusResponse(sub_type, STATUS_ERROR, errors);
        return;
    }

    writeStatusResponse(sub_type, STATUS_OK, "");
    writeToOutputWithSize(temp, true);
    subs.push_back(sub);
}

void traci_api::TraCIServer::processSubscriptions(tcpip::Storage& sub_store)
{
    debugPrint("Processing subscriptions");
    tcpip::Storage temp;
    tcpip::Storage sub_results;
    uint8_t sub_res;
    std::string errors;
    int count = 0;

    for (auto i = subs.begin(); i != subs.end();)
    {
        sub_res = (*i)->handleSubscription(temp, false, errors);

        if (sub_res == VariableSubscription::STATUS_EXPIRED || sub_res == VariableSubscription::STATUS_OBJNOTFOUND)
        {
            delete *i;
            i = subs.erase(i);
        }
        else// if (sub_res == VariableSubscription::STATUS_OK)
        {
            writeToStorageWithSize(temp, sub_results, true);
            count++;
            ++i; // increment
        }

        temp.reset();
    }

    debugPrint("Done processing subscriptions.");

    sub_store.writeInt(count);
    sub_store.writeStorage(sub_results);
}


/**
 * \brief Executes a shutdown command, destroying the current connections and closing the socket.
 */
void traci_api::TraCIServer::cmdShutDown()
{
    debugPrint("Got shutdown command, acknowledging and shutting down");

    this->writeStatusResponse(CMD_SHUTDOWN, STATUS_OK, "");
    this->sendResponse();
    this->close();
}

/**
 * \brief Gets a variable from the simulation.
 * \param simvar ID of the interal simulation variable to fetch.
 */
void traci_api::TraCIServer::cmdGetSimVar(uint8_t simvar)
{
    tcpip::Storage subs_store;

    if (Simulation::getInstance()->packSimulationVariable(simvar, subs_store))
    {
        this->writeStatusResponse(CMD_GETSIMVAR, STATUS_OK, "");
        this->writeToOutputWithSize(subs_store, false);
    }
    else
    {
        this->writeStatusResponse(CMD_GETSIMVAR, STATUS_NIMPL, "");
    }
}

void traci_api::TraCIServer::cmdGetVhcVar(tcpip::Storage& input)
{
    tcpip::Storage result;
    try
    {
        VehicleManager::getInstance()->packVehicleVariable(input, result);

        this->writeStatusResponse(CMD_GETVHCVAR, STATUS_OK, "");
        this->writeToOutputWithSize(result, false);
    }
    catch (NotImplementedError& e)
    {
        debugPrint("Variable not implemented");
        debugPrint(e.what());


        this->writeStatusResponse(CMD_GETVHCVAR, STATUS_NIMPL, e.what());
    }
    catch (std::exception& e)
    {
        debugPrint("Fatal error???");
        debugPrint(e.what());

        this->writeStatusResponse(CMD_GETVHCVAR, STATUS_ERROR, e.what());
        throw;
    }
}

void traci_api::TraCIServer::cmdGetNetworkVar(tcpip::Storage& input, uint8_t cmdid)
{
    tcpip::Storage result;
    try
    {
        switch (cmdid)
        {
        case CMD_GETLNKVAR:
            debugPrint("Got CMD_GETLNKVAR");
            Network::getInstance()->getLinkVariable(input, result);
            break;
        case CMD_GETNDEVAR:
            debugPrint("Got CMD_GETNDEVAR");
            Network::getInstance()->getJunctionVariable(input, result);
            break;
        case CMD_GETRTEVAR:
            debugPrint("Got CMD_GETRTEVAR");
            Network::getInstance()->getRouteVariable(input, result);
            break;
        default:
            throw std::runtime_error("???");
        }

        this->writeStatusResponse(cmdid, STATUS_OK, "");
        this->writeToOutputWithSize(result, false);
    }
    catch (NotImplementedError& e)
    {
        debugPrint("Variable not implemented");
        debugPrint(e.what());


        this->writeStatusResponse(cmdid, STATUS_NIMPL, e.what());
    }
    catch (std::exception& e)
    {
        debugPrint("Fatal error???");
        debugPrint(e.what());

        this->writeStatusResponse(cmdid, STATUS_ERROR, e.what());
        throw;
    }
}

void traci_api::TraCIServer::cmdSetVhcState(tcpip::Storage& input)
{
    try
    {
        VehicleManager::getInstance()->setVehicleState(input);
        this->writeStatusResponse(CMD_SETVHCSTATE, STATUS_OK, "");
    }
    catch (NotImplementedError& e)
    {
        debugPrint("State change not implemented");
        debugPrint(e.what());


        this->writeStatusResponse(CMD_SETVHCSTATE, STATUS_NIMPL, e.what());
    }
    catch (std::exception& e)
    {
        debugPrint("Fatal error???");
        debugPrint(e.what());

        this->writeStatusResponse(CMD_SETVHCSTATE, STATUS_ERROR, e.what());
        throw;
    }
}

// ReSharper disable once CppMemberFunctionMayBeStatic
// ReSharper disable once CppMemberFunctionMayBeConst
void traci_api::TraCIServer::cmdGetPolygonVar(tcpip::Storage& input)
{
    /* 
     * we currently don't have polygons in Paramics, at least not API-accesible ones,
     * so we'll just report that there are 0 polygons, to maintain compatibility.
     */

    uint8_t var_id = input.readUnsignedByte();
    std::string pol_id = input.readString();
    tcpip::Storage result;

    result.writeUnsignedByte(RES_GETPOLVAR);
    result.writeUnsignedByte(var_id);
    result.writeString(pol_id);

    try
    {
        switch (var_id)
        {
        case VARLST:
            result.writeUnsignedByte(VTYPE_STRLST);
            result.writeStringList(std::vector<std::string>()); // hard coded, no polygons
            break;
        case VARCNT:
            result.writeUnsignedByte(VTYPE_INT);
            result.writeInt(0); // hard coded -- no polygons
            break;
        default:
            throw std::exception();
        }

        writeStatusResponse(CMD_GETPOLVAR, STATUS_OK, "");
        writeToOutputWithSize(result, false);
    }
    catch (...)
    {
        writeStatusResponse(CMD_GETPOLVAR, STATUS_ERROR, "No such polygon (id: " + pol_id + ")");
    }
}

void traci_api::TraCIServer::cmdGetVtpVar(tcpip::Storage& input)
{
    tcpip::Storage result;
    try
    {
        VehicleManager::getInstance()->packVhcTypesVariable(input, result);

        this->writeStatusResponse(CMD_GETVTPVAR, STATUS_OK, "");
        this->writeToOutputWithSize(result, false);
    }
    catch (NotImplementedError& e)
    {
        debugPrint("Variable not implemented");
        debugPrint(e.what());


        this->writeStatusResponse(CMD_GETVTPVAR, STATUS_NIMPL, e.what());
    }
    catch (std::exception& e)
    {
        debugPrint("Fatal error???");
        debugPrint(e.what());

        this->writeStatusResponse(CMD_GETVTPVAR, STATUS_ERROR, e.what());
        throw;
    }
}
