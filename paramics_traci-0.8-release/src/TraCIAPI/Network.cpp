#include "Network.h"


traci_api::Network* traci_api::Network::instance = nullptr;

traci_api::Network::Network()
{
    int routes = qpg_NET_busroutes();
    for (int i = 1; i <= routes; i++)
    {
        BUSROUTE* route = qpg_NET_busrouteByIndex(i);
        std::string name = qpg_BSR_name(route);

        route_name_map[name] = route;

        int link_n = qpg_BSR_links(route);
        std::vector<std::string> link_names;

        LINK* current_link = qpg_BSR_firstLink(route);
        link_names.push_back(qpg_LNK_name(current_link));

        for (int link_i = 0; link_i < link_n - 1; link_i++)
        {
            current_link = qpg_BSR_nextLink(route, current_link);
            link_names.push_back(qpg_LNK_name(current_link));
        }

        route_links_map[route] = link_names;
    }
}

traci_api::Network* traci_api::Network::getInstance()
{
    if (!instance)
        instance = new Network();
    return instance;
    // TODO: Delete instances
}

// ReSharper disable once CppMemberFunctionMayBeStatic
void traci_api::Network::getLinkVariable(tcpip::Storage& input, tcpip::Storage& output) const throw(traci_api::NoSuchObjectError)
{
    uint8_t varID = input.readUnsignedByte();
    std::string lnkID = input.readString();
    LINK* lnk;

    if (varID != VARLST && varID != VARCNT)
    {
        lnk = qpg_NET_link(&lnkID[0]);
        if (!lnk)
            throw traci_api::NoSuchObjectError("No such edge: " + lnkID);
    }

    output.writeUnsignedByte(RES_GETLNKVAR);
    output.writeUnsignedByte(varID);
    output.writeString(lnkID);

    switch (varID)
    {
    case VARLST:
        output.writeUnsignedByte(VTYPE_STRLST);
        {
            std::vector<std::string> edges;
            int lnks = qpg_NET_links();
            for (int i = 1; i <= lnks; i++)
                edges.push_back(qpg_LNK_name(qpg_NET_linkByIndex(i)));
            output.writeStringList(edges);
        }
        break;
    case VARCNT:
        output.writeUnsignedByte(VTYPE_INT);
        output.writeInt(qpg_NET_links());
        break;
    default:
        throw NotImplementedError("");
    }
}

// ReSharper disable once CppMemberFunctionMayBeStatic
void traci_api::Network::getJunctionVariable(tcpip::Storage& input, tcpip::Storage& output) const throw(traci_api::NoSuchObjectError)
{
    uint8_t varID = input.readUnsignedByte();
    std::string ndeID = input.readString();
    NODE* node;

    if (varID != VARLST && varID != VARCNT)
    {
        node = qpg_NET_node(&ndeID[0]);
        if (!node)
            throw traci_api::NoSuchObjectError("No such node: " + ndeID);
    }

    output.writeUnsignedByte(RES_GETNDEVAR);
    output.writeUnsignedByte(varID);
    output.writeString(ndeID);

    switch (varID)
    {
    case VARLST:
        output.writeUnsignedByte(VTYPE_STRLST);
        {
            std::vector<std::string> nodes;
            int node_count = qpg_NET_nodes();
            for (int i = 1; i <= node_count; i++)
                nodes.push_back(qpg_NDE_name(qpg_NET_nodeByIndex(i)));
            output.writeStringList(nodes);
        }
        break;

    case VARCNT:
        output.writeUnsignedByte(VTYPE_INT);
        output.writeInt(qpg_NET_nodes());
        break;

    case VAR_NDE_POS:
        output.writeUnsignedByte(VTYPE_POSITION);
        {
            float x, y, z;
            // ReSharper disable once CppLocalVariableMightNotBeInitialized
            qpg_POS_node(node, &x, &y, &z);
            output.writeDouble(x);
            output.writeDouble(y);
        }
        break;

    case VAR_NDE_SHP:
    default:
        throw NotImplementedError("");
    }
}

void traci_api::Network::getRouteVariable(tcpip::Storage& input, tcpip::Storage& output) const throw(traci_api::NoSuchObjectError)
{
    uint8_t varID = input.readUnsignedByte();
    std::string routeID = input.readString();
    BUSROUTE* route;

    if (varID != VARLST && varID != VARCNT)
    {
        try
        {
            route = route_name_map.at(routeID);
        }
        // ReSharper disable once CppEntityNeverUsed
        catch (std::out_of_range& e)
        {
            throw traci_api::NoSuchObjectError("No such node: " + routeID);
        }
    }

    output.writeUnsignedByte(RES_GETRTEVAR);
    output.writeUnsignedByte(varID);
    output.writeString(routeID);

    switch (varID)
    {
    case VARLST:
        output.writeUnsignedByte(VTYPE_STRLST);
        {
            std::vector<std::string> routes;
            for (auto kv : route_name_map)
                routes.push_back(kv.first);
            output.writeStringList(routes);
        }
        break;

    case VARCNT:
        output.writeUnsignedByte(VTYPE_INT);
        output.writeInt(qpg_NET_busroutes());
        break;

    case VAR_RTE_EDGES:
        output.writeUnsignedByte(VTYPE_STRLST);
        output.writeStringList(route_links_map.at(route));
        break;

    default:
        throw NotImplementedError("");
    }
}
