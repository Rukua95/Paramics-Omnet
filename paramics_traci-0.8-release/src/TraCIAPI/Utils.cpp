#include "Utils.h"


/**
* \brief Convenience function, prints an std::string on Paramics' output window.
* \param text
*/
void traci_api::debugPrint(std::string text)
{
    if (DEBUG)
    {
        text = "TraCI DEBUG: " + text;
        qps_GUI_printf(&text[0u]);
        std::cerr << text << std::endl;
    }
}

void traci_api::infoPrint(std::string text)
{
    text = "TraCI INFO: " + text;
    qps_GUI_printf(&text[0u]);
    std::cerr << text << std::endl;
}

bool traci_api::readTypeCheckingInt(tcpip::Storage& inputStorage, int& into)
{
    if (inputStorage.readUnsignedByte() != VTYPE_INT)
    {
        return false;
    }
    into = inputStorage.readInt();
    return true;
}

bool traci_api::readTypeCheckingDouble(tcpip::Storage& inputStorage, double& into)
{
    if (inputStorage.readUnsignedByte() != VTYPE_DOUBLE)
    {
        return false;
    }
    into = inputStorage.readDouble();
    return true;
}

bool traci_api::readTypeCheckingString(tcpip::Storage& inputStorage, std::string& into)
{
    if (inputStorage.readUnsignedByte() != VTYPE_STR)
    {
        return false;
    }
    into = inputStorage.readString();
    return true;
}

bool traci_api::readTypeCheckingStringList(tcpip::Storage& inputStorage, std::vector<std::string>& into)
{
    if (inputStorage.readUnsignedByte() != VTYPE_STRLST)
    {
        return false;
    }
    into = inputStorage.readStringList();
    return true;
}

bool traci_api::readTypeCheckingColor(tcpip::Storage& inputStorage, uint32_t& hex)
{
    if (inputStorage.readUnsignedByte() != VTYPE_COLOR)
        return false;

    uint8_t r, g, b; /* paramics doesn't use transparency :( */
    r = inputStorage.readUnsignedByte();
    g = inputStorage.readUnsignedByte();
    b = inputStorage.readUnsignedByte();
    inputStorage.readUnsignedByte(); /* discard the alpha channel */

    hex = RGB2HEX(r, g, b);

    return true;
}

bool traci_api::readTypeCheckingPosition2D(tcpip::Storage& inputStorage, Vector2D& into)
{
    if (inputStorage.readUnsignedByte() != VTYPE_POSITION)
    {
        return false;
    }
    into.x = inputStorage.readDouble();
    into.y = inputStorage.readDouble();
    return true;
}

bool traci_api::readTypeCheckingByte(tcpip::Storage& inputStorage, int8_t& into)
{
    if (inputStorage.readUnsignedByte() != VTYPE_BYTE)
    {
        return false;
    }
    into = inputStorage.readByte();
    return true;
}

bool traci_api::readTypeCheckingUnsignedByte(tcpip::Storage& inputStorage, uint8_t& into)
{
    if (inputStorage.readUnsignedByte() != VTYPE_UBYTE)
    {
        return false;
    }
    into = inputStorage.readUnsignedByte();
    return true;
}

uint32_t RGB2HEX(uint8_t r, uint8_t g, uint8_t b)
{
    /*
     * Paramics is weird and idiotic, and put the RGB in the reverse order.
     */
    return ((b & 0xff) << 16) + ((g & 0xff) << 8) + (r & 0xff);
}

void HEX2RGB(uint32_t hex, uint8_t& r, uint8_t& g, uint8_t& b)
{
    r = hex & 0xff;
    g = (hex >> 8) & 0xff;
    b = (hex >> 16) & 0xff;
}
