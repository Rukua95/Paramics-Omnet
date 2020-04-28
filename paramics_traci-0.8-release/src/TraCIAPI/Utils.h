#pragma once
#include "storage.h"
#include <mutex>

#include "programmer.h"
#include "Constants.h"

class Vector2D
{
public:
    double x;
    double y;

    Vector2D(double x, double y) : x(x), y(y)
    {
    }
};

class Vector3D : public Vector2D
{
public:
    double z;

    Vector3D(double x, double y, double z) : Vector2D(x, y), z(z)
    {
    }
};

class PositionalData : public Vector3D
{
public:
    double gradient;
    double bearing;

    PositionalData(double x, double y, double z, double b, double g) : Vector3D(x, y, z), gradient(g), bearing(b)
    {
    }
};

class DimensionalData
{
public:
    double height;
    double length;
    double width;

    DimensionalData(double h, double l, double w) : height(h), length(l), width(w)
    {
    }
};

uint32_t RGB2HEX(uint8_t r, uint8_t g, uint8_t b);
void HEX2RGB(uint32_t hex, uint8_t& r, uint8_t& g, uint8_t& b);

namespace traci_api
{
    void debugPrint(std::string text);
    void infoPrint(std::string text);

    // the following convenience functions were obtained from SUMO.
    // these functions simplify the obtention of variables from storage:

    /// @name Helpers for reading and checking values
    /// @{

    /** @brief Reads the value type and an int, verifying the type
    *
    * @param[in, changed] inputStorage The storage to read from
    * @param[out] into Holder of the read value
    * @return Whether an integer value was given (by data type)
    */
    bool readTypeCheckingInt(tcpip::Storage& inputStorage, int& into);


    /** @brief Reads the value type and a double, verifying the type
    *
    * @param[in, changed] inputStorage The storage to read from
    * @param[out] into Holder of the read value
    * @return Whether a double value was given (by data type)
    */
    bool readTypeCheckingDouble(tcpip::Storage& inputStorage, double& into);


    /** @brief Reads the value type and a string, verifying the type
    *
    * @param[in, changed] inputStorage The storage to read from
    * @param[out] into Holder of the read value
    * @return Whether a string value was given (by data type)
    */
    bool readTypeCheckingString(tcpip::Storage& inputStorage, std::string& into);


    /** @brief Reads the value type and a string list, verifying the type
    *
    * @param[in, changed] inputStorage The storage to read from
    * @param[out] into Holder of the read value
    * @return Whether a double value was given (by data type)
    */
    bool readTypeCheckingStringList(tcpip::Storage& inputStorage, std::vector<std::string>& into);


    /** @brief Reads the value type and a color, verifying the type
    *
    * @param[in, changed] inputStorage The storage to read from
    * @param[out] into Holder of the read value
    * @return Whether a color was given (by data type)
    */
    bool readTypeCheckingColor(tcpip::Storage& inputStorage, uint32_t& hex);


    /** @brief Reads the value type and a 2D position, verifying the type
    *
    * @param[in, changed] inputStorage The storage to read from
    * @param[out] into Holder of the read value
    * @return Whether a 2D position was given (by data type)
    */
    bool readTypeCheckingPosition2D(tcpip::Storage& inputStorage, Vector2D& into);


    /** @brief Reads the value type and a 2D bounding box, verifying the type
    *
    * @param[in, changed] inputStorage The storage to read from
    * @param[out] into Holder of the read value
    * @return Whether a 2D bounding box was given (by data type)
    */
    //bool  readTypeCheckingBoundary(tcpip::Storage& inputStorage, Boundary& into);


    /** @brief Reads the value type and a byte, verifying the type
    *
    * @param[in, changed] inputStorage The storage to read from
    * @param[out] into Holder of the read value
    * @return Whether a byte was given (by data type)
    */
    bool readTypeCheckingByte(tcpip::Storage& inputStorage, int8_t& into);


    /** @brief Reads the value type and an unsigned byte, verifying the type
    *
    * @param[in, changed] inputStorage The storage to read from
    * @param[out] into Holder of the read value
    * @return Whether an unsigned byte was given (by data type)
    */
    bool readTypeCheckingUnsignedByte(tcpip::Storage& inputStorage, uint8_t& into);


    /** @brief Reads the value type and a polygon, verifying the type
    *
    * @param[in, changed] inputStorage The storage to read from
    * @param[out] into Holder of the read value
    * @return Whether an unsigned byte was given (by data type)
    */
    //bool  readTypeCheckingPolygon(tcpip::Storage& inputStorage, PositionVector& into);
    /// @}
}
