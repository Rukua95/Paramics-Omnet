#pragma once
#include <stdexcept>

namespace traci_api
{
    class NotImplementedError : public std::runtime_error
    {
    public:
        explicit NotImplementedError(const std::string& _Message)
            : runtime_error(_Message)
        {
        }

        explicit NotImplementedError(const char* _Message)
            : runtime_error(_Message)
        {
        }
    };

    class NoSuchObjectError : public std::runtime_error
    {
    public:
        explicit NoSuchObjectError(const std::string& _Message)
            : runtime_error(_Message)
        {
        }

        explicit NoSuchObjectError(const char* _Message)
            : runtime_error(_Message)
        {
        }
    };
}
