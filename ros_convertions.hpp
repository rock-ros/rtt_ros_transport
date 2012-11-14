#ifndef RTT_TRANSPORT_ROS_CONVERTIONS_HPP
#define RTT_TRANSPORT_ROS_CONVERTIONS_HPP

#include <stdexcept>

namespace ros_integration
{
    struct InvalidROSConvertion : public std::runtime_error
    {
        InvalidROSConvertion(std::string const& msg)
            : std::runtime_error(msg) {}
    };
}

#endif

