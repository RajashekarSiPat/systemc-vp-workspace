/*
 * main.cc — Virtual-platform entry point for the Cortex-A53 virt machine.
 *
 * This is pure boilerplate: the entire platform topology (CPU, memory, USART,
 * loader, …) is defined in conf.lua.  Add CCI parameters or custom wiring
 * here only when Lua is not sufficient.
 *
 * Usage:
 *   ./cortex-a53-virt-vp -c platforms/cortex-a53-virt/conf.lua
 */

#include <systemc>

#include <chrono>
#include <string>

#include <cci_configuration>

#include <cciutils.h>
#include <argparser.h>
#include <module_factory_container.h>

#if SC_VERSION_MAJOR < 3
#warning PLEASE UPDATE TO SYSTEMC 3.0, OLDER VERSIONS ARE DEPRECATED
#endif

class CortexA53VirtPlatform : public gs::ModuleFactory::Container
{
protected:
    cci::cci_param<int> m_quantum_ns;
    cci::cci_param<int> m_gdb_port;

public:
    CortexA53VirtPlatform(const sc_core::sc_module_name& n)
        : gs::ModuleFactory::Container(n)
        , m_quantum_ns("quantum_ns", 1000000, "TLM-2.0 global quantum in ns")
        , m_gdb_port("gdb_port", 0, "GDB server port (0 = disabled)")
    {
        using tlm_utils::tlm_quantumkeeper;
        sc_core::sc_time quantum(m_quantum_ns, sc_core::SC_NS);
        tlm_quantumkeeper::set_global_quantum(quantum);
    }
};

int sc_main(int argc, char* argv[])
{
    if (sc_core::sc_version_major < 3) {
        SCP_WARN()("\n*** WARNING: using deprecated SystemC version, please upgrade ***");
    }

    scp::LoggingGuard logging_guard(
        scp::LogConfig()
            .fileInfoFrom(sc_core::SC_ERROR)
            .logAsync(false)
            .logLevel(scp::log::DBGTRACE)
            .msgTypeFieldWidth(30));

    gs::ConfigurableBroker m_broker{};
    cci::cci_originator orig("sc_main");
    cci::cci_param<int> p_log_level{
        "log_level", 0, "Default log level", cci::CCI_ABSOLUTE_NAME, orig
    };
    auto broker_h = m_broker.create_broker_handle(orig);
    ArgParser ap{ broker_h, argc, argv };

    CortexA53VirtPlatform platform("platform");

    auto wall_start = std::chrono::system_clock::now();
    try {
        SCP_INFO() << "SC_START";
        sc_core::sc_start();
    } catch (std::runtime_error const& e) {
        std::cerr << argv[0] << " Error: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << argv[0] << " Error: " << e.what() << std::endl;
        return 2;
    } catch (...) {
        SCP_ERR() << "Unknown error in sc_main";
        return 3;
    }

    auto wall_end = std::chrono::system_clock::now();
    auto elapsed  = std::chrono::duration_cast<std::chrono::seconds>(wall_end - wall_start);

    std::cout << "Simulation Time:     " << sc_core::sc_time_stamp().to_seconds() << " SC_SEC\n";
    std::cout << "Wall-clock Duration: " << elapsed.count() << " s\n";

    return 0;
}
