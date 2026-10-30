#define NANOSVG_IMPLEMENTATION
#include "nanosvg/nanosvg.h"
#define NANOSVGRAST_IMPLEMENTATION
#include "nanosvg/nanosvgrast.h"
#include <catch2/catch_all.hpp>
#include <catch2/reporters/catch_reporter_event_listener.hpp>
#include <catch2/reporters/catch_reporter_registrars.hpp>

#include "libslic3r/GCode/GCodeProcessor.hpp"

namespace {

// Slicing sets the process-wide GCodeProcessor::s_IsBBLPrinter and never restores it, and the flag
// selects which spelling of the reserved comment tags the processor matches. Reset it to its default
// before every test case so a test does not depend on which tests ran before it in random order.
class ResetGCodeProcessorFlavor final : public Catch::EventListenerBase
{
public:
    using Catch::EventListenerBase::EventListenerBase;

    void testCaseStarting(const Catch::TestCaseInfo&) override { Slic3r::GCodeProcessor::s_IsBBLPrinter = true; }
};

} // namespace

CATCH_REGISTER_LISTENER(ResetGCodeProcessorFlavor)
