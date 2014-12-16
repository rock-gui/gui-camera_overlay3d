#include <boost/test/unit_test.hpp>
#include <camera_overlay3d/Dummy.hpp>

using namespace camera_overlay3d;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    camera_overlay3d::DummyClass dummy;
    dummy.welcome();
}
