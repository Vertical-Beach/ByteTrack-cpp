#include <ByteTrack/BYTETracker.h>

#include <gtest/gtest.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

TEST (ByteTrack, BYTETracker)
{
    byte_track::BYTETracker tracker(10, 30);
    // TODO: impl test of tracker.update()
}

int main (int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return (RUN_ALL_TESTS());
}
