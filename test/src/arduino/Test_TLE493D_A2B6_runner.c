// Unity c includes
#include "unity.h"
#include "unity_fixture.h"


TEST_GROUP_RUNNER(TLE493D_A2B6)
{
  RUN_TEST_CASE(TLE493D_A2B6, calculateTemperature);
  RUN_TEST_CASE(TLE493D_A2B6, dummy);
}


TEST_GROUP_RUNNER(TLE493D_A2B6_needsSensor)
{
  RUN_TEST_CASE(TLE493D_A2B6_needsSensor, defaultConfig);
}
