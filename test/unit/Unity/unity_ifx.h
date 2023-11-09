
#define TEST_IFX(group, name) static void TEST_##group##_##name##_(void); static void TEST_##group##_##name##_run(void); void TEST_##group##_##name##_run(void) { UnityTestRunner(TEST_##group##_SETUP, TEST_##group##_##name##_, TEST_##group##_TEAR_DOWN, "TEST(" #group ", " #name ")", TEST_GROUP_##group, #name, __FILE__, __LINE__); } void TEST_##group##_##name##_(void)
