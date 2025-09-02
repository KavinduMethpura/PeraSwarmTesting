#include <unity.h>
#include "../../src/modules/motors/motors.h"

// Create a global SW_Motors instance for testing
SW_Motors motors;

void setUp(void) {
    // This function is run before EACH test
}

void tearDown(void) {
    // This function is run after EACH test
}

void test_motors_default_values(void) {
    // Example: check if motor speed defaults to 0
    for (int i = 0; i < SW_MOTORS_COUNT; i++) {
        TEST_ASSERT_EQUAL_INT(0, motors.getSpeed(i));
    }
}

void test_motors_set_and_get_speed(void) {
    motors.setSpeed(0, 100);
    TEST_ASSERT_EQUAL_INT(100, motors.getSpeed(0));

    motors.setSpeed(1, -50);
    TEST_ASSERT_EQUAL_INT(-50, motors.getSpeed(1));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_motors_default_values);
    RUN_TEST(test_motors_set_and_get_speed);

    UNITY_END();
    return 0;
}
