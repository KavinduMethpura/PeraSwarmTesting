#include <unity.h>
#include <thread>
#include <chrono>
#include "../../src/modules/motors/motors.h"

// Variables for expected values
static int32_t encLeftStart, encRightStart;
extern SW_Motors motors;

void setUp(void) {
    // Reset encoders before each test
    motors.encoder_reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void tearDown(void) {
    // Stop motors after each test
    motors.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void test_encoder_initial_zero(void) {
    int32_t left, right;
    motors.encoder_read(&left, &right);
    TEST_ASSERT_EQUAL_INT32(0, left);
    TEST_ASSERT_EQUAL_INT32(0, right);
}

void test_encoder_counts_increase_forward(void) {
    encLeftStart = encRightStart = 0;

    // Drive forward slowly
    motors.write(100, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    int32_t left, right;
    motors.encoder_read(&left, &right);

    TEST_ASSERT_TRUE(left > encLeftStart);
    TEST_ASSERT_TRUE(right > encRightStart);
}

void test_encoder_counts_decrease_backward(void) {
    encLeftStart = encRightStart = 0;

    // Drive backward slowly
    motors.write(-100, -100);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    int32_t left, right;
    motors.encoder_read(&left, &right);

    TEST_ASSERT_TRUE(left < encLeftStart);
    TEST_ASSERT_TRUE(right < encRightStart);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    motors.begin();

    RUN_TEST(test_encoder_initial_zero);
    RUN_TEST(test_encoder_counts_increase_forward);
    RUN_TEST(test_encoder_counts_decrease_backward);

    UNITY_END();
    return 0;
}
