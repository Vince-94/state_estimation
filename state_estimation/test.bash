#!/bin/bash

./build/kalman_filter/test_linear_kalman_filter --gtest_color=yes --gtest_print_time=1
./build/kalman_filter/test_extended_kalman_filter --gtest_color=yes --gtest_print_time=1
./build/kalman_filter/test_unscented_kalman_filter --gtest_color=yes --gtest_print_time=1
