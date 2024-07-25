# CMake generated Testfile for 
# Source directory: /home/oswaldor/trochoids_ws/src/trochoids
# Build directory: /home/oswaldor/trochoids_ws/build/trochoids
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_trochoids_gtest_trochoids-test "/home/oswaldor/trochoids_ws/build/trochoids/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/oswaldor/trochoids_ws/build/trochoids/test_results/trochoids/gtest-trochoids-test.xml" "--return-code" "/home/oswaldor/trochoids_ws/devel/.private/trochoids/lib/trochoids/trochoids-test --gtest_output=xml:/home/oswaldor/trochoids_ws/build/trochoids/test_results/trochoids/gtest-trochoids-test.xml")
set_tests_properties(_ctest_trochoids_gtest_trochoids-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/oswaldor/trochoids_ws/src/trochoids/CMakeLists.txt;88;catkin_add_gtest;/home/oswaldor/trochoids_ws/src/trochoids/CMakeLists.txt;0;")
subdirs("gtest")
