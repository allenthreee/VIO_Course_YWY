add_executable(testCurveFitting CurveFitting.cpp)
add_executable(new_testCurveFitting newCurveFitting.cpp)

target_link_libraries(testCurveFitting ${PROJECT_NAME}_backend)
target_link_libraries(new_testCurveFitting ${PROJECT_NAME}_backend)

