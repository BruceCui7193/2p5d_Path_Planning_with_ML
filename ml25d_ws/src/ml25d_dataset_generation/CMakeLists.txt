cmake_minimum_required(VERSION 3.8)
project(ml25d_dataset_generation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
)

add_executable(sim_bridge_node src/cpp/sim_bridge_node.cpp)
ament_target_dependencies(sim_bridge_node rclcpp std_msgs)

install(TARGETS
  sim_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_dataset.py
  scripts/train_risk_model.py
  scripts/calibrate_risk_model.py
  scripts/run_single_sample.py
  scripts/smoke_ros_gz.py
  scripts/analyze_failure_modes.py
  scripts/validate_samples.py
  scripts/stats_report.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  docs
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_terrain
    test/test_terrain_generator.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python
  )

  ament_add_pytest_test(
    test_labels
    test/test_label_extractor.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python
  )

  ament_add_pytest_test(
    test_training_pipeline
    test/test_training_pipeline.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python
  )

  ament_add_pytest_test(
    test_gazebo_runner
    test/test_gazebo_runner.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python
  )
endif()

ament_package()
