# Install script for directory: /home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/crh/文档/Machine_Learning_25D/ml25d_ws/install/ml25d_dataset_generation")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation/environment" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation/environment" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ml25d_dataset_generation-0.2.0-py3.12.egg-info" TYPE DIRECTORY FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_python/ml25d_dataset_generation/ml25d_dataset_generation.egg-info/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ml25d_dataset_generation" TYPE DIRECTORY FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/python/ml25d_dataset_generation/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/crh/文档/Machine_Learning_25D/ml25d_ws/install/ml25d_dataset_generation/lib/python3.12/site-packages/ml25d_dataset_generation"
      )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ml25d_dataset_generation/sim_bridge_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ml25d_dataset_generation/sim_bridge_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ml25d_dataset_generation/sim_bridge_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ml25d_dataset_generation" TYPE EXECUTABLE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/sim_bridge_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ml25d_dataset_generation/sim_bridge_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ml25d_dataset_generation/sim_bridge_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ml25d_dataset_generation/sim_bridge_node"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ml25d_dataset_generation/sim_bridge_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/CMakeFiles/sim_bridge_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ml25d_dataset_generation" TYPE PROGRAM FILES
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/scripts/generate_dataset.py"
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/scripts/train_risk_model.py"
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/scripts/run_single_sample.py"
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/scripts/smoke_ros_gz.py"
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/scripts/analyze_failure_modes.py"
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/scripts/validate_samples.py"
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/scripts/stats_report.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation" TYPE DIRECTORY FILES
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/config"
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/launch"
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/docs"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/README.md")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ml25d_dataset_generation")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ml25d_dataset_generation")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation/environment" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation/environment" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_index/share/ament_index/resource_index/packages/ml25d_dataset_generation")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation/cmake" TYPE FILE FILES
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_core/ml25d_dataset_generationConfig.cmake"
    "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/ament_cmake_core/ml25d_dataset_generationConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ml25d_dataset_generation" TYPE FILE FILES "/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/crh/文档/Machine_Learning_25D/ml25d_ws/build/ml25d_dataset_generation/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
