# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(leo_vcu_driver)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)



ament_auto_find_build_dependencies(REQUIRED
        ${${PROJECT_NAME}_BUILD_DEPENDS}
        ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
        )

set(LEO_VCU_DRIVER_LIB_SRC
        src/leo_vcu_driver.cpp
        src/AsyncSerial.cpp
        )

set(LEO_VCU_DRIVER_LIB_HEADERS
        include/leo_vcu_driver/leo_vcu_driver.hpp
        include/leo_vcu_driver/AsyncSerial.h
        include/leo_vcu_driver/checksum.h
        include/leo_vcu_driver/vehicle_interface.h
        include/leo_vcu_driver/visibility_control.hpp
        )

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
        ${LEO_VCU_DRIVER_LIB_SRC}
        ${LEO_VCU_DRIVER_LIB_HEADERS}
        )
autoware_set_compile_options(${PROJECT_NAME})

set(LEO_VCU_DRIVER_NODE_SRC
        src/leo_vcu_driver_node.cpp
        )



# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
        ${LEO_VCU_DRIVER_NODE_SRC}
        ${LEO_VCU_DRIVER_NODE_HEADERS}
        )
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
        PLUGIN "autoware::leo_vcu_driver::LeoVcuDriverNode"
        EXECUTABLE ${PROJECT_NAME}_node_exe
        )

# Testing
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # Unit tests
    set(TEST_SOURCES test/test_leo_vcu_driver.cpp)
    set(TEST_LEO_VCU_DRIVER_EXE test_leo_vcu_driver)
    ament_add_gtest(${TEST_LEO_VCU_DRIVER_EXE} ${TEST_SOURCES})
    autoware_set_compile_options(${TEST_LEO_VCU_DRIVER_EXE})
    target_link_libraries(${TEST_LEO_VCU_DRIVER_EXE} ${PROJECT_NAME})
endif ()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
        launch
        config
        )
