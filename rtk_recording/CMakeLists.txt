cmake_minimum_required(VERSION 3.10)
project(rtk_recording VERSION 1.0.0)

set(CMAKE_INSTALL_PREFIX /usr)

# # set version
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}-${CPACK_PACKAGE_VERSION}")
add_definitions(-DPKG_VERSION="${CPACK_PACKAGE_VERSION}")

# # Build
add_subdirectory(sample)

find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND REQUIRED
    sglog
    sgtime
)

message("depend inc: ${PC_DEPEND_INCLUDE_DIRS}")
message("depend lib: ${PC_DEPEND_LIBRARIES}")

include_directories(
    include
    ${PC_DEPEND_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/rtk_recording.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PC_DEPEND_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Install
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
    COMPONENT core
    NAMELINK_COMPONENT devel
)

install(DIRECTORY include/
    COMPONENT devel
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jarvis
)

install(DIRECTORY sample
    COMPONENT devel
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jarvis/${PROJECT_NAME}
    PATTERN "build" EXCLUDE
)

configure_file(template.pc.in template.pc)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/template.pc
    COMPONENT devel
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/pkgconfig
)

# # Packages
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VENDOR "SingGo")
set(CPACK_PACKAGE_CONTACT "{Your Name}@sg.cambricon.com")
set(CPACK_GENERATOR "DEB")

set(CPACK_DEBIAN_FILE_NAME "DEB-DEFAULT")
set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS ON)
set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS_POLICY ">=")
set(CPACK_DEBIAN_PACKAGE_DEPENDS
    "jarvis-framework-sdk, libeigen3-dev")
set(CPACK_DEB_COMPONENT_INSTALL ON)

# core pkg
set(CPACK_DEBIAN_CORE_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_CORE_DEBUGINFO_PACKAGE ON)
set(CPACK_DEBIAN_CORE_PACKAGE_NAME ${CPACK_PACKAGE_NAME})

# devel pkg
set(CPACK_DEBIAN_DEVEL_PACKAGE_DEPENDS "${CPACK_PACKAGE_NAME} (= ${CPACK_PACKAGE_VERSION})")

set(CPACK_SET_DESTDIR true)
set(CPACK_INSTALL_PREFIX /opt/ros/$ENV{ROS_DISTRO})

include(CPack)
