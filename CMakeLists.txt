cmake_minimum_required(VERSION 3.7)

project(SLAM3D_DB VERSION "1.0.0")
enable_testing()

include(slam3d_db-dependencies.cmake)
add_subdirectory(slam3d)

# Export the library interface
install(EXPORT slam3d_db-targets
	NAMESPACE slam3d_db::
	DESTINATION lib/cmake/slam3d_db
)

# Create and install the version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file("slam3d_db-config-version.cmake"
	VERSION ${SLAM3D_DB_VERSION}
	COMPATIBILITY SameMajorVersion
)

install(
	FILES
	slam3d_db-config.cmake
	slam3d_db-dependencies.cmake
		${PROJECT_BINARY_DIR}/slam3d_db-config-version.cmake
	DESTINATION
		lib/cmake/slam3d_db
)

# Install catkin package-manifest
install(
	FILES package.xml
	DESTINATION share/slam3d_db
)
