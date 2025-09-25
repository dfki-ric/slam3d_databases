# Find all dependencies
find_package(PkgConfig REQUIRED)

find_package(slam3d REQUIRED)

find_package(hiredis)
find_package(cpprestsdk)
pkg_check_modules(neo4j REQUIRED IMPORTED_TARGET neo4j-client)
pkg_check_modules(neo4j-client REQUIRED IMPORTED_TARGET GLOBAL)

# if (NOT jsoncpp_FOUND)
#   pkg_check_modules(jsoncpp IMPORTED_TARGET jsoncpp)
#   if (jsoncpp_FOUND)
#     add_library(jsoncpp_lib ALIAS PkgConfig::jsoncpp)
#   endif()
# endif()



