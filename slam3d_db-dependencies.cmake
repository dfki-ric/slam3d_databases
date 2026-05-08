# Find all dependencies
find_package(PkgConfig REQUIRED)
find_package(slam3d 3.0 REQUIRED)

find_package(hiredis)
if(TARGET hiredis::hiredis)
  set(HIREDIS_TARGET hiredis::hiredis)
else()
  pkg_check_modules(hiredis IMPORTED_TARGET hiredis)
  if(TARGET PkgConfig::hiredis)
    set(HIREDIS_TARGET PkgConfig::hiredis)
  endif()
endif()

find_package(cpprestsdk)
pkg_check_modules(neo4j REQUIRED IMPORTED_TARGET neo4j-client)
pkg_check_modules(neo4j-client REQUIRED IMPORTED_TARGET GLOBAL)

