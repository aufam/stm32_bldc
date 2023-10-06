file(READ version.txt BLDC_VERSION)
message("BLDC_VERSION : ${BLDC_VERSION}")

# sources
file(GLOB_RECURSE BLDC_SOURCES bldc/*.*)
add_library(bldc ${BLDC_SOURCES})

# include dirs
target_include_directories(bldc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# defines
target_compile_definitions(bldc PUBLIC -DBLDC_VERSION="${BLDC_VERSION}")

# depends
target_link_libraries(bldc etl)
target_link_libraries(bldc periph)