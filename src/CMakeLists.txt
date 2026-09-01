# Hybrid (arduino+espidf) envs only -- see the root CMakeLists.txt. Mirrors
# the [env] build_src_filter (+<*> -<utilities/>), which the espidf builder
# cannot honour ("'src_filter' option cannot be used with ESP-IDF"): the
# utilities/ tree holds standalone nRF flash tools with their own setup()/
# loop() and must never be linked into a device image.
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.c ${CMAKE_SOURCE_DIR}/src/*.cpp)
list(FILTER app_sources EXCLUDE REGEX "/utilities/")

idf_component_register(SRCS ${app_sources})
target_compile_options(${COMPONENT_LIB} PRIVATE -Wformat)
