find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(calibmar_cli main.cpp)
target_link_libraries(calibmar_cli PRIVATE calibmar Boost::program_options)

install(TARGETS calibmar_cli)
