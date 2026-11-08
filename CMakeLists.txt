# CMakeLists.txt for DMMotor

# Add module to include path
target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Auto-include optional source files (the current implementation is header-only)
file(GLOB MODULE_DMMOTOR_SRC CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/*.cxx"
  "${CMAKE_CURRENT_LIST_DIR}/*.cc"
  "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

target_sources(xr PRIVATE ${MODULE_DMMOTOR_SRC})
