add_library(grblhal-pneumaseal INTERFACE)
target_sources(grblhal-pneumaseal INTERFACE ${CMAKE_CURRENT_LIST_DIR}/pneumaseal.c)
target_include_directories(grblhal-pneumaseal INTERFACE ${CMAKE_CURRENT_LIST_DIR})
