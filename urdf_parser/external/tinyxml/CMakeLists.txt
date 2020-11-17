add_library(tinystr STATIC tinystr.cpp)
add_library(tinyxml STATIC tinyxml.cpp tinyxmlerror.cpp tinyxmlparser.cpp)
target_link_libraries(tinyxml tinystr)
