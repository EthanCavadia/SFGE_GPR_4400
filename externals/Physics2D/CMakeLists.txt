project(Physics2D)


include_directories(include/)

file(GLOB_RECURSE Physics2D_SRC 
    include/*.h src/*.cpp)

add_library(Physics2D STATIC ${Physics2D_SRC})
set_property(TARGET Physics2D PROPERTY CXX_STANDARD 11)
