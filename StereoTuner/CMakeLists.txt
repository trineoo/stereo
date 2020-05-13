cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(EXTRA_C_FLAGS "-lgtk-x11-2.0 -lgdk-x11-2.0 -lpangocairo-1.0 -latk-1.0 -lcairo -lgdk_pixbuf-2.0 -lgio-2.0 -lpangoft2-1.0 -lpango-1.0 -lgobject-2.0 -lfontconfig -lfreetype -Wl,--export-dynamic -lgmodule-2.0 -pthread -lglib-2.0")
project( main )
find_package( OpenCV REQUIRED )
find_package(PkgConfig REQUIRED)
find_package(Threads)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})

add_definitions(${GTK3_CFLAGS_OTHER})
add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} ${GTK3_LIBRARIES} pthread)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} gtk+-3.0 gmodule-2.0")
#all: --cflags gtk+-3.0 gmodule-2.0 opencv` main.cpp -o main `pkg-config --libs gtk+-3.0 gmodule-export-2.0 opencv`
