add_subdirectory(libs)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configuration/root_directory.h.in
	${CMAKE_CURRENT_SOURCE_DIR}/include/root_directory.h)

add_library(animation STATIC
	./src/filesystem.cpp
	./src/euler_rotations_anim.cpp
	./src/display.cpp
	./src/stb_image.cpp
	./src/shader.cpp
	./src/texture.cpp
	./src/camera.cpp
	./src/callback_handler.cpp
	./src/callback_setter.cpp
	./src/transform.cpp
	./src/obj_loader.cpp
	./src/mesh.cpp
	./src/indexed_model.cpp
	./src/imported_surface.cpp
	./src/indexed_surface.cpp
	./include/primitives/box.cpp
	./include/primitives/line.cpp
	# ./include/primitives/plane.cpp
	./include/primitives/point.cpp
	# ./include/primitives/sphere.cpp
)

# Armadillo linking
find_package(Armadillo REQUIRED)

message(STATUS ${CMAKE_SOURCE_DIR})

target_link_libraries(animation PRIVATE graphics ${ARMADILLO_LIBRARIES})

target_include_directories(animation PUBLIC include libs
	${ARMADILLO_INCLUDE_DIRS})
