if(MSVC AND NOT "${MSVC_VERSION}" LESS 1400)
	add_definitions( "/MP" )
endif()

add_library(graphics INTERFACE)

#Link OPENGL library
cmake_policy(SET "CMP0072" OLD)
find_package(OpenGL REQUIRED)
set(OPENGL_LIBRARY_FULL
	${OPENGL_LIBRARY}
	-lGL -lGLU -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi
	${CMAKE_DL_LIBS})

# Create GLAD library
add_library(GLAD STATIC
	glad/glad.h
	glad/glad.c)

# Create IMGui library
add_library(IMGUI STATIC
	imgui/imgui_impl_glfw.h
	imgui/imgui_impl_glfw.cpp
	imgui/imgui_impl_opengl3_loader.h
	imgui/imgui_impl_opengl3.h
	imgui/imgui_impl_opengl3.cpp
	imgui/imgui_widgets.cpp
	imgui/imgui_draw.cpp
	imgui/imgui_tables.cpp
	imgui/imgui.h
	imgui/imgui.cpp
	imgui/imgui_demo.cpp
	imgui/imconfig.h
	imgui/imgui_internal.h
	imgui/imstb_textedit.h
	imgui/imstb_rectpack.h
	imgui/imstb_truetype.h)

target_link_libraries(GLAD PRIVATE ${OPENGL_LIBRARY_FULL})
target_include_directories(GLAD PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(IMGUI PRIVATE ${OPENGL_LIBRARY_FULL})
target_include_directories(IMGUI PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(graphics INTERFACE ${OPENGL_LIBRARY_FULL} GLAD IMGUI)
