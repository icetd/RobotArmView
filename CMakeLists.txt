cmake_minimum_required(VERSION 3.10)
project(RobotArmView)

add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")
add_subdirectory(thirdparty/glfw)
add_subdirectory(thirdparty/glm)
add_subdirectory(thirdparty/assimp)

add_definitions(-w)

set (CMKAE_CXX_STANDARD 14)
set (CMAKE_BUILD_TYPE Release)
set (CMAKE_INCLUDE_CURRENT_DIR ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set (SOURCE_DIR  "${CMAKE_CURRENT_LIST_DIR}/src") 
set (INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/src")

set (GLFW_INC           "${CMAKE_CURRENT_LIST_DIR}/thirdparty/glfw/include")
set (GLAD_SRC           "${CMAKE_CURRENT_LIST_DIR}/thirdparty/glad/src")
set (GLAD_INC           "${CMAKE_CURRENT_LIST_DIR}/thirdparty/glad/include")
set (STB_IMAGE_INC      "${CMAKE_CURRENT_LIST_DIR}/thirdparty/stb_image")
set (GLM_INC            "${CMAKE_CURRENT_LIST_DIR}/thirdparty/glm")
set (ASSIMP_INC         "${CMAKE_CURRENT_LIST_DIR}/thirparty/assimp/include")
set (IMPLOT_SRC         "${CMAKE_CURRENT_LIST_DIR}/thirdparty/implot")
set (IMGUI_SRC          "${CMAKE_CURRENT_LIST_DIR}/thirdparty/imgui")
set (IMGUIZMO_SRC       "${CMAKE_CURRENT_LIST_DIR}/thirdparty/ImGuizmo")
set (IMPLOT_SRC         "${CMAKE_CURRENT_LIST_DIR}/thirdparty/implot")
set (URDF_PARSER_INC    "${CMAKE_CURRENT_LIST_DIR}/thirdparty/urdf_parser/include")
set (URDF_PARSER_SRC    "${CMAKE_CURRENT_LIST_DIR}/thirdparty/urdf_parser/src")
set (VLD_INCLUDE_DIR    "${CMAKE_SOURCE_DIR}/thirdparty/VLD/include")
set (VLD_LIB_DIR        "${CMAKE_SOURCE_DIR}/thirdparty/VLD/lib")

set(ENGIN_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/thirdparty/eigen")
set(KDL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/thirdparty/orocos_kdl/src")
set(KDL_SRC_DIR "${CMAKE_SOURCE_DIR}/thirdparty/orocos_kdl/src")

link_directories(
    ${VLD_LIB_DIR}
    ${FFMPEG_LIB_DIR}
)

include_directories(
    ${INCLUDE_DIR}
    ${GLFW_INC}
    ${GLAD_INC}
    ${STB_IMAGE_INC}
    ${GLM_INC}
    ${ENGIN_INCLUDE_DIR}
    ${KDL_INCLUDE_DIR}
    ${ASSIMP_INC}
    ${YAML_CPP_INC}
    ${IMGUI_SRC}
    ${FFMPEG_INCLUDE_DIR}
    ${IMPLOT_SRC}
    ${IMGUIZMO_SRC}
    ${SQLITE3_INC}
    ${URDF_PARSER_INC}
    ${VLD_INCLUDE_DIR}
) 

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

file(GLOB_RECURSE SRC_FILES
    "${SOURCE_DIR}/*.c*"
    "${SOURCE_DIR}/Graphics/Buffers/*.c*"
    "${SOURCE_DIR}/Graphics/Renderer/*.c*"
    "${SOURCE_DIR}/Graphics/Objects/*.c*"
    "${SOURCE_DIR}/Graphics/Io/*.c*"
    "${SOURCE_DIR}/Graphics/Ui/*.c*"
    "${SOURCE_DIR}/Kinematics/*.c*"
    "${SOURCE_DIR}/Core/*.c*"
    "${IMGUIZMO_SRC}/*.c*"
    "${GLAD_SRC}/*c*"
    "${IMGUI_SRC}/*.c*"
    "${IMPLOT_SRC}/*.c*"
    "${URDF_PARSER_SRC}/*.c*"
    "${KDL_SRC_DIR}/*.c*"
    "${STB_IMAGE_INC}/*.h"
)

file (GLOB_RECURSE INC_FILES
    "${INCLUDE_DIR}/*.h"
    "${IMGUI_SRC}/*.h"
    "${IMPLOT_SRC}/*.h"
    "${IMGUIZMO_SRC}/*.h"
    "${GLAD_INC}/*h"
)

file (GLOB_RECURSE RES_FILES
    "${CMAKE_SOURCE_DIR}/res/*.rc"
)


source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${SRC_FILES})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${INC_FILES})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${RES_FILES})

list(APPEND EXTRA_LIBS
    ws2_32 assimp opengl32 glfw User32 Gdi32 Shell32 
)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${INC_FILES} ${RES_FILES})

# Add the target includes for MY_PROJECT 
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${SOURCE_DIR})


#===================== LINKING LIBRARIES =======================#
target_link_libraries(${PROJECT_NAME} ${EXTRA_LIBS})

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output)

file(COPY ${CMAKE_SOURCE_DIR}/res DESTINATION ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE})
file(COPY ${CMAKE_SOURCE_DIR}/configs DESTINATION ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE})
file (GLOB FFMPEG_COPY_DLL "${CMAKE_SOURCE_DIR}/thirdparty/FFmpeg/dll/*")
file(COPY ${FFMPEG_COPY_DLL} DESTINATION ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE})