cmake_minimum_required(VERSION 3.10)
project(simulator)
set(CMAKE_C_STANDARD 11)#C11
set(CMAKE_CXX_STANDARD 17)#C17
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LV_USE_FREETYPE "Use freetype lib" ON)
option(LV_USE_FFMPEG "Use ffmpeg lib" ON)

set(UI_PATH ui_builder)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../${UI_PATH})

file(GLOB_RECURSE INCLUDES "lv_drivers/*.h" "lvgl/*.h" "aic_widgets/*.h" "./*.h" "../${UI_PATH}/*.h")
file(GLOB_RECURSE SOURCES  "lv_drivers/*.c" "lvgl/src/*.c" "aic_widgets/*.c" "../${UI_PATH}/*.c")

include_directories(${PROJECT_SOURCE_DIR}/lvgl/)
include_directories(${PROJECT_SOURCE_DIR}/aic_widgets/)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
SET(CMAKE_CXX_FLAGS "-O3")

if(LV_USE_FREETYPE)
	add_definitions(-DFT2_BUILD_LIBRARY)
	add_definitions(-DHAVE_UNISTD_H)
	add_definitions(-DHAVE_FCNTL_H)

	include_directories(${PROJECT_SOURCE_DIR}/freetype/include)
	include_directories(${PROJECT_SOURCE_DIR}/freetype/include/freetype)

	set(FREETYPE_PATH ${PROJECT_SOURCE_DIR}/freetype)
	file(GLOB FREETYPE_SRC
		${FREETYPE_PATH}/src/autofit/autofit.c
		${FREETYPE_PATH}/src/base/ftbase.c
		${FREETYPE_PATH}/src/base/ftbbox.c
		${FREETYPE_PATH}/src/base/ftbdf.c
		${FREETYPE_PATH}/src/base/ftbitmap.c
		${FREETYPE_PATH}/src/base/ftcid.c
		${FREETYPE_PATH}/src/base/ftfstype.c
		${FREETYPE_PATH}/src/base/ftgasp.c
		${FREETYPE_PATH}/src/base/ftglyph.c

		${FREETYPE_PATH}/src/base/ftgxval.c
		${FREETYPE_PATH}/src/base/ftinit.c
		${FREETYPE_PATH}/src/base/ftmm.c
		${FREETYPE_PATH}/src/base/ftotval.c

		${FREETYPE_PATH}/src/base/ftpatent.c
		${FREETYPE_PATH}/src/base/ftpfr.c
		${FREETYPE_PATH}/src/base/ftstroke.c
		${FREETYPE_PATH}/src/base/ftsynth.c

		${FREETYPE_PATH}/src/base/fttype1.c
		${FREETYPE_PATH}/src/base/ftwinfnt.c
		${FREETYPE_PATH}/src/bdf/bdf.c
		${FREETYPE_PATH}/src/bzip2/ftbzip2.c

		${FREETYPE_PATH}/src/cache/ftcache.c
		${FREETYPE_PATH}/src/cff/cff.c
		${FREETYPE_PATH}/src/cid/type1cid.c
		${FREETYPE_PATH}/src/gzip/ftgzip.c

		${FREETYPE_PATH}/src/lzw/ftlzw.c
		${FREETYPE_PATH}/src/pcf/pcf.c
		${FREETYPE_PATH}/src/pfr/pfr.c
		${FREETYPE_PATH}/src/psaux/psaux.c

		${FREETYPE_PATH}/src/pshinter/pshinter.c
		${FREETYPE_PATH}/src/psnames/psnames.c
		${FREETYPE_PATH}/src/raster/raster.c
		${FREETYPE_PATH}/src/sfnt/sfnt.c

		${FREETYPE_PATH}/src/smooth/smooth.c
		${FREETYPE_PATH}/src/truetype/truetype.c
		${FREETYPE_PATH}/src/type1/type1.c
		${FREETYPE_PATH}/src/type42/type42.c

		${FREETYPE_PATH}/src/winfonts/winfnt.c
		${FREETYPE_PATH}/src/base/ftdebug.c
		${FREETYPE_PATH}/src/base/ftsystem.c
	)
	set(SOURCES ${SOURCES} ${FREETYPE_SRC})
endif(LV_USE_FREETYPE)

# add ffmpeg
if(LV_USE_FFMPEG)
    set(FFMPEG_LIBRARY ${PROJECT_SOURCE_DIR}/ffmpeg/lib)
    set(FFMPEG_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ffmpeg/include)
    find_library(AVCODEC_LIB avcodec PATHS ${FFMPEG_LIBRARY})
    find_library(AVFORMAT_LIB avformat PATHS ${FFMPEG_LIBRARY})
    find_library(AVUTIL_LIB avutil PATHS ${FFMPEG_LIBRARY})
    find_library(SWSCALE_LIB swscale PATHS ${FFMPEG_LIBRARY})
    include_directories(${FFMPEG_INCLUDE_DIR})
endif(LV_USE_FFMPEG)

add_definitions(-DLVGL_STORAGE_PATH="${PROJECT_SOURCE_DIR}/../${UI_PATH}/assets/")
add_definitions(-DLVGL_DIR="L:${PROJECT_SOURCE_DIR}/../${UI_PATH}/assets/")

add_executable(main main.c ${SOURCES} ${INCLUDES})

set(TARGE_DIR "${PROJECT_SOURCE_DIR}/build")
if(DEFINED UI_BUILD_FROM_ROOT)
    if(UI_BUILD_FROM_ROOT)
        set(TARGE_DIR "${PROJECT_SOURCE_DIR}/../build")
        message(NOTICE rootbuild)
        set_target_properties(main PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../build)
    endif(UI_BUILD_FROM_ROOT)
endif()

# copy dll files to build dir
file(COPY "${PROJECT_SOURCE_DIR}/lib/"  DESTINATION ${TARGE_DIR})
file(COPY "${PROJECT_SOURCE_DIR}/ffmpeg/bin/"  DESTINATION ${TARGE_DIR})

# copy custom assets
if(EXISTS "${PROJECT_SOURCE_DIR}/../${UI_PATH}/custom/assets")
    file(COPY ${PROJECT_SOURCE_DIR}/../${UI_PATH}/custom/assets/  DESTINATION ${PROJECT_SOURCE_DIR}/../${UI_PATH}/assets/)
    message(NOTICE " exist: ${PROJECT_SOURCE_DIR}/../${UI_PATH}/custom/assets")
else()
    message(NOTICE " Not exist: ${PROJECT_SOURCE_DIR}/../${UI_PATH}/custom/assets")
endif()

add_compile_definitions(LV_CONF_INCLUDE_SIMPLE)
target_link_libraries(main PRIVATE SDL2)

if(LV_USE_FFMPEG)
    target_link_libraries(main PRIVATE ${AVCODEC_LIB}  ${AVFORMAT_LIB}  ${AVUTIL_LIB} ${SWSCALE_LIB})
endif(LV_USE_FFMPEG)
