function(finallab_generate_flatbuffers)
    set(oneValueArgs SCHEMA_DIR OUTPUT_DIR OUTPUT_VAR)
    cmake_parse_arguments(FBG "" "${oneValueArgs}" "" ${ARGN})

    file(GLOB FBS_FILES CONFIGURE_DEPENDS "${FBG_SCHEMA_DIR}/*.fbs")

    if(NOT FBS_FILES)
        message(STATUS "finalLab: no .fbs files in ${FBG_SCHEMA_DIR} (codegen skipped).")
        set(${FBG_OUTPUT_VAR} "" PARENT_SCOPE)
        return()
    endif()

    if(TARGET flatbuffers::flatc)
        set(FLATC_EXECUTABLE $<TARGET_FILE:flatbuffers::flatc>)
    else()
        find_program(FLATC_EXECUTABLE flatc REQUIRED)
    endif()

    file(MAKE_DIRECTORY "${FBG_OUTPUT_DIR}")

    set(GEN_HEADERS "")
    foreach(FBS ${FBS_FILES})
        get_filename_component(FBS_NAME_WE "${FBS}" NAME_WE)
        set(GEN_HEADER "${FBG_OUTPUT_DIR}/${FBS_NAME_WE}_generated.h")
        add_custom_command(
            OUTPUT "${GEN_HEADER}"
            COMMAND ${FLATC_EXECUTABLE} --cpp --scoped-enums --gen-object-api
                    -o "${FBG_OUTPUT_DIR}" "${FBS}"
            DEPENDS "${FBS}"
            COMMENT "flatc ${FBS_NAME_WE}.fbs"
            VERBATIM
        )
        list(APPEND GEN_HEADERS "${GEN_HEADER}")
    endforeach()

    set(${FBG_OUTPUT_VAR} "${GEN_HEADERS}" PARENT_SCOPE)
endfunction()
