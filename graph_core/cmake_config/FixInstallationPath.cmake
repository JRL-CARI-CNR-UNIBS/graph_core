# Define ANSI color sequences for messages
string(ASCII 27 Esc)
set(RESET "${Esc}[0m")
set(BLUE "${Esc}[34m")

# Paths for cnr_yaml's cmake_alternative and cmake
set(CNR_YAML_CMAKE_ALTERNATIVE_PATH "${CMAKE_INSTALL_PREFIX}/share/cnr_yaml/cmake_alternative")
set(CNR_YAML_CMAKE_PATH "${CMAKE_INSTALL_PREFIX}/share/cnr_yaml/cmake")

# Logic to copy cmake_alternative to cmake
install(CODE "
    if(EXISTS \"${CNR_YAML_CMAKE_ALTERNATIVE_PATH}\")
        message(STATUS \"${BLUE}Copying cmake_alternative to cmake for cnr_yaml during installation.${RESET}\")
        
        # Create the cmake directory if it doesn't exist
        file(MAKE_DIRECTORY \"${CNR_YAML_CMAKE_PATH}\")
        
        # Copy files from cmake_alternative to cmake
        file(GLOB CMAKE_FILES_TO_COPY \"${CNR_YAML_CMAKE_ALTERNATIVE_PATH}/*\")
        foreach(FILE \${CMAKE_FILES_TO_COPY})
            file(COPY \${FILE} DESTINATION \"${CNR_YAML_CMAKE_PATH}\")
        endforeach()

        message(STATUS \"${BLUE}Copied ${CNR_YAML_CMAKE_ALTERNATIVE_PATH} to ${CNR_YAML_CMAKE_PATH}.${RESET}\")
    else()
        message(WARNING \"${BLUE}cmake_alternative directory not found: ${CNR_YAML_CMAKE_ALTERNATIVE_PATH}.${RESET}\")
    endif()
")
