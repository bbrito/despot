cmake_minimum_required(VERSION 2.8.3)

add_executable("${PROJECT_NAME}_POMDP"
        src/spline.cpp
        src/Clothoid.cpp
        src/POMDP.cpp
        src/main.cpp
        )
target_link_libraries("${PROJECT_NAME}_POMDP"
        "${PROJECT_NAME}"
        )
install(TARGETS "${PROJECT_NAME}_POMDP"
        RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
        )




