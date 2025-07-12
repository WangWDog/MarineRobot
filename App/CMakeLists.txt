# 自动收集所有 .c 文件
file(GLOB_RECURSE APP_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/*/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/app_main.c
)

# 添加 App 源文件到目标可执行文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${APP_SOURCES})

# 添加 App 子目录的头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}/BMI088
        ${CMAKE_CURRENT_SOURCE_DIR}/SHT30
)