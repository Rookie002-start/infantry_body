# modules/vofa+ —— VOFA+ JustFloat 波形输出组件
#
# 库名显式取 "vofa"：目录名带 "+"，让 zephyr_library() 按目录推断会得到
# 一个带 "+" 的畸形 target 名。
zephyr_library_named(vofa)
zephyr_library_sources(vofa.cpp)

# 业务代码直接 #include "vofa.hpp"
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
