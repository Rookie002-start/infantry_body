# modules/dbg —— 公共调参终端（给用双环 PID 的业务对象在线改参数）
#
# 头文件始终可见：业务线程 include dbg.hpp 登记自己的 Pid（关掉 CONFIG_DBG 时那些
# 调用退化成空操作，业务代码不用写 #if）；命令行线程只在 CONFIG_DBG=y 时编译。
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(CONFIG_DBG)
  zephyr_library_named(dbg)
  zephyr_library_sources(dbg.cpp)
endif()
