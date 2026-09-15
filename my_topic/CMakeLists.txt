if(CONFIG_DUST_TPC_TO_MCU_TX)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/to_mcu)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/to_mcu/to_mcu_tx.cpp)
endif()

if(CONFIG_DUST_TPC_FROM_HEAD)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/from_head)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/from_head/from_head.cpp)
endif()

if(CONFIG_DUST_TPC_GIMBAL_TO)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gimbal_to)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gimbal_to/gimbal_to.cpp)
endif()
