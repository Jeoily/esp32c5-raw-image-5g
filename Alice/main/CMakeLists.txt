idf_component_register(SRCS "../../shared/link.c" INCLUDE_DIRS "." REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_driver_uart)
target_compile_definitions(${COMPONENT_LIB} PRIVATE ROLE_ALICE=1)
