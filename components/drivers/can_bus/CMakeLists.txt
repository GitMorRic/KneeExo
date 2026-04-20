idf_component_register(
    SRCS "can_bus.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer log
)
