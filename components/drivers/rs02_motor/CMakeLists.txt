idf_component_register(
    SRCS "rs02_motor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES can_bus config driver esp_timer log
)
