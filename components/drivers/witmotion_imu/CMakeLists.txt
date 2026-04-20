idf_component_register(
    SRCS "witmotion_imu.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer log
)
