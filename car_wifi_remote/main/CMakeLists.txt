idf_component_register(SRCS "audio_uac.c" "main.c" "motor.c" "servo.c" "uvc_stream.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "certs/cert.pem" "certs/key.pem"
                       REQUIRES esp_https_server esp_http_server esp_timer esp_wifi
                                esp_netif esp_event nvs_flash esp_driver_gpio
                                esp_driver_ledc esp_hw_support freertos wpa_supplicant)
