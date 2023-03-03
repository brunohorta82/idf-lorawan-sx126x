idf_component_register(SRC_DIRS . src radio/src  mac/src crypto/src board/src timer/src
                        INCLUDE_DIRS . inc radio/inc  mac/inc crypto/inc board/inc timer/inc
                        REQUIRES freertos 
                                    esp_common 
                                    esp_event 
                                    nvs_flash 
                                    esp_timer
                                    driver
                                    )
