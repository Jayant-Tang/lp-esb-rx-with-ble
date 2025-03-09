target_sources(app PRIVATE
  ./app_esb.c
  ./app_mpsl.c
)

target_include_directories(app PRIVATE .)

# Add configs
list(APPEND EXTRA_CONF_FILE
  "${CMAKE_CURRENT_LIST_DIR}/mpsl-esb-overlay.conf"
)

set(EXTRA_CONF_FILE ${EXTRA_CONF_FILE} CACHE INTERNAL "")