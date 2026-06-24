cmake_minimum_required(VERSION 3.8)
project(rl_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes
install(DIRECTORY models/crazyflie/meshes
  DESTINATION share/${PROJECT_NAME}/models/crazyflie
)

# Generate model.sdf at install time with absolute mesh path baked in,
# so gz sim's per-drone model copies (which live in sibling directories)
# can still resolve the meshes.
install(CODE "
  set(MESHES_PATH \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/crazyflie/meshes\")
  file(READ \"${CMAKE_CURRENT_SOURCE_DIR}/models/crazyflie/model.sdf.in\" _sdf)
  string(REPLACE \"__MESHES_PATH__\" \"file://\${MESHES_PATH}\" _sdf \"\${_sdf}\")
  file(WRITE \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/crazyflie/model.sdf\" \"\${_sdf}\")
")

install(FILES models/crazyflie/model.config
  DESTINATION share/${PROJECT_NAME}/models/crazyflie
  OPTIONAL
)

install(PROGRAMS
  scripts/tune_tracking.py
  scripts/waypoint_player.py
  scripts/prepare_arena.py
  scripts/save_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/record_video.sh
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
