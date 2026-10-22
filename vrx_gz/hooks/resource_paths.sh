ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/vrx_gz/worlds"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/vrx_gz/worlds/2023_practice"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/vrx_gz/worlds/2023_phase2"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/vrx_gz/models"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/vrx_gz/models/tmp"
ament_prepend_unique_value GZ_SIM_SYSTEM_PLUGIN_PATH "$AMENT_CURRENT_PREFIX/lib"

ament_prepend_unique_value LD_LIBRARY_PATH "$AMENT_CURRENT_PREFIX/lib"
# ament_prepend_unique_value 这是 ament 提供的一个 bash 函数。它的作用是安全地往环境变量前面追加路径。
# $AMENT_CURRENT_PREFIX，这个变量会在你 source 环境时自动指向你的工作空间安装目录