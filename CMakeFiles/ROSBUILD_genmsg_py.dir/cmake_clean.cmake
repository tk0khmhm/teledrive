FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/teledrive/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/teledrive/msg/__init__.py"
  "src/teledrive/msg/_Teledrive.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
