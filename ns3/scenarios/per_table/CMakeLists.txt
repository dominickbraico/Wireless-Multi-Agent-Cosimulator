build_exec(
  EXECNAME per_table
  SOURCE_FILES per_table.cc
  LIBRARIES_TO_LINK ${libcore} ${libnetwork} ${libwifi}
  EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/per_table
)
