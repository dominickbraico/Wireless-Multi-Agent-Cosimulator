# Built by symlinking this directory into ns-3's scratch/ (see ns3/build_ns3.sh).
find_package(yaml-cpp REQUIRED)
build_exec(
  EXECNAME wifi_adhoc_tap
  SOURCE_FILES wifi_adhoc_tap.cc
  LIBRARIES_TO_LINK ${libcore} ${libnetwork} ${libwifi} ${libmobility}
                    ${libpropagation} ${libtap-bridge} yaml-cpp
  EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/wifi_adhoc_tap
)
