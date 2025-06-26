# generated from ament_cmake_export_targets/cmake/ament_cmake_export_targets-extras.cmake.in

set(_exported_targets "export_traffic_information_msgs__rosidl_generator_c;export_traffic_information_msgs__rosidl_typesupport_fastrtps_c;export_traffic_information_msgs__rosidl_generator_cpp;export_traffic_information_msgs__rosidl_typesupport_fastrtps_cpp;traffic_information_msgs__rosidl_typesupport_introspection_c;traffic_information_msgs__rosidl_typesupport_c;traffic_information_msgs__rosidl_typesupport_introspection_cpp;traffic_information_msgs__rosidl_typesupport_cpp;export_traffic_information_msgs__rosidl_generator_py")

# include all exported targets
if(NOT _exported_targets STREQUAL "")
  foreach(_target ${_exported_targets})
    set(_export_file "${traffic_information_msgs_DIR}/${_target}Export.cmake")
    include("${_export_file}")

    # extract the target names associated with the export
    set(_regex "foreach\\((_cmake)?_expected_?[Tt]arget (IN ITEMS )?(.+)\\)")
    file(
      STRINGS "${_export_file}" _foreach_targets
      REGEX "${_regex}")
    list(LENGTH _foreach_targets _matches)
    if(NOT _matches EQUAL 1)
      message(FATAL_ERROR
        "Failed to find exported target names in '${_export_file}'")
    endif()
    string(REGEX REPLACE "${_regex}" "\\3" _targets "${_foreach_targets}")
    string(REPLACE " " ";" _targets "${_targets}")
    list(LENGTH _targets _length)

    list(APPEND traffic_information_msgs_TARGETS ${_targets})
  endforeach()
endif()
