#.rst
# FindCVB
# -------
#
# Finding and using CVB
# ^^^^^^^^^^^^^^^^^^^^^
#
# This module can be used to find Common Vision Blox (CVB).  An installed 
# version of CVB defines the ``CVB`` environment variable pointing to its root.
#
# Typical usage could be something like:
#
# .. code-block:: cmake
#
#    find_package(CVB REQUIRED CVCImg CVCUtilities)
#    add_executable(myexe main.cpp)
#    target_link_libraries(myexe CVB::CVCUtilities)
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# A particular CVB library may be used by using the corresponding 
# :prop_tgt:`IMPORTED`target with the :command:`target_link_libraries' 
# command:
#
# .. code-block:: cmake
#
#    target_link_libraries(myexe CVB::CVCUtilities)
#
# Using a target in this way causes :cmake(1)`to use the appropriate include
# directories and compile definitions for the target when compiling ``myexe``.
# At least when cmake supports this (version 3 and above). Use 
# ``CVB_INCLUDE_DIR`` otherwise:
#
# .. code-block:: cmake
#
#    include_directories(${CVB_INCLUDE_DIR})
#
# Targets are aware of their dependencies, so for example it is not necessary
# to list ``CVB::CVCImg`` if another CVB library is listed. Targets may be 
# tested for existence in the usual way with the :command:`if(TARGET)` command.
# (This is also only supported starting with :cmake(1)` version 3 and above).
#
# CVB only contains release libraries.
#
# ``CVB::CVCImg``
#  The core CVB image target.
# ``CVB::CVCDriver``
#  The driver interfaces target.
# ``CVB::CVCUtilities``
#  CVB utility functions target.
# ``CVB::CVGenApi``
#  The GenApi target.
#
# The following targets are available if installed:
#
# ``CVB::CVGevServer``
#  GigE Vision Server target.
# ``CVB::MinosCVC``
#  CVB Minos target.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# Below is a detailed list of variables that FindCVB.cmake sets.
#
# ``CVB_FOUND``
#  If false, don't try to use CVB.
# ``CVB_INCLUDE_DIR``
#  The CVB include directory

find_path(CVB_INCLUDE_DIR iCVCImg.h HINTS "$ENV{CVB}" 
                                    PATH_SUFFIXES Lib/C "include"
)

# mandatory libraries
find_library(LIB_CVCIMG CVCImg PATHS "$ENV{CVB}" PATH_SUFFIXES Lib/C lib)
find_library(LIB_CVCDRIVER CVCDriver PATHS "$ENV{CVB}" PATH_SUFFIXES Lib/C lib)
find_library(LIB_CVCUTILITIES CVCUtilities PATHS "$ENV{CVB}" PATH_SUFFIXES Lib/C lib)
find_library(LIB_CVGENAPI CVGenApi PATHS "$ENV{CVB}" PATH_SUFFIXES Lib/C lib)
mark_as_advanced(LIB_CVCIMG LIB_CVCDRIVER LIB_CVCUTILITIES LIB_CVGENAPI)

# optional libraries
find_library(LIB_CVGEVSERVER CVGevServer PATHS "$ENV{CVB}" PATH_SUFFIXES Lib/C lib)
find_library(LIB_MINOSCVC MinosCVC PATHS "$ENV{CVB}" PATH_SUFFIXES Lib/C lib)
mark_as_advanced(LIB_CVGEVSERVER LIB_MINOSCVC)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(CVB "Could not find CVB. Install Common Vision Blox via its install package."
  CVB_INCLUDE_DIR
  LIB_CVCIMG
  LIB_CVCDRIVER
  LIB_CVCUTILITIES
  LIB_CVGENAPI
)

if(CVB_FOUND)

  if(NOT TARGET CVB::CVCImg)
    add_library(CVB::CVCImg UNKNOWN IMPORTED)
    set_target_properties(CVB::CVCImg PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CVB_INCLUDE_DIR}"
      IMPORTED_LOCATION             "${LIB_CVCIMG}")
  endif()

  if(NOT TARGET CVB::CVGenApi)
    add_library(CVB::CVGenApi UNKNOWN IMPORTED)
    set_target_properties(CVB::CVGenApi PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CVB_INCLUDE_DIR}"
      IMPORTED_LOCATION             "${LIB_CVGENAPI}"
      INTERFACE_LINK_LIBRARIES      CVB::CVCImg)
    if(UNIX)
      set_target_properties(CVB::CVGenApi PROPERTIES
        INTERFACE_LINK_LIBRARIES    -Wl,-rpath-link,$ENV{CVB}/lib/genicam)
    endif()
  endif()

  if(NOT TARGET CVB::CVCDriver)
    add_library(CVB::CVCDriver UNKNOWN IMPORTED)
    set_target_properties(CVB::CVCDriver PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CVB_INCLUDE_DIR}"
      IMPORTED_LOCATION             "${LIB_CVCDRIVER}"
      INTERFACE_LINK_LIBRARIES      "CVB::CVCImg;CVB::CVGenApi")
  endif()

  if(NOT TARGET CVB::CVCUtilities)
    add_library(CVB::CVCUtilities UNKNOWN IMPORTED)
    set_target_properties(CVB::CVCUtilities PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CVB_INCLUDE_DIR}"
      IMPORTED_LOCATION             "${LIB_CVCUTILITIES}"
      INTERFACE_LINK_LIBRARIES      CVB::CVCImg)
  endif()

  if(NOT TARGET CVB::CVGevServer AND LIB_CVGEVSERVER)
    add_library(CVB::CVGevServer UNKNOWN IMPORTED)
    set_target_properties(CVB::CVGevServer PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CVB_INCLUDE_DIR}"
      IMPORTED_LOCATION             "${LIB_CVGEVSERVER}"
      INTERFACE_LINK_LIBRARIES      "CVB::CVCImg;CVB::CVCDriver;CVB::CVCUtilities")
  endif()

  if(NOT TARGET CVB::MinosCVC AND LIB_MINOSCVC)
    add_library(CVB::MinosCVC UNKNOWN IMPORTED)
    set_target_properties(CVB::MinosCVC PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CVB_INCLUDE_DIR}"
      IMPORTED_LOCATION             "${LIB_MINOSCVC}"
      INTERFACE_LINK_LIBRARIES      CVB::CVCImg)
  endif()

endif()
