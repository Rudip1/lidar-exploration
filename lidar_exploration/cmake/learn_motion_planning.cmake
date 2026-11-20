# The motion-planning library (occupancy grids, distance transform, Dubins RRT*, Path2D, pure pursuit) from
# learn-motion-planning, pinned to a commit. Only the C++ library target `motion_planning` is built: no tests,
# examples or Python module.
#
# Offline or with a local checkout:  colcon build --cmake-args -DFETCHCONTENT_SOURCE_DIR_LEARN_MOTION_PLANNING=/path
include(FetchContent)
set(MP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MP_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    learn_motion_planning
    GIT_REPOSITORY https://github.com/Rudip1/learn-motion-planning.git
    GIT_TAG fa8984081eede3047d2e12b4752c6ed2be83ae79
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(learn_motion_planning)
