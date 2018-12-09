cmake_minimum_required(VERSION 2.8.3)
project(mrl_rqt_dashboard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rqt_gui
  rqt_gui_cpp
  std_msgs
  image_transport
  geometry_msgs
  cv_bridge
  rviz
)


if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED )
  find_package(Qt5WebKit REQUIRED)
  find_package(Qt5WebKitWidgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui widgets webkitwidgets REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs rviz
)


file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resource/resource.qrc resource/qdarkstyle/style.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/mrl_rqt_dashboard/*.h)




file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/mrl_rqt_dashboard/*.cpp)


file(GLOB_RECURSE QT_UI RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/mrl_rqt_dashboard/*.ui)

set(mrl_rqt_dashboard_UIS
  src/mrl_rqt_dashboard/dashboard.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")


  QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
  QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
  QT5_WRAP_UI(QT_UI_H ${QT_UI})


else()
  qt4_wrap_cpp(mrl_rqt_dashboard_MOCS ${mrl_rqt_dashboard_HDRS})
  qt4_wrap_ui(mrl_rqt_dashboard_UIS_H ${mrl_rqt_dashboard_UIS})
endif()

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}

    ${QT_SOURCES}
    ${QT_MOC_HPP}
    ${QT_RESOURCES_CPP}
    ${QT_MOC}
    ${QT_UI_H}
    )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}

)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::WebKit Qt5::WebKitWidgets
)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

