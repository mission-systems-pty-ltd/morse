#Python_add_library(sickc sickcmodule.c)
#INSTALL(TARGETS sickc DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

Python_add_library(zbuffertodepth zbuffertodepth.c)
INSTALL(TARGETS zbuffertodepth DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

Python_add_library(zbufferto3d zbufferto3d.c)
INSTALL(TARGETS zbufferto3d DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

Python_add_library(depthaggregator depthaggregator.c)
INSTALL(TARGETS depthaggregator DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

Python_add_library(_magnetometer magnetometer.c GeomagnetismLibrary.c)
INSTALL(TARGETS _magnetometer DESTINATION ${PYTHON_INSTDIR}/morse/sensors)
INSTALL(FILES WMM.COF DESTINATION ${CMAKE_INSTALL_PREFIX}/share/morse/data)

# install python files
FILE(GLOB PYTHON_FILES *.py)
INSTALL(FILES ${PYTHON_FILES} DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

# install capnp files
FILE(GLOB CAPNP_FILES *.capnp)
INSTALL(FILES ${CAPNP_FILES} DESTINATION ${PYTHON_INSTDIR}/morse/sensors)
