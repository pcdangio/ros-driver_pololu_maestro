TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += \
    /opt/ros/melodic/include \
    $$PWD/../../devel/include

SOURCES += \
    src/driver.cpp \
    src/main.cpp \
    src/ros_node.cpp

DISTFILES += \
    CMakeLists.txt \
    README.md \
    msg/servo_position.msg \
    msg/servo_target.msg \
    package.xml

HEADERS += \
    src/driver.h \
    src/ros_node.h
