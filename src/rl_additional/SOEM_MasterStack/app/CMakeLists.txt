
set(SOURCES motor_control.c
            motor_control.h
            math_ops.c
            math_ops.h
            transmit.c
            transmit.h
            config.h
)
LINK_LIBRARIES(m)
add_library(motor ${SOURCES})
target_link_libraries(motor soem)

add_executable(master_stack_test main.c)
target_link_libraries(master_stack_test motor soem)


add_executable(cppmaster_stack_test main.cpp)
target_link_libraries(master_stack_test motor soem)

