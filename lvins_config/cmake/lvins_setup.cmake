macro(lvins_setup)
    # 设置编译选项
    set(CMAKE_CXX_STANDARD 17)

    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif ()

    if (CMAKE_BUILD_TYPE MATCHES Release)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    elseif (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3")
    elseif (CMAKE_BUILD_TYPE MATCHES Debug)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
    endif ()

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

    if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
    elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2")
    endif ()
endmacro()