include(FetchContent)

function(fetch_project)
    cmake_parse_arguments(FETCH_SOURCE "" "NAME;URL" "" ${ARGN})
    FetchContent_Declare(${FETCH_SOURCE_NAME}
        URL ${FETCH_SOURCE_URL}
    )

    FetchContent_MakeAvailable(${FETCH_SOURCE_NAME})
endfunction()