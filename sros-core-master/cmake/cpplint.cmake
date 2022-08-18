
set(CPPLINT_ARG_OUTPUT     "--output=eclipse")
set(CPPLINT_ARG_VERBOSE    "--verbose=3")
set(CPPLINT_ARG_LINELENGTH "--linelength=120")

option(SROS_RUN_CPPLINT "Run cpplint.py tool for Google C++ StyleGuide." ON)

if (SROS_RUN_CPPLINT)
    message(STATUS "SROS_RUN_CPPLINT ON")
    find_package(PythonInterp)
    if (PYTHONINTERP_FOUND)
        # add a cpplint target to the "all" target
        add_custom_target(SROS_cpplint
                ALL
                COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/cpplint.py
                    ${CPPLINT_ARG_OUTPUT} ${CPPLINT_ARG_VERBOSE} ${CPPLINT_ARG_LINELENGTH}
                    ${CMAKE_SOURCE_DIR}
                )
    endif (PYTHONINTERP_FOUND)
else (SROS_RUN_CPPLINT)
    message(STATUS "SROS_RUN_CPPLINT OFF")
endif (SROS_RUN_CPPLINT)
