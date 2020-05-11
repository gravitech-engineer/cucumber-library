# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_INCLUDEDIRS "./")
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS main.cpp Wire.cpp BMP280.cpp)
set(COMPONENT_ADD_INCLUDEDIRS "./")


register_component()
