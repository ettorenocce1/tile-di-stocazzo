# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.


idf_component_register(
    SRCS main.c ../../LibreriaDisplay/xi2c.c ../../LibreriaDisplay/fonts.c ../../LibreriaDisplay/ssd1306.c         # list the source files of this component
    INCLUDE_DIRS ../../LibreriaDisplay       # optional, add here public include directories
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES            # optional, list the public requirements (component names)
    PRIV_REQUIRES       # optional, list the private requirements
)