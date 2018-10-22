INCLUDEPATH += $${_PRO_FILE_PWD_}/../

#Generate binary executables at the root of the build folder
DESTDIR = $${OUT_PWD}/../
DEFINES += OM_STATIC_BUILD

win32{
    LIBS += "$${OUT_PWD}/../OpenMesh/OpenMesh.lib"
    LIBS += opengl32.lib
    LIBS += glu32.lib
}

unix{
    LIBS += -L$${OUT_PWD}/../OpenMesh -lOpenMesh

    macx{
        LIBS += -framework OpenGL
    }
    else{
        LIBS += -lGLU
    }
}
