TEMPLATE = subdirs

SUBDIRS += \
    OpenMesh \
    Denoising

Denoising.depends = OpenMesh

