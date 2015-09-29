CONFIG -= qt
TARGET = OpenMesh
TEMPLATE = lib
CONFIG += staticlib
CONFIG += c++11
INCLUDEPATH += . Core/Geometry
INCLUDEPATH += $${_PRO_FILE_PWD_}/../
DEFINES += OM_STATIC_BUILD

win32{
    DESTDIR = ./
}

DEFINES += _USE_MATH_DEFINES

# Input
HEADERS += Core/Geometry/Config.hh \
           Core/Geometry/LoopSchemeMaskT.hh \
           Core/Geometry/MathDefs.hh \
           Core/Geometry/NormalConeT.hh \
           Core/Geometry/Plane3d.hh \
           Core/Geometry/QuadricT.hh \
           Core/Geometry/VectorT.hh \
           Core/Geometry/VectorT_inc.hh \
           Core/IO/BinaryHelper.hh \
           Core/IO/IOInstances.hh \
           Core/IO/IOManager.hh \
           Core/IO/MeshIO.hh \
           Core/IO/OFFFormat.hh \
           Core/IO/OMFormat.hh \
           Core/IO/Options.hh \
           Core/IO/SR_binary.hh \
           Core/IO/SR_binary_spec.hh \
           Core/IO/SR_rbo.hh \
           Core/IO/SR_store.hh \
           Core/IO/SR_types.hh \
           Core/IO/StoreRestore.hh \
           Core/Mesh/ArrayItems.hh \
           Core/Mesh/ArrayKernel.hh \
           Core/Mesh/AttribKernelT.hh \
           Core/Mesh/Attributes.hh \
           Core/Mesh/BaseKernel.hh \
           Core/Mesh/BaseMesh.hh \
           Core/Mesh/Casts.hh \
           Core/Mesh/CirculatorsT.hh \
           Core/Mesh/FinalMeshItemsT.hh \
           Core/Mesh/Handles.hh \
           Core/Mesh/IteratorsT.hh \
           Core/Mesh/PolyConnectivity.hh \
           Core/Mesh/PolyMesh_ArrayKernelT.hh \
           Core/Mesh/PolyMeshT.hh \
           Core/Mesh/Status.hh \
           Core/Mesh/Traits.hh \
           Core/Mesh/TriConnectivity.hh \
           Core/Mesh/TriMesh_ArrayKernelT.hh \
           Core/Mesh/TriMeshT.hh \
           Core/System/compiler.hh \
           Core/System/config.h \
           Core/System/config.hh \
           Core/System/mostream.hh \
           Core/System/omstream.hh \
           Core/System/OpenMeshDLLMacros.hh \
           Core/Utils/AutoPropertyHandleT.hh \
           Core/Utils/BaseProperty.hh \
           Core/Utils/color_cast.hh \
           Core/Utils/Endian.hh \
           Core/Utils/GenProg.hh \
           Core/Utils/Noncopyable.hh \
           Core/Utils/Property.hh \
           Core/Utils/PropertyContainer.hh \
           Core/Utils/PropertyManager.hh \
           Core/Utils/RandomNumberGenerator.hh \
           Core/Utils/SingletonT.hh \
           Core/Utils/vector_cast.hh \
           Core/Utils/vector_traits.hh \
           Core/IO/exporter/BaseExporter.hh \
           Core/IO/exporter/ExporterT.hh \
           Core/IO/importer/BaseImporter.hh \
           Core/IO/importer/ImporterT.hh \
           Core/IO/reader/BaseReader.hh \
           Core/IO/reader/OBJReader.hh \
           Core/IO/reader/OFFReader.hh \
           Core/IO/reader/OMReader.hh \
           Core/IO/reader/PLYReader.hh \
           Core/IO/reader/STLReader.hh \
           Core/IO/writer/BaseWriter.hh \
           Core/IO/writer/OBJWriter.hh \
           Core/IO/writer/OFFWriter.hh \
           Core/IO/writer/OMWriter.hh \
           Core/IO/writer/PLYWriter.hh \
           Core/IO/writer/STLWriter.hh \
           Core/IO/writer/VTKWriter.hh \
           Core/Mesh/gen/circulators_header.hh \
           Core/Mesh/gen/circulators_template.hh \
           Core/Mesh/gen/footer.hh \
           Core/Mesh/gen/iterators_header.hh \
           Core/Mesh/gen/iterators_template.hh \
           Core/Geometry/NormalConeT.cc
SOURCES += Core/Geometry/NormalConeT.cc \
           Core/IO/BinaryHelper.cc \
           Core/IO/IOManager.cc \
           Core/IO/OMFormat.cc \
           Core/IO/OMFormatT.cc \
           Core/Mesh/ArrayKernel.cc \
           Core/Mesh/ArrayKernelT.cc \
           Core/Mesh/BaseKernel.cc \
           Core/Mesh/PolyConnectivity.cc \
           Core/Mesh/PolyMeshT.cc \
           Core/Mesh/TriConnectivity.cc \
           Core/Mesh/TriMeshT.cc \
           Core/System/omstream.cc \
           Core/Utils/BaseProperty.cc \
           Core/Utils/Endian.cc \
           Core/Utils/RandomNumberGenerator.cc \
           Core/Utils/SingletonT.cc \
           Core/IO/reader/BaseReader.cc \
           Core/IO/reader/OBJReader.cc \
           Core/IO/reader/OFFReader.cc \
           Core/IO/reader/OMReader.cc \
           Core/IO/reader/PLYReader.cc \
           Core/IO/reader/STLReader.cc \
           Core/IO/writer/BaseWriter.cc \
           Core/IO/writer/OBJWriter.cc \
           Core/IO/writer/OFFWriter.cc \
           Core/IO/writer/OMWriter.cc \
           Core/IO/writer/PLYWriter.cc \
           Core/IO/writer/STLWriter.cc \
           Core/IO/writer/VTKWriter.cc

