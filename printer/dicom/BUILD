[
{ "cc_library": {
    "name": "dicom_file",
    "cc_headers": [ "dicom_file.h" ],
    "cc_sources": [ "dicom_file.cc" ],
    "dependencies": [
        "//common/base:types",
        "//common/log:log",
        "//printer/base:geometry",
        "//third_party/dcmtk:dcmtk"
    ]
} },
{ "cc_library": {
    "name": "dicom_set",
    "cc_headers": [ "dicom_set.h" ],
    "cc_sources": [ "dicom_set.cc" ],
    "dependencies": [
        ":dicom_file",
        "//common/base:callback",
        "//common/base:flags",
        "//common/log:log",
        "//common/thread:counter",
        "//common/thread:threadpool",
        "//common/util:stl",
        "//printer/base:geometry",
        "//printer/base:point_octree",
        "//printer/objects:point_octree_object"
    ]
} }
]