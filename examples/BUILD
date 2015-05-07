[
{ "cc_library": {
    "name": "example_base",
    "dependencies": [
        "//common/base:base_tcmalloc",
        "//printer/objects:all_objects",
        "//printer/stl:driver"
    ]
} },

{ "cc_binary": {
    "name": "sphere",
    "cc_sources": [ "sphere.cc" ],
    "dependencies": [ ":example_base" ]
} },

{ "cc_binary": {
    "name": "cylinder",
    "cc_sources": [ "cylinder.cc" ],
    "dependencies": [ ":example_base" ]
} },

{ "cc_binary": {
    "name": "rotated",
    "cc_sources": [ "rotated.cc" ],
    "dependencies": [ ":example_base" ]
} },

{ "cc_binary": {
    "name": "buckle",
    "cc_sources": [ "buckle.cc" ],
    "dependencies": [ ":example_base" ]
} },

{ "cc_binary": {
    "name": "merged",
    "cc_sources": [ "merged.cc" ],
    "dependencies": [ ":example_base" ]
} },

{ "cc_binary": {
    "name": "mandel",
    "cc_sources": [ "mandel.cc" ],
    "dependencies": [ ":example_base" ]
} },

{ "cc_binary": {
    "name": "print_dicom",
    "cc_sources": [ "print_dicom.cc" ],
    "dependencies": [
        "//common/base:base_tcmalloc",
        "//common/file:fileutil",
        "//common/strings:strutil",
        "//printer/dicom:dicom_set",
        "//printer/execute:driver",
        "//printer/execute:print_box",
        "//printer/objects:object",
        "//printer/stl:format"
    ]
} }
]
