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
    "name": "branching",
    "cc_sources": [ "branching.cc" ],
    "dependencies": [ ":example_base" ]
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
    "name": "barrel",
    "cc_sources": [ "barrel.cc" ],
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
    "name": "smooth_spheroid",
    "cc_sources": [ "smooth_spheroid.cc" ],
    "dependencies": [ ":example_base" ]
} },
{ "cc_binary": {
    "name": "print_dicom",
    "cc_sources": [ "print_dicom.cc" ],
    "dependencies": [
        "//common/base:base",  // Not _tcmalloc, uses too much memory?
        "//common/file:fileutil",
        "//common/strings:strutil",
        "//printer/dicom:dicom_set",
        "//printer/execute:driver",
        "//printer/execute:print_box",
        "//printer/objects:object",
        "//printer/objects:primitives",
        "//printer/objects:transform",
        "//printer/stl:format"
    ]
} }
]
