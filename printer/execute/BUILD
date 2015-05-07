[
{ "cc_library": {
    "name": "print_box",
    "cc_headers": [ "print_box.h" ],
    "cc_sources": [ "print_box.cc" ],
    "dependencies": [ "//common/base:macros",
                      "//common/base:types",
                      "//common/log:log",
                      "//printer/base:geometry" ]
} },

{ "cc_library": {
    "name": "marching_cubes",
    "cc_headers": [ "marching_cubes.h", "marching_cubes_tables.h" ],
    "cc_sources": [ "marching_cubes.cc", "marching_cubes_tables.cc" ],
    "dependencies": [ ":print_box",
                      "//common/base:callback",
                      "//common/base:flags",
                      "//common/log:log",
                      "//common/thread:counter",
                      "//common/thread:threadpool",
                      "//printer/base:geometry",
                      "//printer/base:mesh"
    ]
} },
{ "cc_library": {
    "name": "voxel_fill",
    "cc_headers": [ "voxel_fill.h" ],
    "cc_sources": [ "voxel_fill.cc" ],
    "dependencies": [
        ":print_box",
        "//common/base:flags",
        "//common/base:macros",
        "//common/log:log",
        "//common/thread:threadpool",
        "//printer/base:geometry",
        "//printer/objects:object"
    ]
} },

{ "cc_library": {
    "name": "driver",
    "cc_headers": [ "driver.h" ],
    "cc_sources": [ "driver.cc" ],
    "dependencies": [
        ":marching_cubes",
        ":print_box",
        ":voxel_fill",
        "//common/base:flags",
        "//common/base:macros",
        "//common/log:log",
        "//common/thread:threadpool",
        "//printer/base:mesh",
        "//printer/simplify:face_reduction",
        "//printer/simplify:simplifier",
        "//printer/objects:object"
    ]
} },

{ "cc_test": {
    "name": "marching_cubes_test",
    "cc_sources": [ "marching_cubes_test.cc" ],
    "dependencies": [ ":marching_cubes",
                      "//common/base:flags",
                      "//common/log:log",
                      "//common/test:test_main"
    ]
} }
]