[
{ "cc_library": {
    "name": "simplifier",
    "cc_sources": [ "simplifier.cc", "simplifier_heap.cc" ],
    "cc_headers": [ "simplifier.h", "simplifier_heap.h" ],
    "dependencies": [
        "//common/base:callback",
        "//common/base:flags",
        "//common/base:macros",
        "//common/thread:counter",
        "//common/thread:threadpool",
        "//common/util:stl",
        "//common/log:log",
        "//printer/base:mesh"
    ]
} },

{ "cc_library": {
    "name": "triangulation",
    "cc_sources": [ "triangulation.cc", "sweep_triangulation.cc" ],
    "cc_headers": [ "triangulation.h", "sweep_triangulation.h" ],
    "dependencies": [
        "//common/log:log",
        "//printer/base:geometry"
    ]
} },

{ "cc_library": {
    "name": "face_reduction",
    "cc_sources": [ "face_reduction.cc" ],
    "cc_headers": [ "face_reduction.h" ],
    "dependencies": [
        ":triangulation",
        "//common/log:log",
        "//common/thread:counter",
        "//common/thread:threadpool",
        "//printer/base:mesh",
        "//printer/base:octree"
    ]
} },

{ "cc_library": {
    "name": "test_helper",
    "cc_sources": [ "test_helper.cc" ],
    "cc_headers": [ "test_helper.h" ],
    "dependencies": [
        "//common/log:log",
        "//printer/base:mesh",
        "//printer/stl:format"
    ]
} },

{ "cc_test": {
    "name": "sweep_triangulation_test",
    "cc_sources": [ "sweep_triangulation_test.cc" ],
    "dependencies": [
        ":triangulation",
        "//common/log:log",
        "//common/test:test_main"
    ]
} },

{ "cc_test": {
    "name": "triangulation_test",
    "cc_sources": [ "triangulation_test.cc" ],
    "dependencies": [
        ":triangulation",
        "//common/log:log",
        "//common/test:test_main"
    ]
} },

{ "cc_test": {
    "name": "simplifier_test",
    "cc_sources": [ "simplifier_test.cc" ],
    "dependencies": [
        ":simplifier",
        ":test_helper",
        "//common/log:log",
        "//common/test:test_main",
        "//common/base:base_tcmalloc"
    ]
} },

{ "cc_test": {
    "name": "face_reduction_test",
    "cc_sources": [ "face_reduction_test.cc" ],
    "dependencies": [
        ":face_reduction",
        ":test_helper",
        "//common/strings:strutil",
        "//common/test:test_main",
        "//common/base:base_tcmalloc"
    ]
} }
]