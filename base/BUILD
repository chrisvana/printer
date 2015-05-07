[
{ "cc_library": {
    "name": "geometry",
    "cc_headers": [ "geometry.h" ],
    "cc_sources": [ "geometry.cc" ],
    "dependencies": [ "//common/base:flags",
                      "//common/log:log",
                      "//common/strings:strutil"
    ]
} },
{ "cc_library": {
    "name": "aabb_tree",
    "cc_headers": [ "aabb_tree.h" ],
    "cc_sources": [ "aabb_tree.cc" ],
    "dependencies": [ ":geometry", "//common/log:log" ]
} },
{ "cc_library": {
    "name": "octree",
    "cc_headers": [ "octree.h" ],
    "cc_sources": [ "octree.cc" ],
    "dependencies": [ ":geometry", ":aabb_tree", "//common/log:log" ]
} },
{ "cc_library": {
    "name": "point_octree",
    "cc_headers": [ "point_octree.h" ],
    "cc_sources": [ "point_octree.cc" ],
    "dependencies": [ ":geometry", "//common/log:log" ]
} },
{ "cc_library": {
    "name": "mesh",
    "cc_headers": [ "mesh.h" ],
    "cc_sources": [ "mesh.cc" ],
    "dependencies": [ ":geometry", ":octree", "//common/log:log" ]
} },
{ "cc_test": {
    "name": "geometry_test",
    "cc_sources": [ "geometry_test.cc" ],
    "dependencies": [ ":geometry",
                      "//common/log:log",
                      "//common/test:test_main"
     ]
} },
{ "cc_test": {
    "name": "octree_test",
    "cc_sources": [ "octree_test.cc" ],
    "dependencies": [ ":octree",
                      "//common/test:test_main"
    ]
} },
{ "cc_test": {
    "name": "aabb_tree_test",
    "cc_sources": [ "aabb_tree_test.cc" ],
    "dependencies": [ ":aabb_tree",
                      "//common/test:test_main"
    ]
} },
{ "cc_test": {
    "name": "mesh_test",
    "cc_sources": [ "mesh_test.cc" ],
    "dependencies": [ ":mesh",
                      "//common/base:flags",
                      "//common/base:base_tcmalloc",
                      "//common/log:log",
                      "//common/test:test_main",
                      "//printer/stl:format"
    ]
} }
]