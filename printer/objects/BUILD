[
{ "cc_library": {
    "name": "object",
    "cc_headers": [ "object.h" ],
    "dependencies": [ "//printer/base:geometry" ]
} },
{ "cc_library": {
    "name": "transform",
    "cc_headers": [ "transform.h" ],
    "cc_sources": [ "transform.cc" ],
    "dependencies": [ ":object", "//common/log:log" ]
} },
{ "cc_library": {
    "name": "primitives",
    "cc_headers": [ "primitives.h" ],
    "cc_sources": [ "primitives.cc" ],
    "dependencies": [ "//common/log:log", ":object" ]
} },
{ "cc_library": {
    "name": "point_octree_object",
    "cc_headers": [ "point_octree_object.h" ],
    "cc_sources": [ "point_octree_object.cc" ],
    "dependencies": [ ":object",
                      "//common/log:log",
                      "//printer/base:point_octree"
     ]
} },
{ "cc_library": {
    "name": "all_objects",
    "dependencies": [ ":object",
                      ":primitives",
                      ":transform"
    ]
} }
]