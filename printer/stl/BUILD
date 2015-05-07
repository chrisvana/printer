[
{ "cc_library": {
    "name": "format",
    "cc_headers": [ "format.h" ],
    "cc_sources": [ "format.cc" ],
    "dependencies": [
        "//common/base:macros",
        "//common/base:types",
        "//common/log:log",
        "//common/strings:strutil",
        "//printer/base:mesh"
    ]
} },

{ "cc_library": {
    "name": "driver",
    "cc_sources": [ "driver.cc" ],
    "cc_headers": [ "driver.h" ],
    "dependencies": [
        ":format",
        "//common/log:log",
        "//common/util:stl",
        "//printer/base:mesh",
        "//printer/objects:object",
        "//printer/execute:driver"
    ]
} }
]