add_rules("mode.debug", "mode.release")

target("PathTracing")
    set_kind("binary")
    add_includedirs("include")
    add_files("src/*.cpp")
    set_languages("c++17")