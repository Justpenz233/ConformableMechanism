includes("build_proj.lua")
target("spdlog")
_config_project({
    project_kind = "static",
    batch_size = 64,
    no_rtti = true
})
add_includedirs("include", {public = true})
on_load(function(target)
    local function rela(p)
        return path.relative(path.absolute(p, os.scriptdir()), os.projectdir())
    end
    if get_config("spdlog_only_fmt") then
        target:add("defines", "FMT_CONSTEVAL=constexpr", "FMT_USE_CONSTEXPR=1", "FMT_EXCEPTIONS=0", {
            public = true
        })
        target:add("headerfiles", rela("include/spdlog/fmt/**.h"))
        target:add("files", rela("src/bundled_fmtlib_format.cpp"))
    else
        target:add("defines", "SPDLOG_NO_EXCEPTIONS", "SPDLOG_NO_THREAD_ID", "SPDLOG_DISABLE_DEFAULT_LOGGER",
            "FMT_CONSTEVAL=constexpr", "FMT_USE_CONSTEXPR=1", "FMT_EXCEPTIONS=0", {
                public = true
            })
        target:add("headerfiles", rela("include/**.h"))
        target:add("files", rela("src/*.cpp"))
    end
    target:add("defines", "SPDLOG_COMPILED_LIB", {
        public = true
    })
end)
target_end()
