includes("build_proj.lua")
target("mimalloc")
_config_project({
	project_kind = "object",
	no_rtti = true
})
on_load(function(target)
	local function rela(p)
		return path.relative(path.absolute(p, os.scriptdir()), os.projectdir())
	end
	target:add("includedirs", rela("include"), {
		public = true
	})
	target:add("defines", "MI_SHARED_LIB", {public = true})
	target:add("defines", "MI_XMALLOC=1", "MI_WIN_NOREDIRECT", "MI_SHARED_LIB_EXPORT")
	if is_plat("windows") then
		target:add("syslinks","advapi32", "bcrypt", {public = true})
		target:add("defines", "_CRT_SECURE_NO_WARNINGS")
	elseif is_plat("linux") then
		target:add("syslinks","pthread", "atomic", {public = true})
	else
		target:add("syslinks","pthread", {public = true})
	end
end)
add_headerfiles("include/*.h")
add_files("src/static.c")
target_end()
