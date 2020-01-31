# small hack to pass "-g" flag to the LTO linker

Import("env")
env.Append(LINKFLAGS=["-g"])
