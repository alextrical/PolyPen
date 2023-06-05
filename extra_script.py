Import("env")

env.Replace(PROGNAME="%s_firmware_%s" % (env.GetProjectOption("board"),env.GetProjectOption("custom_prog_version")))