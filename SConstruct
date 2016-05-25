env = Environment()

sources_threaded = [
	"pisoc.cpp",
	"face_tracking.cpp"
]

sources_simple = [
	"pisoc.cpp",
	"face_tracking_simple.cpp"
]

test_sources = [
	"pisoc.cpp",
	"test.cpp"
]
libpaths = [
	"/opt/vc/lib",
]

cpp_path = [
	"/usr/local/include/raspicam"
]

libs = [
	"pthread",
	"raspicam_cv",
	"raspicam",
	"mmal",
	"mmal_core",
	"mmal_util",
]

env.Append(CPPPATH = cpp_path, LIBPATH = libpaths, CXXFLAGS = ["-std=c++11"], LIBS = libs)
env.ParseConfig("pkg-config --libs opencv")
env.Program("facetracker", sources_threaded)
env.Program("simpletracker", sources_simple)
env.Program("test", test_sources);
