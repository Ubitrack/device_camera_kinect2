from conans import ConanFile, CMake, tools


class UbitrackCoreConan(ConanFile):
    name = "ubitrack_device_camera_kinect2"
    version = "1.3.0"

    description = "Ubitrack Device Camera Kinect2"
    url = "https://github.com/Ubitrack/device_camera_kinect2.git"
    license = "GPL"

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"

    options = { 
        "workspaceBuild" : [True, False],
    }

    default_options = {
        "ubitrack_core:shared" : True,
        "ubitrack_vision:shared" : True,
        "ubitrack_dataflow:shared" : True,
        "workspaceBuild" : False,
        }

    # all sources are deployed with the package
    exports_sources = "cmake/*", "doc/*", "src/*", "CMakeLists.txt"

    def system_requirements(self):
        if os_info.is_linux:
            if os_info.with_apt:
                installer = SystemPackageTool()
                if self.settings.arch == "x86" and tools.detected_architecture() == "x86_64":
                    arch_suffix = ':i386'
                    installer.install("g++-multilib")
                    installer.install("gcc-multilib")
                else:
                    arch_suffix = ''
                installer.install("pkg-config")
                installer.install("ninja-build")
                installer.install("doxygen")
                installer.install("clang")
                installer.install("python3")
                installer.install("git-lfs")
                installer.install("nasm")
                installer.install("%s%s" % ("mono-devel"))
    
                installer.install("%s%s" % ("libusb-1.0-0-dev", arch_suffix))
                installer.install("%s%s" % ("libgl1-mesa-dev", arch_suffix))
                installer.install("%s%s" % ("libsoundio-dev", arch_suffix))
                installer.install("%s%s" % ("libvulkan-dev", arch_suffix))
                installer.install("%s%s" % ("libx11-dev", arch_suffix))
                installer.install("%s%s" % ("libxcursor-dev", arch_suffix))
                installer.install("%s%s" % ("libxinerama-dev", arch_suffix))
                installer.install("%s%s" % ("libxrandr-dev", arch_suffix))
                installer.install("%s%s" % ("libusb-1.0-0-dev", arch_suffix))
                installer.install("%s%s" % ("libssl-dev", arch_suffix))
                installer.install("%s%s" % ("libudev-dev", arch_suffix))
                installer.install("%s%s" % ("mesa-common-dev", arch_suffix))
                installer.install("%s%s" % ("uuid-dev", arch_suffix))
            else:
                self.output.warn("Could not determine package manager, skipping Linux system requirements installation.")


    def requirements(self):
        userChannel = "ubitrack/stable"
        if self.options.workspaceBuild:
            userChannel = "local/dev"

        self.requires("ubitrack_core/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_dataflow/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_vision/%s@%s" % (self.version, userChannel))

    # def imports(self):
    #     self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
       
    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        pass

    def package_info(self):
        pass
