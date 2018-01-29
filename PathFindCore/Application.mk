APP_ABI :=armeabi-v7a x86
APP_PLATFORM:=android-14
APP_OPTIM := release
# APP_STL := stlport_shared
# APP_STL := stlport_static
# APP_STL := gnustl_shared
APP_STL := gnustl_static
APP_CPPFLAGS += -std=c++11
APP_CPPFLAGS += -fexceptions
ANDROID := 1