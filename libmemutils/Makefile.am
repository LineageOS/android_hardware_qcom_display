c_sources = membuf_wrapper.cpp

lib_LTLIBRARIES = libmemutils.la
libmemutils_la_CC = @CC@
libmemutils_la_SOURCES = $(c_sources)
libmemutils_la_CFLAGS = $(COMMON_CFLAGS)
libmemutils_la_CPPFLAGS = $(AM_CPPFLAGS) -DLOG_TAG=\"MEM_UTILS\"
libmemutils_la_LDFLAGS = -shared -avoid-version
