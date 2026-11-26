bin_PROGRAMS = roboSoccer
roboSoccer_SOURCES = roboSoccer.c imagecapture/imageCapture.c imagecapture/avilib.c imagecapture/color.c imagecapture/gui.c imagecapture/imageProc.c imagecapture/svdDynamic.c imagecapture/utils.c imagecapture/v4l2uvc.c \
			API/btcomm.c roboAI.c \
			logic/geometry.c logic/skills.c logic/drive.c logic/fsm.c logic/perception.c logic/mode_penalty.c logic/mode_penalty_adapter.c logic/gyro.c \
			logic/mode_chase.c logic/mode_chase_adapter.c \
			logic/mode_soccer.c logic/mode_soccer_adapter.c
# Run log / flight recorder (see logic/runlog.h). Kept on its own line so it
# merges cleanly with changes to the list above.
roboSoccer_SOURCES += logic/runlog.c
CC=g++
AM_CPPFLAGS=-fpermissive
# OpenMP: the vision code (fieldUnwarp2, bgSubtract3, display copy, imageProc)
# is full of "#pragma omp parallel for". Without -fopenmp those are
# IGNORED!!!!! and the whole pipeline runs on A SINGULAR thread (Insert Barfing
# emoji cuz EWWWWWWWWWW). AM_CFLAGS is used for both
# compiling and linking here, so this also ropes in libgomp. The same flag is
# in compile_standalone.sh.
AM_CFLAGS=-fopenmp
