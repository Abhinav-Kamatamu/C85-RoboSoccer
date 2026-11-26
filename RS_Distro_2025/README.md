UTSC - RoboSoccer

Will compile and run on Linux provided the OpenGL and Bluetooth libraries are
installed.

From the project's folder type:

>./configure

If any dependencies are missing, install them and re-run configure.

Once the configuration ends successfully, type

>make

The executable file will be in ./src so you can run it with

./src/roboSoccer /dev/video1 0 0

See the handout and the executable's usage doc. for the meaning of each
command line parmeter.

## Run logging (flight recorder)

Pass a logItems bitmask as the 6th argument to record a run, e.g. everything:

>./src/roboSoccer /dev/video1 1 1 colours.dat 7

bit0 (1) = text logs, bit1 (2) = display frames (5 fps snapshots + 3 s of
every frame around each pre-empt/strike/goal/bookmark), bit2 (4) = raw camera
and unwarped field frames. Each run gets its own folder in `.local/logs/`
(oldest runs are deleted past 5 GB unless the folder holds a `KEEP` file).
Press `b` in the window to bookmark a moment; `q` or Ctrl+C saves and quits.

Then:

>python3 tools/runlog/analyze_run.py report latest

writes a summary, per-event reports and contact sheets into the run's
`report/` folder. `tools/runlog/README_FOR_AI.md` explains every field, and is
what an AI assistant should read before analysing a run.
