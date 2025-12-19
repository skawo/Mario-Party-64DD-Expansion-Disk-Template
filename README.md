# Mario-Party-64DD-Expansion-Disk-Template
A C template for making Mario Party 1 64DD Expansion Disks.

Mario Party doesn't do anything except for rebooting the game into new code - this template only serves to provide the barest example of the functionality being used and displays a full-screen image.

# Building
Requires Python 3 and make.

1. Extract the toolchain archives found in tool/gcc.
2. Run build.py found in the repository root.

Options may be passed to build.sh:
   * "USA", "USA-DEV", "JPN", "JPN-DEV" - Build target
   * "-clean" - Runs make clean before building the disk
   * "-skipfs" - Foregoes building the filesystem (to speed up compilation once a lot of files have been added).
   * "-fs" - Only generates the filesystem (for when a new file is added, to just generate the necessary symbols).
