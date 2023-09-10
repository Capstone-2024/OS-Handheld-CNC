import shlex, subprocess

import shlex, subprocess
command_line = input()
args = shlex.split(command_line)

print(args)