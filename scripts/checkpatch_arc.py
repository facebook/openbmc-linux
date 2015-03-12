#!/usr/bin/python

# Linux kernel checkpatch.pl intergration with arcanist's linter system.

import os
import sys
import json
import subprocess

if(len(sys.argv) != 2):
    print("USAGE: "+sys.argv[0]+" PATH")
    sys.exit(1)
filepath = sys.argv[1]

# Within .arcconfig in the git repository, you will see a line
# "git.default-relative-commit" : "origin/<BRANCHNAME>",
# this configuration option will be used as the comparison branch.

with open('.arcconfig', 'r') as f:
    filedata = f.read()
jobj = json.loads(filedata)
comparison_branch = str(jobj['git.default-relative-commit'])


if os.path.isfile(filepath) and os.access(filepath, os.R_OK):

    # Create a diff of the current branch against the comparison branch
    gitcmd = ["git", "diff", comparison_branch, filepath]
    gproc = subprocess.Popen(gitcmd, stdout=subprocess.PIPE)

    # Give the diff to checkpatch using terse mode so that
    # arcanist can run a regex on the output
    checkcmd = ["scripts/checkpatch.pl", "--terse", "--no-summary",
                "--show-types", "--no-signoff", "-"]
    cproc = subprocess.Popen(checkcmd, stdin=gproc.stdout)
    cproc.wait()

sys.exit(0)
