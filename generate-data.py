"""
Generate data from example code and store in files for analysis.

* Select all local files and filter to chapter code
* Run each file sequentially
* Store the output in a file named after the script that generated it with a 'data' suffix
"""

import os
import fnmatch

SUFFIX = '.data'

scripts = [file for file in os.listdir() if fnmatch.fnmatch(file, 'ch*.py')]

for script in scripts:
    print('Running: ' + script)
    os.system('python {} > {}'.format(script, script + SUFFIX))
