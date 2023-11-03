import os
import sys

print('about to import', file=sys.stderr)
print('python is', sys.version_info)
print('pid is', os.getpid())

from bindings import adder

print('imported, about to call', file=sys.stderr)

result = adder.add(2, 3)
assert result == 5

print('done!', file=sys.stderr)
