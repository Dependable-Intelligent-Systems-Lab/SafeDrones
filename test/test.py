import unittest
import os
import sys

try:
    import safedrone
    print("Imported packaged SafeDrone module.")
except ImportError as e:
    print("SafeDrone module is imported from the raw source code instead of using the installed package.")
    sys.path.insert(1, os.path.join(sys.path[0], '..'))

#########################################################
# Import all UnitTest Classes comprising the test cases #
#########################################################

from core_package_tests import TestSafeDroneMethods

if __name__ == '__main__':
    unittest.main()