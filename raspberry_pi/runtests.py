import unittest

loader = unittest.TestLoader()
suite = loader.discover("tests", pattern="*test.py")

runner = unittest.TextTestRunner()
runner.run(suite)
