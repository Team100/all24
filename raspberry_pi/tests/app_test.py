""" a test """
import unittest
from app import example1
from app2 import example3

class AppTest(unittest.TestCase):
    """ a test example"""
    def test_it(self):
        """ run a test """
        print("do_test")
        example1.do_example1()
        example3.do_example3()
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
