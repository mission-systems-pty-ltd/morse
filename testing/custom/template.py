#! /usr/bin/env python

"""
 Name:            Morse Test Template
 Description:     Basic template for custom morse test
 Copyright (C):   2023 Mission Systems Pty Ltd

"""

# Libraries
import warnings
import morse.testing.testing as testing
from morse.builder import *
from pymorse import Morse

# The MorseTestCase Template
class MorseTestCase(testing.MorseTestCase):
    
    # Set up the environment (abstract)
    def setUpEnv(self):
        robot = ATRV()
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    # Creates a sample test
    def test_functions(self):
        
        # Test boolean
        self.assertTrue(1+1==2)
        self.assertFalse(1+1==3)
        
        # Test equality
        self.assertEqual(1+1, 2)
        self.assertNotEqual(1+1, 3)
        self.assertCountEqual([1,2,"c"], [2,"c",1])
        
        # Test inequality
        self.assertLess(1, 2)
        self.assertLessEqual(2, 2)
        self.assertGreater(2, 1)
        self.assertGreaterEqual(2, 2)
        
        # Test quasi-equality
        self.assertAlmostEqual(1+1, 2.01, places=1)
        self.assertAlmostEqual(1+1, 2.01, delta=0.01)
        self.assertNotAlmostEqual(1+1, 2.01, places=2)
        self.assertNotAlmostEqual(1+1, 2.01, delta=0.009)
        
        # Test multi-equality
        self.assertSequenceEqual([1,2,3], [1,2,1+2], list)
        self.assertListEqual([1,2,3], [1,2,1+2])
        self.assertTupleEqual((1,2,3), (1,2,1+2))
        self.assertSetEqual({1,2,3}, {1,2,1+2})
        self.assertDictEqual({"a": 1, "b": 2}, {"a": 1, "b": 1+1})
        self.assertMultiLineEqual("a\nbc", "a\nbc")
        
        # Test containment
        self.assertIn(1, [1,2,3])
        self.assertNotIn("a", [1,2,3])
        self.assertDictContainsSubset({"a": 1}, {"a":1, "b": 1+1})
        
        # Test is
        list_1 = [1,2,3]
        list_2 = list_1
        self.assertIs(list_1, list_2)
        self.assertIsNot(list_1, [1,2,3])
        self.assertIsNone(None)
        self.assertIsNotNone("abc")
        self.assertIsInstance("abc", str)
        self.assertNotIsInstance("abc", int)
        
        # Test
        self.assertRegex("abc", r'a.*')
        self.assertNotRegex("abc", r'd.*')
        
        # Test error and warnings
        with self.assertRaises(ArithmeticError):
            raise ArithmeticError
        with self.assertRaisesRegex(ArithmeticError, "abc"):
            raise ArithmeticError("abc")
        with self.assertWarnsRegex(FutureWarning, "def"):
            warnings.warn("def", FutureWarning)        

# Calls the tests
if __name__ == "__main__":
    testing.main(MorseTestCase)
