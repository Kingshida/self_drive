import os
import sys

print('os.getcwd():', os.getcwd())
print('dirname(sys.path[0]):',os.path.dirname(sys.path[0]))
print('dirname(abspath(sys.argv[0])):',os.path.dirname(os.path.abspath(sys.argv[0])))
print('dirname(realpath(__file__)):',os.path.dirname(os.path.realpath(__file__)))
