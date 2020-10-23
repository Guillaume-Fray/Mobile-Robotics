import threading
import re
import string
import time

class _Getch:
    """Gets a single character from standard input.  Does not echo to the screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


def WaitForChar(gotchar,valid_chars=None,char=None):
    reFlags = 0
    if valid_chars is None:
       valid_chars="."
       reFlags=re.DOTALL
    charmatch=re.compile(valid_chars,reFlags)
    getChar = _Getch()
    while not gotchar.is_set():
          char = getChar()
          if charmatch.match(char) is not None:
             gotchar.set()
          time.sleep(0.4)
    return
