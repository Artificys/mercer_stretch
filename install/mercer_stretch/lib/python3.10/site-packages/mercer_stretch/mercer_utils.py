import sys
import tty
import termios
import threading

class Keyboard:
    def __init__(self):
        self.pressed_keys = set()
        self.running = False
        self.lock = threading.Lock()

    def _key_capture_thread(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                ch = sys.stdin.read(1)
                with self.lock:
                    self.pressed_keys.add(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def start(self):
        self.running = True
        threading.Thread(target=self._key_capture_thread, daemon=True).start()

    def stop(self):
        self.running = False

    def get_pressed_keys(self):
        with self.lock:
            keys = list(self.pressed_keys)
            self.pressed_keys.clear() 
        return keys
    
    def is_pressed(self, key):
        with self.lock:
            return key in self.pressed_keys
