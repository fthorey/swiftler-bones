import sys, termios, select, serial, threading, signal, traceback
import contextlib, os, codecs, tty

class TimeoutException(Exception):
    def __init__(self,what):
        self.what = what
    def __str__(self):
        return self.what

def timeoutmanager(f):
    def f2(*args):
        def timeout_handler(signum, frame):
            raise TimeoutException('Timeout')

        old_handler = signal.signal(signal.SIGALRM, timeout_handler)
        signal.alarm(1)

        try:
            f(*args)

        except TimeoutException as e:
            raise

        finally:
            signal.signal(signal.SIGALRM, old_handler)
            signal.alarm(0)

        return
    return f2

@timeoutmanager
def checkPrompt(*args):
    prompt = []
    while True:
        data = args[0].ser.read(1)

        if not data:
            continue

        if ((ord(data) >= 32 and ord(data) < 128) or
            data == '\r' or data == '\n' or data == '\t'):
            prompt += data

        if args[1] in ''.join(prompt):
            return

        sys.stdout.flush()

@contextlib.contextmanager
def console():
    old_attrs = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    try:
        yield
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)

class Term:
    def __init__(self, ser, termName):
        self.ser = ser
        self.termName = termName
        self.threads = []

    def getkey(self):
        # Return -1 if we don't get input in 0.1 seconds, so that
        # the main code can check the "alive" flag and respond to SIGINT.
        [r, w, x] = select.select([sys.stdin], [], [sys.stdin], 0.1)
        reader = codecs.getreader(sys.stdin.encoding)(sys.stdin)
        if r:
            return reader.read(1)
        elif x:
            return ''
        else:
            return -1

    def start(self):
        self.alive = True

        # serial -> console
        self.threads.append(threading.Thread(
                target = self.reader,
                ))

        # console -> serial
        self.threads.append(threading.Thread(
                target = self.writer,
                ))

        # start all threads
        for thread in self.threads:
            thread.start()

    def stop(self):
        self.alive = False

    def join(self):
        for thread in self.threads:
            while thread.isAlive():
                thread.join(0.1)

    def reader(self):
        try:
            while self.alive:
                data = self.ser.read(1)
                if not data:
                    continue

                if ((ord(data) >= 32 and ord(data) < 128) or
                    data == '\r' or data == '\n' or data == '\t'):

                    if ord(data) == 0x7f or ord(data) == 0x8:
                        data = '\b'
                    sys.stdout.write(data)

                else:
                    sys.stdout.write('\\x'+("0"+hex(ord(data))[2:])[-2:])

                sys.stdout.flush()

        except Exception as e:
            sys.stdout.flush()
            traceback.print_exc()
            os._exit(1)

    def writer(self):
        try:
            while self.alive:
                try:
                    c = self.getkey()
                except KeyboardInterrupt:
                    c = '\x03'

                if c == '\x03':
                    self.stop()
                    return

                elif c == -1:
                    # No input, try again
                    continue

                else:
                    # send character
                    self.ser.write(c)

        except Exception as e:
            sys.stdout.flush()
            traceback.print_exc()
            os._exit(1)

    def run(self):
        # Set timeout
        self.ser.timeout = 0.1

        # Handle SIGINT gracefully
        signal.signal(signal.SIGINT, lambda *args: self.stop())

        # Go
        self.start()
        self.join()
