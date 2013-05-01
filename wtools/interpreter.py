import sys, termios, select, serial, threading, signal, traceback, os

class Console:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        try:
            self.old = termios.tcgetattr(self.fd)
            tc = termios.tcgetattr(self.fd)
            tc[3] = tc[3] & ~termios.ICANON & ~termios.ECHO & ~termios.ISIG
            tc[6][termios.VMIN] = 1
            tc[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, tc)
        except termios.error:
            # ignore errors, so we can pipe stuff to this script
            pass
    def cleanup(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old)
        except:
            # ignore errors, so we can pipe stuff to this script
            pass
    def getkey(self):
        # Return -1 if we don't get input in 0.1 seconds, so that
        # the main code can check the "alive" flag and respond to SIGINT.
        [r, w, x] = select.select([self.fd], [], [self.fd], 0.1)
        if r:
            return os.read(self.fd, 1)
        elif x:
            return ''
        else:
            return -1

class Term:
    def __init__(self, ser):
        self.ser = ser
        self.threads = []

    def start(self):
        self.alive = True

        # serial -> console
        self.threads.append(threading.Thread(
                target = self.reader,
                ))

        # console -> serial
        self.console = Console()
        self.threads.append(threading.Thread(
                target = self.writer,
                ))

        # start all threads
        for thread in self.threads:
            thread.daemon = True
            thread.start()

    def stop(self):
        self.alive = False

    def join(self):
        for thread in self.threads:
            while thread.isAlive():
                thread.join(0.1)


    def reader(self):
        first = True
        try:
            while self.alive:
                data = self.ser.read(1)
                if not data:
                    continue

                # Don't print a NULL if it's the first character
                # we read. This hide startup/port-opening glitches
                # with some serial devices.
                if first and data == '\0':
                    first = False
                    continue
                first = False

                if ((ord(data) >= 32 and ord(data) < 128) or
                    data == '\r' or data == '\n' or data == '\t'):
                    sys.stdout.write(data)
                else:
                    sys.stdout.write('\\x'+("0"+hex(ord(data))[2:])[-2:])

                sys.stdout.flush()

        except Exception as e:
            sys.stdout.flush()
            traceback.print_exc()
            self.console.cleanup()
            os._exit(1)


    def writer(self):
        try:
            while self.alive:
                try:
                    c = self.console.getkey()
                except KeyboardInterrupt:
                    c = '\x03'
                if c == '\x03':
                    self.stop()
                    return

                elif c == -1:
                    # No input, try again
                    continue

                elif c == '':
                    # EOF on input. Wait a tiny bit
                    # so we can flush the remaining input,
                    # then stop.
                    time.sleep(0.25)
                    self.stop()
                    return

                else:
                    # send character
                    self.ser.write(c)

        except Exception as e:
            sys.stdout.flush()
            traceback.print_exc()
            self.console.cleanup()
            os._exit(1)

    def print_header(self):
        print "Welcome to Woggle Terminal"
        print "^C to exit"
        print "----------"

    def run(self):
        # Set timeout
        self.ser.timeout = 0.1

        # Handle SIGINT gracefully
        signal.signal(signal.SIGINT, lambda *args: self.stop())

        # Header
        self.print_header()

        # Go
        self.start()
        self.join()

        # Cleanup
        self.console.cleanup()
