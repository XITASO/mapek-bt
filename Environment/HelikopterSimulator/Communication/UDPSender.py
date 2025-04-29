import socket
from threading import Thread
import time


class UDPSender:
    def __init__(self, message, address, port):
        self.message = message
        self.address = address
        self.port = port

        self.shouldSend = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.thread = None

    def close(self):
        self.sock.close()

    def send_once(self):
        self.sock.sendto(self.message, (self.address, self.port))

    def init_sending_with_frequency(self, frequency=30):
        self.shouldSend = True

        self.thread = Thread(target=self.send_with_frequency, args=(frequency, ))
        self.thread.start()

    def stop_sending_with_frequency(self):
        if (self.thread):
            self.shouldSend = False
            self.thread.join()

    def send_with_frequency(self, frequency=30):
        desired_sleep_time = 1 / frequency
        
        while self.shouldSend:
            start_time = time.time()
            
            self.send_once()

            current_time = time.time()
            actual_sleep_time = desired_sleep_time - (current_time - start_time)
            if actual_sleep_time > 0:
                time.sleep(actual_sleep_time)
