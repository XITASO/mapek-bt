import socket


class UDPReceiver:
    def __init__(self, address, port):
        self.address = address
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.address, self.port))
        self.sock.settimeout(2000)
        self.setBlocking(False)

    def __del__(self):
        self.close()

    def close(self):
        self.sock.close()

    def setBlocking(self, blocking = True):
        self.sock.setblocking(blocking)

    def receive_once(self, expected_size=8012):
        try:
            message = self.sock.recv(expected_size)
            return message
        except:
            return None
            
