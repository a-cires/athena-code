import can

class CANInterface:
    def __init__(self, callback=None, channel='can1', bustype='socketcan', bitrate=1000000):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)
        self.callback = callback
        self.notifier = None

        if self.callback:
            self.start_listening()

    def send(self, arbitration_id, data):
        arbitration_id = int(arbitration_id & 0x7FF)
        # print(f"Received message. ID: {arbitration_id}, Data: {data}")
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            # print("Message sent!")
        except can.CanError:
            print("Message NOT sent!")

    def receive(self, timeout=0.005):
        return self.bus.recv(timeout)
    
    def start_listening(self):
        if not self.callback:
            return
        
        self.notifier = can.Notifier(self.bus, [self.callback])

    def stop_listening(self):
        if self.notifier:
            self.notifier.stop()
            self.notifier = None
