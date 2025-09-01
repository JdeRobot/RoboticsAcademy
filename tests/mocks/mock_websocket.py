class MockWebSocketApp:
    def __init__(self, url, on_message=None, on_error=None, on_close=None, on_open=None):
        self.url = url
        self.on_message = on_message
        self.on_error = on_error
        self.on_close = on_close
        self.on_open = on_open
        self.sent_messages = []
        self.is_running = False
    
    def run_forever(self, ping_timeout=None, ping_interval=0):
        self.is_running = True
        # In a real test, you would control when this returns
    
    def send(self, message):
        self.sent_messages.append(message)
    
    def close(self):
        self.is_running = False
        if self.on_close:
            self.on_close(self)
    
    def simulate_message(self, message):
        """Simulate receiving a message"""
        if self.on_message:
            self.on_message(self, message)
    
    def simulate_error(self, error):
        """Simulate an error"""
        if self.on_error:
            self.on_error(self, error)
