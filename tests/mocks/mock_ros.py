class MockROSNode:
    def __init__(self, node_name):
        self.node_name = node_name
        self.publishers = {}
        self.subscribers = {}

    def create_publisher(self, msg_type, topic, qos):
        publisher = MockROSPublisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos):
        subscriber = MockROSSubscriber(topic, callback)
        self.subscribers[topic] = subscriber
        return subscriber


class MockROSPublisher:
    def __init__(self, topic):
        self.topic = topic
        self.published_messages = []

    def publish(self, msg):
        self.published_messages.append(msg)


class MockROSSubscriber:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback

    def simulate_message(self, msg):
        """Simulate receiving a message"""
        self.callback(msg)
