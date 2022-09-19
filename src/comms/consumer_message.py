from __future__ import annotations

import json
from typing import Any


class ManagerConsumerMessage:
    def __init__(self, id: str, command: str, data: Any = None):
        """
        ManagerConsumer message utility class
        @param id: unique identifier of message
        @param command: message command
        @param data: message data (optional)
        """
        self.id = id
        self.command = command
        self.data = data

    def __str__(self):
        return json.dumps({
            'id': self.id,
            'command': self.command,
            'data': self.data
        })

    def response(self, response: Any = None) -> ManagerConsumerMessage:
        """
        Returns ack response for this message
        @param response: response data
        @return: the response message as a ManagerConsumerMessage
        """
        return ManagerConsumerMessage(self.id, 'ack', response)

    @staticmethod
    def from_dict(d: dict) -> ManagerConsumerMessage:
        """
        Creates a new ManagerConsumerMessage from a dictionary representation
        @param d: the dictionary containing message parámeters
        @return: the ManagerConsumerMessage object
        """
        defaults = {'data': None}
        defaults.update(d)
        return ManagerConsumerMessage(**defaults)

    @staticmethod
    def from_str(string) -> ManagerConsumerMessage:
        """
        Creates a new ManagerConsumerMessage from a json string
        @param string: parameters as json encoded string
        @return: the ManagerConsumerMessage object
        """
        return ManagerConsumerMessage.from_dict(json.loads(string))


class ManagerConsumerMessageException(BaseException):
    def __init__(self, origin: ManagerConsumerMessage, message: str = None):
        super(ManagerConsumerMessageException, self).__init__(message)
        self.id = origin.id
        self.command = 'error'
        self.message = message

    def __str__(self):
        return str(ManagerConsumerMessage(self.id, self.command, {'message': self.message}))
