from __future__ import annotations

import json
from functools import singledispatchmethod
from typing import Any

from pydantic import BaseModel


class ManagerConsumerMessage(BaseModel):
    """
    ManagerConsumer message utility class
    @param id: unique identifier of message
    @param command: message command
    @param data: message data (optional)
    """
    id: str
    command: str
    data: Any

    def response(self, response: Any = None) -> ManagerConsumerMessage:
        """
        Returns ack response for this message
        @param response: response data
        @return: the response message as a ManagerConsumerMessage
        """
        return ManagerConsumerMessage(id=self.id, command='ack', message=response)

    def __repr__(self):
        return self.json()

    def __str__(self):
        return self.json()


class ManagerConsumerMessageException(BaseException):
    def __init__(self, id: str, message: str = None):
        super(ManagerConsumerMessageException, self).__init__(message)
        self.id = id
        self.command = 'error'
        self.message = message

    def consumer_message(self):
        return ManagerConsumerMessage(id=self.id, command=self.command, data={'message': self.message})

    def __str__(self):
        return str(ManagerConsumerMessage(id=self.id, command=self.command, data={'message': self.message}))
