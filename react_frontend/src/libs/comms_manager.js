import * as log from 'loglevel';

const CommsManager = (address) => {
  // Messages and events
  const messages = {
    START: 'start',
    STOP: 'stop',
    PAUSE: 'pause',
    RESET: 'reset'
  };

  const events = {
    UPDATE_GUI: 'update-gui',
  }

  // Observer pattern methods
  const observers = {}

  const subscribe = (callback, event) => {
    observers[event] = observers[event] || [];
    observers[event].push(callback);
  }

  const unsuscribe = (callback, event) => {
    observers[event] = observers[event] || [];
    observers[event].splice(observers[event].indexOf(callback))
  }

  const unsuscribeAll = () => {
    for(const event in observers) {
      observers[event].length = 0;
    }
  }

  // Websocket methods
  let websocket = new WebSocket(address);

  websocket.onopen = () => {
    log.debug(`Connection with ${address} opened`);
    websocket.send("HELLO");
  };

  websocket.onclose = (e) => {
    // TODO: Rethink what to do when connection is interrupted,
    //  maybe try to reconnect and not clear the suscribers?
    unsuscribeAll();
    if(e.wasClean) {
      log.debug(`Connection with ${address} closed, all suscribers cleared`);
    } else {
      log.debug(`Connection with ${address} interrupted`);
    }
  };

  websocket.onmessage = (e) => {
    const message = JSON.parse(e.data);
    dispatch(message.name, message.data);
  };

  // Send and receive method
  const send = (message, data) => {
    const msg = JSON.stringify({ 'message': messages[message],
      'data': data });
    websocket.send(msg);
  }

  const dispatch = (message, data) => {
    const subscriptions = observers[message] || [];
    for(let i=0, length=subscriptions.length; i < length; ++i) {
      subscriptions[i](data);
    }
  }

  return {
    messages: messages,
    send: send,

    events: events,
    subscribe: subscribe,
    unsuscribe: unsuscribe,
  }
}

export default CommsManager;