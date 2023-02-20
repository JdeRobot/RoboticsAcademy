import * as log from "loglevel";
import { v4 as uuidv4 } from "uuid";

const CommsManager = (address) => {
  let websocket = null;

  log.enableAll();

  const events = {
    RESPONSES: ["ack", "error"],
    UPDATE: "update",
    STATE_CHANGED: "state-changed",
  };

  //region Observer pattern methods
  const observers = {};

  const subscribe = (events, callback) => {
    if (typeof events === "string") {
      events = [events];
    }
    for (let i = 0, length = events.length; i < length; i++) {
      observers[events[i]] = observers[events[i]] || [];
      observers[events[i]].push(callback);
    }
  };

  const unsubscribe = (events, callback) => {
    if (typeof events === "string") {
      events = [events];
    }
    for (let i = 0, length = events.length; i < length; i++) {
      observers[events[i]] = observers[events[i]] || [];
      observers[events[i]].splice(observers[events[i]].indexOf(callback));
    }
  };

  const unsuscribeAll = () => {
    for (const event in observers) {
      observers[event].length = 0;
    }
  };

  const subscribeOnce = (event, callback) => {
    subscribe(event, (response) => {
      callback(response);
      unsubscribe(event, callback);
    });
  };

  const dispatch = (message) => {
    const subscriptions = observers[message.command] || [];
    let length = subscriptions.length;
    while (length--) {
      subscriptions[length](message);
    }
  };
  //endregion

  // Send and receive method
  const connect = () => {
    return new Promise((resolve, reject) => {
      websocket = new WebSocket(address);

      websocket.onopen = () => {
        log.debug(`Connection with ${address} opened`);
        send("connect")
          .then(() => {
            resolve();
          })
          .catch(() => {
            reject();
          });
      };

      websocket.onclose = (e) => {
        // TODO: Rethink what to do when connection is interrupted,
        //  maybe try to reconnect and not clear the suscribers?
        unsuscribeAll();
        if (e.wasClean) {
          log.debug(
            `Connection with ${address} closed, all suscribers cleared`
          );
        } else {
          log.debug(`Connection with ${address} interrupted`);
        }
      };

      websocket.onerror = (e) => {
        log.debug(`Error received from websocket: ${e.type}`);
        reject();
      };

      websocket.onmessage = (e) => {
        const message = JSON.parse(e.data);
        dispatch(message);
      };
    });
  };

  const send = (message, data) => {
    // Sending messages to remote manager
    return new Promise((resolve, reject) => {
      const id = uuidv4();

      if (!websocket) {
        reject({
          id: "",
          command: "error",
          data: {
            message: "Websocket not connected",
          },
        });
      }

      subscribeOnce(["ack", "error"], (response) => {
        if (id === response.id) {
          if (response.command === "ack") {
            resolve(response);
          } else {
            reject(response);
          }
        }
      });

      const msg = JSON.stringify({
        id: id,
        command: message,
        data: data,
      });
      websocket.send(msg);
    });
  };

  // Messages and events
  const commands = {
    connect: connect,
    launch: (configuration) => send("launch", configuration),
    run: () => send("run"),
    stop: () => send("stop"),
    pause: () => send("pause"),
    resume: () => send("resume"),
    reset: () => send("reset"),
    terminate: () => send("terminate"),
    disconnect: () => send("disconnect"),
  };

  return {
    ...commands,

    send: send,

    events: events,
    subscribe: subscribe,
    unsubscribe: unsubscribe,
    suscribreOnce: subscribeOnce,
  };
};

export default CommsManager;
