import * as log from "loglevel";
import { v4 as uuidv4 } from "uuid";

const CommsManager = (address) => {
  log.enableAll();

  //region Observer pattern methods
  // TODO: Maybe move to it's own class?
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

  //region  Websocket handling, connect, send, receive
  let websocket = null;

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
        //unsuscribeAll();
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
  //endregion

  // Events and commands
  const events = {
    RESPONSES: ["ack", "error"],
    UPDATE: "update",
    STATE_CHANGED: "state-changed",
    INTROSPECTION: "introspection",
  };

  const commands = {
    connect: connect,
    launchWorld: (configuration) => send("launch_world", configuration),
    prepareVisualization: (visualization) => send("prepare_visualization", visualization),
    run: (code) => send("run_application", code),
    stop: () => send("stop"),
    pause: () => send("pause"),
    resume: () => send("resume"),
    reset: () => send("reset"),
    terminate_application: () => send("terminate_application"),
    terminate_visualization: () => send("terminate_visualization"),
    terminate_universe: () => send("terminate_universe"),
    disconnect: () => send("disconnect"),
    style_check: () => send("style_check"),
  };

  return {
    ...commands,

    events: events,
    send: send,

    subscribe: subscribe,
    unsubscribe: unsubscribe,
    suscribreOnce: subscribeOnce,
  };
};

export default CommsManager;
