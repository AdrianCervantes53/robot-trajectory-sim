/**
 * websocket.js
 * WebSocket connection lifecycle and trajectory message streaming.
 *
 * Responsibilities:
 *  - Opens the WS connection and auto-reconnects on close.
 *  - Exposes a thin interface: send(), close(), isOpen().
 *
 * Depends on: config.js (WS)
 * Consumed by: main.js
 */

import { WS } from './config.js';

/**
 * Creates and manages a WebSocket connection to the robot API.
 *
 * @param {object} handlers
 * @param {function(object): void} handlers.onFrame  - Called with each parsed JSON frame.
 * @param {function(): void}       handlers.onOpen   - Called when connection is established.
 * @param {function(): void}       handlers.onClose  - Called on disconnect (before retry).
 * @param {function(): void}       handlers.onError  - Called on WebSocket error.
 *
 * @returns {{ send: function(object): void, close: function(): void, isOpen: function(): boolean }}
 */
export function createWSConnection({ onFrame, onOpen, onClose, onError }) {
  let ws = null;

  function connect() {
    ws = new WebSocket(WS);

    ws.onopen = onOpen;

    ws.onmessage = (evt) => {
      try {
        onFrame(JSON.parse(evt.data));
      } catch {
        console.error('WS: failed to parse frame', evt.data);
      }
    };

    ws.onerror = onError;

    ws.onclose = () => {
      onClose();
      setTimeout(connect, 2000);
    };
  }

  connect();

  return {
    /** Sends a message if the socket is open. Silently drops if not. */
    send(msg) {
      if (ws?.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(msg));
      }
    },
    /** Closes the connection (suppresses auto-reconnect for that cycle). */
    close() {
      ws?.close();
    },
    /** Returns true if the socket is currently open. */
    isOpen() {
      return ws?.readyState === WebSocket.OPEN;
    },
  };
}
