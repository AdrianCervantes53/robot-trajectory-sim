/**
 * config.js
 * Runtime configuration: API base URL and WebSocket URL.
 *
 * Derived from window.location so the simulator works from any device
 * on the network — not just the machine running the server.
 */

const _host = window.location.hostname;
const _port = window.location.port || '8000';

/** Base URL for all REST API calls. */
export const API = `${window.location.protocol}//${_host}:${_port}`;

/** WebSocket URL. Automatically uses wss:// when the page is served over HTTPS. */
export const WS = `${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${_host}:${_port}/ws`;

/** Scale factor: converts MATLAB distance units to Three.js scene units. */
export const SC = 18;
