/**
 * api.js
 * REST client: thin async wrappers around each API endpoint.
 *
 * All functions return the parsed JSON response on success.
 * Callers are responsible for error handling and applying state.
 *
 * Depends on: config.js (API)
 * Consumed by: main.js
 */

import { API } from './config.js';

const JSON_HEADERS = { 'Content-Type': 'application/json' };

/** Fetches the current robot state. */
export async function fetchState() {
  const r = await fetch(`${API}/state`);
  return r.json();
}

/**
 * Moves to specified joint angles.
 * @param {number[]} joints_deg - Array of 6 angles in degrees.
 */
export async function postJoints(joints_deg) {
  const r = await fetch(`${API}/joints`, {
    method: 'POST',
    headers: JSON_HEADERS,
    body: JSON.stringify({ joints_deg }),
  });
  return r.json();
}

/**
 * Moves to a Cartesian pose via inverse kinematics.
 * @param {number} px
 * @param {number} py
 * @param {number} pz
 * @returns {Promise<Response>} Raw response (caller must check r.ok for singularity errors).
 */
export async function postPose(px, py, pz) {
  return fetch(`${API}/pose`, {
    method: 'POST',
    headers: JSON_HEADERS,
    body: JSON.stringify({ px, py, pz }),
  });
}

/**
 * Opens or closes the gripper.
 * @param {boolean} open
 */
export async function postGripper(open) {
  const r = await fetch(`${API}/gripper`, {
    method: 'POST',
    headers: JSON_HEADERS,
    body: JSON.stringify({ open }),
  });
  return r.json();
}

/**
 * Updates robot motion configuration.
 * @param {number} velocity_pct - Speed percentage (0–100).
 * @param {number} trajectory_duration - Duration in seconds.
 */
export async function postConfig(velocity_pct, trajectory_duration) {
  await fetch(`${API}/config`, {
    method: 'POST',
    headers: JSON_HEADERS,
    body: JSON.stringify({ velocity_pct, trajectory_duration }),
  });
}

/** Clears the recorded trajectory history on the server. */
export async function clearTrajectoryHistory() {
  await fetch(`${API}/trajectory/clear`, { method: 'POST' });
}

/**
 * Saves the current trajectory under a given name.
 * @param {string} name
 * @returns {Promise<Response>} Raw response (caller must check r.ok).
 */
export async function saveTrajectory(name) {
  return fetch(`${API}/trajectory/save`, {
    method: 'POST',
    headers: JSON_HEADERS,
    body: JSON.stringify({ name }),
  });
}

/** Returns the list of saved trajectory names. */
export async function listTrajectories() {
  const r = await fetch(`${API}/trajectory/list`);
  return r.json();
}

/**
 * Loads and replays a saved trajectory by name.
 * @param {string} name
 * @returns {Promise<Response>} Raw response (caller must check r.ok).
 */
export async function loadTrajectory(name) {
  return fetch(`${API}/trajectory/load`, {
    method: 'POST',
    headers: JSON_HEADERS,
    body: JSON.stringify({ name }),
  });
}
