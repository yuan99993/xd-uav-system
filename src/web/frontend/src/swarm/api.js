const BASE = '/api/swarm'

async function request(url, options) {
  const response = await fetch(url, options)
  if (response.ok) return response.json()
  const text = await response.text()
  try {
    const body = JSON.parse(text)
    throw new Error(body.detail || `请求失败 (${response.status})`)
  } catch (error) {
    if (error instanceof SyntaxError) {
      throw new Error(text || `请求失败 (${response.status})`)
    }
    throw error
  }
}

export function getSwarmHealth() {
  return request(`${BASE}/health`)
}

export function getRuntimeCapabilities() {
  return request(`${BASE}/capabilities`)
}

export function getSwarmTerrain() {
  return request(`${BASE}/terrain`)
}

export function dispatchSwarm(routes, useMock) {
  return request(`${BASE}/dispatch`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ frame_id: 'world', use_mock: useMock, routes }),
  })
}

export function getSwarmSnapshot(useMock) {
  return request(`${BASE}/snapshot?use_mock=${useMock ? 'true' : 'false'}`)
}

export function getFleet(useMock) {
  return request(`${BASE}/fleet?use_mock=${useMock ? 'true' : 'false'}`)
}

export function spawnVehicle(uavName, worldOrigin, useMock) {
  return request(`${BASE}/fleet/spawn`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      uav_name: uavName,
      world_origin: worldOrigin,
      use_mock: useMock,
    }),
  })
}

export function removeVehicle(uavName, useMock) {
  return request(`${BASE}/fleet/remove`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ uav_name: uavName, use_mock: useMock }),
  })
}

export function takeoffFleet(uavNames, altitude, useMock) {
  return request(`${BASE}/fleet/takeoff`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      uav_names: uavNames,
      takeoff_altitude_m: altitude,
      use_mock: useMock,
    }),
  })
}

export function landFleet(uavNames, useMock) {
  return request(`${BASE}/fleet/land`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ uav_names: uavNames, use_mock: useMock }),
  })
}

export function restartSwarm(useMock) {
  return request(`${BASE}/system/restart`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ use_mock: useMock }),
  })
}

export function geoToWorld(longitude, latitude, clearance = 80) {
  return request(`${BASE}/coordinates/to_world`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ longitude, latitude, clearance_m: clearance }),
  })
}

export function getWorldElevation(x, y, clearance = 80) {
  return request(`${BASE}/coordinates/elevation`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ x, y, clearance_m: clearance }),
  })
}

export function swarmTerrainOverlayUrl() {
  return `${BASE}/terrain.png`
}
