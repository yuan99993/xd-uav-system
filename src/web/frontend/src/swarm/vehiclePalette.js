export const VEHICLE_COLORS = [
  '#34d399', // uav1 · green
  '#38bdf8', // uav2 · blue
  '#fbbf24', // uav3 · amber
  '#c084fc', // uav4 · purple
  '#fb7185', // uav5 · rose
  '#fb923c', // uav6 · orange
  '#a3e635', // uav7 · lime
  '#22d3ee', // uav8 · cyan
]

export function vehicleColorIndex(name) {
  const match = /^uav(\d+)$/i.exec(String(name || ''))
  if (match && Number(match[1]) > 0) {
    return (Number(match[1]) - 1) % VEHICLE_COLORS.length
  }
  let hash = 0
  for (const character of String(name || '')) {
    hash = ((hash * 31) + character.codePointAt(0)) >>> 0
  }
  return hash % VEHICLE_COLORS.length
}

export function vehicleColor(name) {
  return VEHICLE_COLORS[vehicleColorIndex(name)]
}
