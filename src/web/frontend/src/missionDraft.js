const STORAGE_KEY = 'xd-uav.fixedwing-mission-draft.v1'

function finitePair(point, label) {
  if (!Array.isArray(point) || point.length !== 2
      || !point.every((value) => Number.isFinite(Number(value)))) {
    throw new Error(`${label}必须是有效的经纬度坐标`)
  }
  return { longitude: Number(point[0]), latitude: Number(point[1]) }
}

export function saveMissionDraft({ start, goal, startAlt, goalAlt, clearanceM, terrainInfo, terrainPackage }) {
  const draft = {
    schema: 'xd-uav.fixedwing-mission-draft',
    version: 2,
    created_at: new Date().toISOString(),
    horizontal_crs: 'EPSG:4326',
    vertical_datum: 'MSL',
    source: 'terrain-preview',
    terrain: {
      terrain_id: terrainPackage?.terrain_id || null,
      source: terrainInfo?.terrain_meta?.source || terrainInfo?.filename || 'online_terrarium',
      bounds: terrainInfo?.bounds || null,
      elevation_min_m: terrainInfo?.elevation_min ?? null,
      elevation_max_m: terrainInfo?.elevation_max ?? null,
    },
    start: { ...finitePair(start, '起点'), altitude_msl: startAlt ?? null },
    goal: { ...finitePair(goal, '终点'), altitude_msl: goalAlt ?? null },
    requested_clearance_m: Number.isFinite(Number(clearanceM)) ? Number(clearanceM) : 80,
  }
  window.sessionStorage.setItem(STORAGE_KEY, JSON.stringify(draft))
  return draft
}

export function consumeMissionDraft() {
  const raw = window.sessionStorage.getItem(STORAGE_KEY)
  if (!raw) return null
  window.sessionStorage.removeItem(STORAGE_KEY)
  let draft
  try {
    draft = JSON.parse(raw)
  } catch (_error) {
    throw new Error('共享任务草稿已损坏，请回到地图重新选择起终点')
  }
  if (draft?.schema !== 'xd-uav.fixedwing-mission-draft' || ![1, 2].includes(draft?.version)
      || draft.horizontal_crs !== 'EPSG:4326' || draft.vertical_datum !== 'MSL') {
    throw new Error('共享任务草稿的坐标基准不受支持')
  }
  finitePair([draft.start?.longitude, draft.start?.latitude], '起点')
  finitePair([draft.goal?.longitude, draft.goal?.latitude], '终点')
  if (draft.version >= 2 && !draft.terrain?.terrain_id) {
    throw new Error('共享任务草稿缺少正式地形标识，请回到地图重新转交')
  }
  return draft
}

export function clearMissionDraft() {
  window.sessionStorage.removeItem(STORAGE_KEY)
}

export { STORAGE_KEY as MISSION_DRAFT_STORAGE_KEY }
