import { useEffect, useRef } from 'react'
import L from 'leaflet'
import { swarmTerrainOverlayUrl } from './api.js'
import { vehicleColor, vehicleColorIndex } from './vehiclePalette.js'

export default function SwarmMap({ terrain, snapshot, previewRoutes, fleet, spawnDraft, onPick }) {
  const containerRef = useRef(null)
  const mapRef = useRef(null)
  const terrainLayerRef = useRef(null)
  const routeLayerRef = useRef(null)
  const conflictLayerRef = useRef(null)
  const vehicleLayerRef = useRef(null)
  const spawnLayerRef = useRef(null)
  const pickRef = useRef(onPick)
  useEffect(() => { pickRef.current = onPick }, [onPick])

  useEffect(() => {
    if (!containerRef.current || mapRef.current) return
    const map = L.map(containerRef.current, { zoomControl: true })
    L.tileLayer(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      { maxZoom: 19, attribution: '© Esri' },
    ).addTo(map)
    terrainLayerRef.current = L.layerGroup().addTo(map)
    routeLayerRef.current = L.layerGroup().addTo(map)
    conflictLayerRef.current = L.layerGroup().addTo(map)
    vehicleLayerRef.current = L.layerGroup().addTo(map)
    spawnLayerRef.current = L.layerGroup().addTo(map)
    map.on('click', (event) => pickRef.current?.(event.latlng.lng, event.latlng.lat))
    mapRef.current = map
    setTimeout(() => map.invalidateSize(), 0)
    return () => {
      map.remove()
      mapRef.current = null
    }
  }, [])

  useEffect(() => {
    if (!mapRef.current || !terrain) return
    terrainLayerRef.current.clearLayers()
    const bounds = [
      [terrain.geo_bounds.south, terrain.geo_bounds.west],
      [terrain.geo_bounds.north, terrain.geo_bounds.east],
    ]
    L.imageOverlay(swarmTerrainOverlayUrl(), bounds, { opacity: 0.82 })
      .addTo(terrainLayerRef.current)
    L.rectangle(bounds, { color: '#67e8f9', weight: 1.5, fill: false, dashArray: '6 5' })
      .bindTooltip('Davosdorf · ROS world')
      .addTo(terrainLayerRef.current)
    mapRef.current.fitBounds(bounds, { padding: [24, 24] })
  }, [terrain])

  useEffect(() => {
    if (!routeLayerRef.current) return
    routeLayerRef.current.clearLayers()
    const paths = snapshot?.display_paths
      ? Object.entries(snapshot.display_paths).map(([name, path]) => ({ name, points: path.points, goalId: path.goal_id }))
      : (previewRoutes || []).map((route) => ({ name: route.uav_name, points: route.display_points, goalId: route.goal_id }))
    paths.forEach((path) => {
      const coordinates = (path.points || []).map((point) => [point.lat, point.lng])
      if (coordinates.length < 2) return
      const color = vehicleColor(path.name)
      L.polyline(coordinates, {
        color,
        weight: 4,
        opacity: 0.9,
        dashArray: vehicleColorIndex(path.name) % 2 ? '10 7' : undefined,
      })
        .bindTooltip(`${path.name} · goal ${path.goalId}`)
        .addTo(routeLayerRef.current)
      L.circleMarker(coordinates[0], { radius: 6, color, fillColor: '#07151c', fillOpacity: 1, weight: 3 })
        .bindTooltip(`${path.name} 起点`)
        .addTo(routeLayerRef.current)
      L.circleMarker(coordinates[coordinates.length - 1], { radius: 6, color, fillColor: color, fillOpacity: 1, weight: 2 })
        .bindTooltip(`${path.name} 终点`)
        .addTo(routeLayerRef.current)
    })
  }, [snapshot, previewRoutes])

  useEffect(() => {
    if (!conflictLayerRef.current) return
    conflictLayerRef.current.clearLayers()
    for (const conflict of snapshot?.display_conflicts || []) {
      if (!conflict.first_geo || !conflict.second_geo) continue
      const color = conflict.conflict_type === 2 ? '#fb7185' : '#fbbf24'
      const points = [
        [conflict.first_geo.lat, conflict.first_geo.lng],
        [conflict.second_geo.lat, conflict.second_geo.lng],
      ]
      L.polyline(points, { color, weight: 7, opacity: 0.95 })
        .bindTooltip(`${conflict.first_uav} ↔ ${conflict.second_uav} · ${Number(conflict.separation_m).toFixed(1)} m`)
        .addTo(conflictLayerRef.current)
      for (const point of points) {
        L.circleMarker(point, { radius: 9, color: '#fff', fillColor: color, fillOpacity: 1, weight: 2 })
          .addTo(conflictLayerRef.current)
      }
    }
  }, [snapshot])

  useEffect(() => {
    if (!spawnLayerRef.current) return
    spawnLayerRef.current.clearLayers()
    if (!spawnDraft?.display_position) return
    const point = spawnDraft.display_position
    const color = vehicleColor(spawnDraft.uav_name)
    L.circleMarker([point.lat, point.lng], {
      radius: 11,
      color: '#fff',
      fillColor: color,
      fillOpacity: 0.9,
      weight: 2,
      dashArray: '3 2',
    })
      .bindTooltip(`${spawnDraft.uav_name} 待创建起点`, { permanent: true, direction: 'top' })
      .addTo(spawnLayerRef.current)
  }, [spawnDraft])

  useEffect(() => {
    if (!vehicleLayerRef.current) return
    vehicleLayerRef.current.clearLayers()
    const runtime = fleet?.runtime || {}
    const vehicles = fleet?.vehicles || []
    vehicles.forEach((vehicle) => {
      const live = runtime[vehicle.uav_name] || {}
      const point = live.display_position
      if (!point) return
      const color = vehicleColor(vehicle.uav_name)
      const trail = (live.display_trail || []).map((item) => [item.lat, item.lng])
      if (trail.length > 1) {
        L.polyline(trail, { color, weight: 2.5, opacity: 0.75 })
          .bindTooltip(`${vehicle.uav_name} 实际轨迹`)
          .addTo(vehicleLayerRef.current)
      }
      const icon = L.divIcon({
        className: 'aircraft-marker-wrap',
        html: `<span class="aircraft-marker" style="--aircraft-color:${color}">✈</span><span class="aircraft-marker-label">${String(vehicle.uav_name).replace(/[^a-z0-9_]/gi, '')}</span>`,
        iconSize: [58, 46],
        iconAnchor: [29, 15],
      })
      L.marker([point.lat, point.lng], { icon, zIndexOffset: 1000 })
        .bindTooltip(`${vehicle.uav_name} · ${vehicle.mode || '模式未知'} · ${Number(live.groundspeed_mps || 0).toFixed(1)} m/s`)
        .addTo(vehicleLayerRef.current)
    })
  }, [fleet])

  return <div className="swarm-map" ref={containerRef} />
}
