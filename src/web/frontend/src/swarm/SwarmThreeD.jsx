import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { swarmTerrainOverlayUrl } from './api.js'
import { vehicleColor } from './vehiclePalette.js'

function disposeGroup(group) {
  while (group.children.length) {
    const object = group.children.pop()
    object.geometry?.dispose()
    if (Array.isArray(object.material)) object.material.forEach((item) => item.dispose())
    else object.material?.dispose()
  }
}

function pointsForScene(points, elevationMin, exaggeration) {
  return (points || []).map((point) => new THREE.Vector3(
    Number(point.x),
    (Number(point.z) - elevationMin) * exaggeration,
    -Number(point.y),
  ))
}

function terrainPointForScene(point, raster, exaggeration) {
  const bounds = raster.world_bounds
  const xRatio = (Number(point.x) - bounds.x_min) / (bounds.x_max - bounds.x_min)
  const yRatio = (bounds.y_max - Number(point.y)) / (bounds.y_max - bounds.y_min)
  if (xRatio < 0 || xRatio > 1 || yRatio < 0 || yRatio > 1) return null
  const column = Math.round(xRatio * (raster.cols - 1))
  const row = Math.round(yRatio * (raster.rows - 1))
  const elevation = raster.elevation[row]?.[column]
  if (!Number.isFinite(elevation)) return null
  return new THREE.Vector3(
    Number(point.x),
    (elevation - raster.elevation_min_m) * exaggeration,
    -Number(point.y),
  )
}

function routeSignature(paths) {
  return paths.map(({ name, points }) => {
    const first = points?.[0] || {}
    const last = points?.[points.length - 1] || {}
    return `${name}:${points?.length || 0}:${first.x}:${first.y}:${first.z}:${last.x}:${last.y}:${last.z}`
  }).join('|')
}

function focusPoints(state, points) {
  if (!state || !points?.length) return
  const bounds = new THREE.Box3().setFromPoints(points)
  const center = bounds.getCenter(new THREE.Vector3())
  const size = bounds.getSize(new THREE.Vector3())
  const horizontal = Math.max(size.x, size.z, 600)
  const height = Math.max(size.y, 120)
  state.camera.position.set(
    center.x + horizontal * 0.68,
    center.y + Math.max(horizontal * 0.48, height * 2.2),
    center.z + horizontal * 0.68,
  )
  state.controls.target.copy(center)
  state.controls.update()
}

export default function SwarmThreeD({ raster, snapshot, previewRoutes, fleet, mode }) {
  const containerRef = useRef(null)
  const sceneRef = useRef(null)
  const [exaggeration, setExaggeration] = useState(1.5)

  useEffect(() => {
    if (!containerRef.current || !raster) return undefined
    const container = containerRef.current
    const width = container.clientWidth
    const height = container.clientHeight
    if (width < 2 || height < 2) return undefined

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setPixelRatio(window.devicePixelRatio)
    renderer.setSize(width, height)
    container.appendChild(renderer.domElement)

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x0b1e25)
    const terrainSize = Math.max(raster.width_m, raster.height_m)
    scene.fog = new THREE.Fog(0x0b1e25, terrainSize * 1.4, terrainSize * 4.0)
    const camera = new THREE.PerspectiveCamera(48, width / height, 1, terrainSize * 10)
    const homePosition = new THREE.Vector3(
      terrainSize * 0.55,
      terrainSize * 0.52,
      terrainSize * 0.72,
    )
    camera.position.copy(homePosition)
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.08
    controls.maxPolarAngle = Math.PI / 2.02
    const homeTarget = new THREE.Vector3(
      0, (raster.elevation_max_m - raster.elevation_min_m) * exaggeration * 0.35, 0)
    controls.target.copy(homeTarget)

    scene.add(new THREE.HemisphereLight(0xdaf5ff, 0x263b32, 1.15))
    const sun = new THREE.DirectionalLight(0xfff2d1, 1.5)
    sun.position.set(-terrainSize, terrainSize * 1.5, -terrainSize)
    scene.add(sun)

    const { rows, cols, width_m: terrainWidth, height_m: terrainHeight } = raster
    const geometry = new THREE.PlaneGeometry(
      terrainWidth, terrainHeight, cols - 1, rows - 1)
    geometry.rotateX(-Math.PI / 2)
    const positions = geometry.attributes.position
    for (let index = 0; index < positions.count; index += 1) {
      const row = Math.floor(index / cols)
      const column = index % cols
      const elevation = raster.elevation[row]?.[column]
      positions.setY(index, ((elevation ?? raster.elevation_min_m) - raster.elevation_min_m) * exaggeration)
    }
    positions.needsUpdate = true
    geometry.computeVertexNormals()
    const material = new THREE.MeshStandardMaterial({ color: 0x91a68e, roughness: 0.92 })
    const textureLoader = new THREE.TextureLoader()
    textureLoader.load(swarmTerrainOverlayUrl(), (texture) => {
      texture.colorSpace = THREE.SRGBColorSpace
      material.map = texture
      material.color.set(0xffffff)
      material.needsUpdate = true
    })
    scene.add(new THREE.Mesh(geometry, material))

    const pathGroup = new THREE.Group()
    const trailGroup = new THREE.Group()
    const aircraftGroup = new THREE.Group()
    scene.add(pathGroup, trailGroup, aircraftGroup)
    sceneRef.current = {
      renderer, scene, camera, controls, homePosition, homeTarget,
      missionPoints: [], routeSignature: '',
      pathGroup, trailGroup, aircraftGroup,
    }

    const resizeObserver = new ResizeObserver(() => {
      const nextWidth = container.clientWidth
      const nextHeight = container.clientHeight
      if (nextWidth < 2 || nextHeight < 2) return
      renderer.setSize(nextWidth, nextHeight)
      camera.aspect = nextWidth / nextHeight
      camera.updateProjectionMatrix()
    })
    resizeObserver.observe(container)

    let frame = 0
    const render = () => {
      frame = requestAnimationFrame(render)
      controls.update()
      renderer.render(scene, camera)
    }
    render()
    return () => {
      cancelAnimationFrame(frame)
      resizeObserver.disconnect()
      controls.dispose()
      disposeGroup(pathGroup)
      disposeGroup(trailGroup)
      disposeGroup(aircraftGroup)
      geometry.dispose()
      material.map?.dispose()
      material.dispose()
      renderer.dispose()
      if (renderer.domElement.parentNode === container) container.removeChild(renderer.domElement)
      sceneRef.current = null
    }
  }, [raster, exaggeration])

  useEffect(() => {
    const state = sceneRef.current
    if (!state || !raster) return
    disposeGroup(state.pathGroup)

    const resultPaths = snapshot?.display_paths
      ? Object.entries(snapshot.display_paths).map(([name, path]) => ({ name, points: path.points }))
      : (previewRoutes || []).map((route) => ({ name: route.uav_name, points: route.points }))
    const missionPoints = []
    resultPaths.forEach((path) => {
      const points = pointsForScene(path.points, raster.elevation_min_m, exaggeration)
      if (points.length < 2) return
      missionPoints.push(...points)
      const material = new THREE.LineBasicMaterial({ color: vehicleColor(path.name) })
      state.pathGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), material))

      // Sparse vertical guides make the real MSL altitude profile readable;
      // their endpoints come from the DEM and the unchanged ROS path.
      const guideVertices = []
      const stride = Math.max(1, Math.ceil(path.points.length / 14))
      for (let index = 0; index < path.points.length; index += stride) {
        const terrainPoint = terrainPointForScene(path.points[index], raster, exaggeration)
        if (terrainPoint) guideVertices.push(terrainPoint, points[index])
      }
      const lastIndex = path.points.length - 1
      if (lastIndex % stride !== 0) {
        const terrainPoint = terrainPointForScene(path.points[lastIndex], raster, exaggeration)
        if (terrainPoint) guideVertices.push(terrainPoint, points[lastIndex])
      }
      if (guideVertices.length) {
        const guideMaterial = new THREE.LineBasicMaterial({
          color: vehicleColor(path.name), transparent: true, opacity: 0.38,
        })
        state.pathGroup.add(new THREE.LineSegments(
          new THREE.BufferGeometry().setFromPoints(guideVertices), guideMaterial))
      }
    })
    state.missionPoints = missionPoints
    const signature = routeSignature(resultPaths)
    if (signature && signature !== state.routeSignature) focusPoints(state, missionPoints)
    state.routeSignature = signature
  }, [snapshot, previewRoutes, raster, exaggeration])

  useEffect(() => {
    const state = sceneRef.current
    if (!state || !raster) return
    disposeGroup(state.trailGroup)
    disposeGroup(state.aircraftGroup)
    const runtime = fleet?.runtime || {}
    Object.entries(runtime).forEach(([name, live]) => {
      const trail = pointsForScene(live.display_trail || live.trail, raster.elevation_min_m, exaggeration)
      if (trail.length > 1) {
        const trailMaterial = new THREE.LineDashedMaterial({ color: vehicleColor(name), dashSize: 35, gapSize: 20 })
        const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(trail), trailMaterial)
        line.computeLineDistances()
        state.trailGroup.add(line)
      }
      const point = live.world_position
      if (!point) return
      const [position] = pointsForScene([point], raster.elevation_min_m, exaggeration)
      const marker = new THREE.Mesh(
        new THREE.ConeGeometry(24, 70, 4),
        new THREE.MeshStandardMaterial({ color: vehicleColor(name), emissive: vehicleColor(name), emissiveIntensity: 0.2 }),
      )
      marker.rotation.x = Math.PI / 2
      marker.position.copy(position)
      state.aircraftGroup.add(marker)
    })
  }, [fleet, raster, exaggeration])

  const resetCamera = () => {
    const state = sceneRef.current
    if (!state) return
    state.camera.position.copy(state.homePosition)
    state.controls.target.copy(state.homeTarget)
    state.controls.update()
  }

  const focusMission = () => focusPoints(sceneRef.current, sceneRef.current?.missionPoints)

  return <div className="swarm-three-d">
    <div className="three-d-canvas" ref={containerRef} />
    <div className="three-d-toolbar">
      <span>{mode === 'ros' ? 'ROS 正式三维航迹与实况' : '离线三维任务回显'} · world / MSL · 竖线为离地高度</span>
      {[1, 1.5, 2.5].map((value) => <button key={value} className={exaggeration === value ? 'active' : ''} onClick={() => setExaggeration(value)}>{value}×</button>)}
      <button onClick={focusMission}>聚焦航线</button>
      <button onClick={resetCamera}>地形总览</button>
    </div>
  </div>
}
