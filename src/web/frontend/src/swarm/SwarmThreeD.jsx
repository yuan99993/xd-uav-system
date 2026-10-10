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
    controls.target.set(0, 180, 0)

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
      renderer, scene, camera, controls, homePosition,
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
    disposeGroup(state.trailGroup)
    disposeGroup(state.aircraftGroup)

    const resultPaths = snapshot?.display_paths
      ? Object.entries(snapshot.display_paths).map(([name, path]) => ({ name, points: path.points }))
      : (previewRoutes || []).map((route) => ({ name: route.uav_name, points: route.points }))
    resultPaths.forEach((path) => {
      const points = pointsForScene(path.points, raster.elevation_min_m, exaggeration)
      if (points.length < 2) return
      const material = new THREE.LineBasicMaterial({ color: vehicleColor(path.name) })
      state.pathGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), material))
    })

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
  }, [snapshot, previewRoutes, fleet, raster, exaggeration])

  const resetCamera = () => {
    const state = sceneRef.current
    if (!state) return
    state.camera.position.copy(state.homePosition)
    state.controls.target.set(0, 180, 0)
    state.controls.update()
  }

  return <div className="swarm-three-d">
    <div className="three-d-canvas" ref={containerRef} />
    <div className="three-d-toolbar">
      <span>{mode === 'ros' ? 'ROS 正式航迹与实况' : '离线任务回显'} · world / MSL</span>
      {[1, 1.5, 2.5].map((value) => <button key={value} className={exaggeration === value ? 'active' : ''} onClick={() => setExaggeration(value)}>{value}×</button>)}
      <button onClick={resetCamera}>重置视角</button>
    </div>
  </div>
}
