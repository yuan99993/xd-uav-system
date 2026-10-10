import { useEffect, useMemo, useState } from 'react'
import SwarmMap from './SwarmMap.jsx'
import SwarmThreeD from './SwarmThreeD.jsx'
import { vehicleColor } from './vehiclePalette.js'
import { consumeMissionDraft } from '../missionDraft.js'
import {
  dispatchSwarm, geoToWorld, getFleet, getSwarmHealth, getSwarmSnapshot,
  getSwarmTerrain, getSwarmTerrainRaster, getRuntimeCapabilities, getWorldElevation, landFleet, removeVehicle, spawnVehicle,
  restartSwarm, takeoffFleet,
} from './api.js'

const PLAN_STATES = ['空闲', '规划中', '执行中', '已到达', '受阻', '失败']
const FLEET_STATES = ['启动中', '就绪', '停止中', '错误']
const PLANNING_CLEARANCE_M = 80

function cloneRoute(route) {
  return {
    uav_name: route.uav_name,
    goal_id: route.goal_id,
    nominal_speed_mps: route.nominal_speed_mps,
    points: route.points.map((point) => ({ ...point })),
    display_points: route.display_points?.map((point) => ({ ...point })),
  }
}

function NumberField({ label, value, onChange, step = 1, disabled = false }) {
  return <label className="field"><span>{label}</span><input type="number" value={value} step={step} disabled={disabled} onChange={(event) => onChange(Number(event.target.value))} /></label>
}

function ElevationStrip({ snapshot }) {
  const paths = Object.entries(snapshot?.display_paths || {}).filter(([, path]) => path.points?.length > 1)
  if (!paths.length) return <div className="empty-state">下发任务后显示规划航迹高度</div>
  const width = 760
  const height = 150
  const pad = 22
  const allPoints = paths.flatMap(([, path]) => path.points)
  const zMin = Math.min(...allPoints.map((point) => point.z))
  const zMax = Math.max(...allPoints.map((point) => point.z))
  const polylines = paths.map(([name, path]) => {
    let distance = 0
    const values = path.points.map((point, pointIndex) => {
      if (pointIndex) {
        const previous = path.points[pointIndex - 1]
        distance += Math.hypot(point.x - previous.x, point.y - previous.y)
      }
      return { distance, z: point.z }
    })
    const total = Math.max(distance, 1)
    return {
      name,
      color: vehicleColor(name),
      points: values.map((value) => {
        const x = pad + (value.distance / total) * (width - 2 * pad)
        const y = height - pad - ((value.z - zMin) / Math.max(zMax - zMin, 1)) * (height - 2 * pad)
        return `${x.toFixed(1)},${y.toFixed(1)}`
      }).join(' '),
    }
  })
  return <div className="profile-wrap">
    <svg viewBox={`0 0 ${width} ${height}`} role="img" aria-label="各机规划航迹高度">
      <line x1={pad} y1={height - pad} x2={width - pad} y2={height - pad} className="profile-axis" />
      {polylines.map((line) => <polyline key={line.name} points={line.points} fill="none" stroke={line.color} strokeWidth="3" />)}
      <text x={pad} y={16}>{Math.round(zMax)} m</text><text x={pad} y={height - 5}>{Math.round(zMin)} m</text>
    </svg>
    <div className="profile-legend">{polylines.map((line) => <span key={line.name} style={{ '--legend': line.color }}>{line.name}</span>)}</div>
  </div>
}

export default function App() {
  const [health, setHealth] = useState(null)
  const [capabilities, setCapabilities] = useState(null)
  const [terrain, setTerrain] = useState(null)
  const [routes, setRoutes] = useState([])
  const [previewRoutes, setPreviewRoutes] = useState([])
  const [snapshot, setSnapshot] = useState(null)
  const [fleet, setFleet] = useState(null)
  const [mode, setMode] = useState('mock')
  const [selectedRoute, setSelectedRoute] = useState(0)
  const [selectedPoint, setSelectedPoint] = useState(1)
  const [takeoffAltitude, setTakeoffAltitude] = useState(80)
  const [busy, setBusy] = useState(false)
  const [restarting, setRestarting] = useState(false)
  const [selectingSpawn, setSelectingSpawn] = useState(false)
  const [spawnDraft, setSpawnDraft] = useState(null)
  const [pendingMissionDraft, setPendingMissionDraft] = useState(null)
  const [visualMode, setVisualMode] = useState('2d')
  const [terrainRaster, setTerrainRaster] = useState(null)
  const [hasDispatched, setHasDispatched] = useState(false)
  const [notice, setNotice] = useState({ type: 'info', text: '正在加载固定地形和飞行接口…' })

  useEffect(() => {
    Promise.all([getSwarmHealth(), getSwarmTerrain(), getRuntimeCapabilities()])
      .then(([healthResult, terrainResult, capabilityResult]) => {
        setHealth(healthResult)
        setTerrain(terrainResult)
        setCapabilities(capabilityResult)
        setRoutes([])
        setPreviewRoutes([])
        setNotice({ type: 'ok', text: capabilityResult.runtime.rosbridge_reachable ? '执行系统接口已上线；可切换到 ROS 执行' : 'Web 已就绪；当前可离线编排和校验任务' })
      })
      .catch((error) => setNotice({ type: 'error', text: error.message }))
  }, [])

  useEffect(() => {
    let active = true
    const refresh = () => getRuntimeCapabilities()
      .then((result) => { if (active) setCapabilities(result) })
      .catch(() => { if (active) setCapabilities(null) })
    const timer = window.setInterval(refresh, 3000)
    return () => { active = false; window.clearInterval(timer) }
  }, [])

  useEffect(() => {
    if (restarting) return undefined
    let active = true
    const refresh = () => {
      getFleet(mode === 'mock')
        .then((result) => { if (active) setFleet(result) })
        .catch((error) => { if (active) setNotice({ type: 'error', text: error.message }) })
      if (hasDispatched) {
        getSwarmSnapshot(mode === 'mock')
          .then((result) => { if (active) setSnapshot(result) })
          .catch((error) => { if (active) setNotice({ type: 'error', text: error.message }) })
      }
    }
    refresh()
    const timer = window.setInterval(refresh, 1500)
    return () => { active = false; window.clearInterval(timer) }
  }, [hasDispatched, mode, restarting])

  const vehicles = fleet?.vehicles || []
  const onlineNames = vehicles.map((vehicle) => vehicle.uav_name)
  const status = snapshot?.status
  const conflicts = snapshot?.display_conflicts || []
  const spatialWarnings = conflicts.filter((conflict) => conflict.conflict_type === 1)
  const blockingConflicts = conflicts.filter((conflict) => conflict.conflict_type !== 1)
  const statusTone = !status?.analysis_ready ? 'waiting' : !status.conflict_free ? 'danger' : spatialWarnings.length ? 'warning' : 'safe'
  const nextVehicleExample = terrain?.vehicle_examples?.find((item) => !onlineNames.includes(item.uav_name))
  const unassignedVehicle = vehicles.find((vehicle) => !routes.some((route) => route.uav_name === vehicle.uav_name))
  const canTakeoff = Boolean(status?.analysis_ready && status?.conflict_free && routes.length >= 2)
  const canRestart = mode === 'mock' || vehicles.every((vehicle) => (
    Number(vehicle.lifecycle) === 1 && vehicle.connected
    && !vehicle.armed && vehicle.on_ground
  ))
  const contractSummary = useMemo(() => health ? `${health.ros_contract.input_type} · ${health.ros_contract.frame_id}` : '接口检查中', [health])
  const runtime = capabilities?.runtime
  const runtimeLabel = runtime?.execution_available
    ? `执行系统在线 · ${runtime.aircraft_count} 架`
    : runtime?.rosbridge_reachable ? 'ROS 接口可连接' : 'Web 独立运行'

  useEffect(() => {
    if (!terrain) return
    let draft
    try {
      draft = consumeMissionDraft()
    } catch (error) {
      setNotice({ type: 'error', text: error.message })
      return
    }
    if (!draft) return
    const clearance = Math.max(51, Math.min(119, Number(draft.requested_clearance_m) || PLANNING_CLEARANCE_M))
    Promise.all([
      geoToWorld(draft.start.longitude, draft.start.latitude, clearance),
      geoToWorld(draft.goal.longitude, draft.goal.latitude, clearance),
    ]).then(([startWorld, goalWorld]) => {
      setPendingMissionDraft({ draft, clearance, startWorld, goalWorld })
      setNotice({
        type: 'ok',
        text: '地图任务草稿已接收：起点用于创建下一架飞机，终点将在新建任务时带入；仍须由正式 ROS 规划器重新规划',
      })
    }).catch((error) => {
      setNotice({
        type: 'error',
        text: `地图草稿不能用于当前执行地形：${error.message}。旧页面仍可预览任意在线 DEM，但 ROS/Gazebo 当前只执行 Davosdorf 基准。`,
      })
    })
  }, [terrain])

  useEffect(() => {
    if (!pendingMissionDraft || !nextVehicleExample || spawnDraft) return
    const world = pendingMissionDraft.startWorld
    setSpawnDraft({
      uav_name: nextVehicleExample.uav_name,
      world_origin: {
        x: Number(world.x.toFixed(3)),
        y: Number(world.y.toFixed(3)),
        z: Number(world.ground_elevation_m.toFixed(3)),
      },
      display_position: {
        lng: pendingMissionDraft.draft.start.longitude,
        lat: pendingMissionDraft.draft.start.latitude,
      },
    })
  }, [pendingMissionDraft, nextVehicleExample, spawnDraft])

  const markEdited = () => {
    setSnapshot(null)
    setPreviewRoutes([])
    setHasDispatched(false)
  }
  const switchVisualMode = async (nextMode) => {
    if (nextMode === '2d') {
      setVisualMode('2d')
      return
    }
    setVisualMode('3d')
    if (terrainRaster) return
    try {
      setNotice({ type: 'info', text: '正在加载与 ROS 一致的三维 DEM…' })
      setTerrainRaster(await getSwarmTerrainRaster())
      setNotice({ type: 'ok', text: '三维视图已加载；规划线来自当前正式结果，飞机标记来自实时状态' })
    } catch (error) {
      setVisualMode('2d')
      setNotice({ type: 'error', text: `三维地形加载失败：${error.message}` })
    }
  }
  const refreshFleetNow = async () => setFleet(await getFleet(mode === 'mock'))
  const switchMode = (nextMode) => {
    if (nextMode === 'ros' && !runtime?.rosbridge_reachable) {
      setNotice({ type: 'error', text: '执行系统尚未上线；可继续离线编辑，启动 ROS/仿真后再切换' })
      return
    }
    setMode(nextMode)
    setHasDispatched(false)
    setSnapshot(null)
    setFleet(null)
    setSpawnDraft(null)
    setSelectingSpawn(false)
    setNotice({ type: 'info', text: nextMode === 'mock' ? '已切换到离线编排，不会向后来上线的执行系统自动补发' : '正在连接 ROS 规划与执行系统…' })
  }
  const updateRoute = (routeIndex, key, value) => {
    setRoutes((current) => current.map((route, index) => index === routeIndex ? { ...route, [key]: value } : route))
    markEdited()
  }
  const updatePoint = (routeIndex, pointIndex, key, value) => {
    setRoutes((current) => current.map((route, index) => index !== routeIndex ? route : {
      ...route,
      points: route.points.map((point, cursor) => cursor === pointIndex ? { ...point, [key]: value } : point),
    }))
    markEdited()
  }
  const addWaypoint = async (routeIndex, afterIndex) => {
    const route = routes[routeIndex]
    if (!route) return
    const first = route.points[afterIndex]
    const second = route.points[afterIndex + 1]
    const x = Number(((first.x + second.x) / 2).toFixed(3))
    const y = Number(((first.y + second.y) / 2).toFixed(3))
    try {
      const elevation = await getWorldElevation(x, y, PLANNING_CLEARANCE_M)
      const midpoint = { x, y, z: Number(elevation.flight_z_m.toFixed(3)) }
      setRoutes((current) => current.map((item, index) => {
        if (index !== routeIndex) return item
        const points = [...item.points]
        points.splice(afterIndex + 1, 0, midpoint)
        return { ...item, points }
      }))
      setSelectedRoute(routeIndex)
      setSelectedPoint(afterIndex + 1)
      markEdited()
      setNotice({ type: 'info', text: `已插入航点，Z 自动取地面高程 + ${PLANNING_CLEARANCE_M} m` })
    } catch (error) { setNotice({ type: 'error', text: error.message }) }
  }
  const removeWaypoint = (routeIndex, pointIndex) => {
    setRoutes((current) => current.map((route, index) => index === routeIndex ? { ...route, points: route.points.filter((_, cursor) => cursor !== pointIndex) } : route))
    setSelectedPoint(1)
    markEdited()
  }

  const handleMapPick = async (longitude, latitude) => {
    if (selectingSpawn) {
      if (!nextVehicleExample) return
      try {
        const world = await geoToWorld(longitude, latitude, PLANNING_CLEARANCE_M)
        setSpawnDraft({
          uav_name: nextVehicleExample.uav_name,
          world_origin: {
            x: Number(world.x.toFixed(3)),
            y: Number(world.y.toFixed(3)),
            z: Number(world.ground_elevation_m.toFixed(3)),
          },
          display_position: { lng: longitude, lat: latitude },
        })
        setSelectingSpawn(false)
        setNotice({ type: 'info', text: `${nextVehicleExample.uav_name} 起始点已选定；创建后任务起点将使用地面高程 + ${PLANNING_CLEARANCE_M} m` })
      } catch (error) { setNotice({ type: 'error', text: error.message }) }
      return
    }
    if (!routes[selectedRoute]) return
    if (selectedPoint === 0) {
      setNotice({ type: 'error', text: '任务起点绑定飞机的真实起飞位置，不能在地图上移动' })
      return
    }
    try {
      const world = await geoToWorld(longitude, latitude, PLANNING_CLEARANCE_M)
      setRoutes((current) => current.map((route, routeIndex) => routeIndex !== selectedRoute ? route : {
        ...route,
        points: route.points.map((point, pointIndex) => pointIndex !== selectedPoint ? point : {
          x: Number(world.x.toFixed(3)),
          y: Number(world.y.toFixed(3)),
          z: Number(world.flight_z_m.toFixed(3)),
        }),
      }))
      markEdited()
      const last = routes[selectedRoute].points.length - 1
      const label = selectedPoint === 0 ? '起点' : selectedPoint === last ? '终点' : `航点 ${selectedPoint}`
      setNotice({ type: 'info', text: `${routes[selectedRoute].uav_name} ${label}已更新；地面 ${world.ground_elevation_m.toFixed(1)} m，飞行 Z 自动设为 ${world.flight_z_m.toFixed(1)} m` })
    } catch (error) { setNotice({ type: 'error', text: error.message }) }
  }
  const currentRouteStart = async (uavName, fleetSnapshot = fleet) => {
    const vehicle = (fleetSnapshot?.vehicles || []).find((item) => item.uav_name === uavName)
    const position = fleetSnapshot?.runtime?.[uavName]?.world_position || vehicle?.world_origin
    if (!position) throw new Error(`${uavName} 当前世界坐标尚不可用`)
    const elevation = await getWorldElevation(position.x, position.y, PLANNING_CLEARANCE_M)
    return {
      x: Number(position.x.toFixed(3)),
      y: Number(position.y.toFixed(3)),
      z: Number(elevation.flight_z_m.toFixed(3)),
    }
  }
  const addTaskForVehicle = async (uavName) => {
    const example = terrain?.vehicle_examples?.find((item) => item.uav_name === uavName)
    if (!example || routes.some((route) => route.uav_name === uavName)) return
    setBusy(true)
    try {
      const route = cloneRoute(example.route)
      route.points[0] = await currentRouteStart(uavName)
      if (pendingMissionDraft) {
        const goal = pendingMissionDraft.goalWorld
        route.points = [route.points[0], {
          x: Number(goal.x.toFixed(3)),
          y: Number(goal.y.toFixed(3)),
          z: Number(goal.flight_z_m.toFixed(3)),
        }]
      }
      delete route.display_points
      setRoutes((current) => [...current, route])
      setSelectedRoute(routes.length)
      setSelectedPoint(1)
      markEdited()
      setNotice({ type: 'info', text: pendingMissionDraft
        ? `已为 ${uavName} 带入地图终点；起点采用飞机当前位置，正式航迹仍需点击下发并规划`
        : `已为 ${uavName} 新建任务；起点自动采用飞机当前位置的地面 + ${PLANNING_CLEARANCE_M} m` })
      setPendingMissionDraft(null)
    } catch (error) { setNotice({ type: 'error', text: error.message }) } finally { setBusy(false) }
  }
  const deleteTask = (routeIndex) => {
    const name = routes[routeIndex]?.uav_name
    setRoutes((current) => current.filter((_, index) => index !== routeIndex))
    setSelectedRoute(0)
    setSelectedPoint(1)
    markEdited()
    setNotice({ type: 'info', text: `${name} 的任务草稿已删除；飞机实例保持在线` })
  }
  const copyTask = async (routeIndex) => {
    if (!unassignedVehicle) {
      setNotice({ type: 'error', text: '没有尚未分配任务的在线飞机，请先添加飞机' })
      return
    }
    setBusy(true)
    try {
      const copied = cloneRoute(routes[routeIndex])
      const example = terrain.vehicle_examples.find((item) => item.uav_name === unassignedVehicle.uav_name)
      copied.uav_name = unassignedVehicle.uav_name
      copied.goal_id = example?.route.goal_id || Math.max(0, ...routes.map((route) => route.goal_id)) + 1
      copied.points[0] = await currentRouteStart(copied.uav_name)
      delete copied.display_points
      setRoutes((current) => [...current, copied])
      markEdited()
      setNotice({ type: 'info', text: `已把任务几何复制给 ${copied.uav_name}；起点采用该机当前位置，请调整后续航点以保持安全间隔` })
    } catch (error) { setNotice({ type: 'error', text: error.message }) } finally { setBusy(false) }
  }

  const handleAddVehicle = async () => {
    if (!nextVehicleExample || spawnDraft?.uav_name !== nextVehicleExample.uav_name) return
    setBusy(true)
    try {
      const result = await spawnVehicle(nextVehicleExample.uav_name, spawnDraft.world_origin, mode === 'mock')
      await refreshFleetNow()
      setSpawnDraft(null)
      setNotice({ type: 'ok', text: `${nextVehicleExample.uav_name}：${result.message}` })
    } catch (error) { setNotice({ type: 'error', text: error.message }) } finally { setBusy(false) }
  }
  const handleRemoveVehicle = async (vehicle) => {
    if (!window.confirm(`确认删除 ${vehicle.uav_name} 的${mode === 'mock' ? '飞机示例' : '完整仿真实例'}？`)) return
    setBusy(true)
    try {
      const result = await removeVehicle(vehicle.uav_name, mode === 'mock')
      setRoutes((current) => current.filter((route) => route.uav_name !== vehicle.uav_name))
      markEdited()
      await refreshFleetNow()
      setNotice({ type: 'ok', text: `${vehicle.uav_name}：${result.message}` })
    } catch (error) { setNotice({ type: 'error', text: error.message }) } finally { setBusy(false) }
  }
  const handleDispatch = async () => {
    setBusy(true)
    setNotice({ type: 'info', text: mode === 'mock' ? '正在检查任务结构与航迹间隔…' : '正在下发任务并等待各机地形规划…' })
    try {
      const currentFleet = await getFleet(mode === 'mock')
      const highestGoal = Math.max(0, ...routes.map((route) => Number(route.goal_id) || 0))
      const firstGoal = highestGoal + routes.length <= 4294967295 ? highestGoal + 1 : 1
      const preparedRoutes = await Promise.all(routes.map(async (route, index) => ({
        ...route,
        goal_id: firstGoal + index,
        points: [await currentRouteStart(route.uav_name, currentFleet), ...route.points.slice(1)],
      })))
      const result = await dispatchSwarm(preparedRoutes, mode === 'mock')
      setFleet(currentFleet)
      setRoutes(preparedRoutes)
      setSnapshot(result)
      setHasDispatched(true)
      setNotice({ type: 'ok', text: mode === 'mock' ? '演示检查完成；航线为任务回显，不代表真实规划结果' : `已向 ${result.dispatched?.length || routes.length} 架飞机下发任务，等待统一安全检查` })
    } catch (error) { setNotice({ type: 'error', text: error.message }) } finally { setBusy(false) }
  }
  const handleTakeoff = async () => {
    const names = routes.map((route) => route.uav_name)
    if (!window.confirm(`确认让 ${names.join('、')} 解锁并起飞？全部飞机离地后才会自动放行规划航迹。`)) return
    setBusy(true)
    try {
      const result = await takeoffFleet(names, takeoffAltitude, mode === 'mock')
      setNotice({ type: 'ok', text: result.message })
      await refreshFleetNow()
    } catch (error) { setNotice({ type: 'error', text: error.message }) } finally { setBusy(false) }
  }
  const handleLand = async () => {
    // Do not trust one browser poll's armed flags: sequential takeoff can make
    // that snapshot omit an aircraft which is already flying. The backend
    // uses the latest fleet state to skip aircraft already on the ground.
    const names = vehicles
      .filter((vehicle) => Number(vehicle.lifecycle) === 1)
      .map((vehicle) => vehicle.uav_name)
    if (!names.length) return
    if (!window.confirm(`确认让 ${names.join('、')} 进入安全降落？固定翼会先继续飞行建立进近，最终落地并自动上锁。`)) return
    setBusy(true)
    try {
      const result = await landFleet(names, mode === 'mock')
      setNotice({ type: 'ok', text: result.message })
      await refreshFleetNow()
    } catch (error) { setNotice({ type: 'error', text: error.message }) } finally { setBusy(false) }
  }
  const handleRestart = async () => {
    const action = mode === 'mock' ? '清空全部离线飞机和任务' : '只重启 ROS、PX4 与 Gazebo 执行系统，Web 保持运行'
    if (!window.confirm(`确认${action}？`)) return
    setBusy(true)
    try {
      const result = await restartSwarm(mode === 'mock')
      if (!result.restarting) {
        setRoutes([])
        setSnapshot(null)
        setHasDispatched(false)
        setSpawnDraft(null)
        await refreshFleetNow()
        setNotice({ type: 'ok', text: result.message })
        return
      }
      setRestarting(true)
      setNotice({ type: 'info', text: result.message })
      const started = Date.now()
      let sawUnavailable = false
      const waitForRecovery = async () => {
        while (Date.now() - started < 120000) {
          try {
            await getFleet(false)
            if (sawUnavailable || Date.now() - started > 20000) {
              window.location.reload()
              return
            }
          } catch (_error) {
            sawUnavailable = true
          }
          await new Promise((resolve) => window.setTimeout(resolve, 1000))
        }
        setRestarting(false)
        setNotice({ type: 'error', text: '重启等待超时，请用终端 status/logs 检查后手动刷新' })
      }
      void waitForRecovery()
    } catch (error) { setNotice({ type: 'error', text: error.message }) } finally { setBusy(false) }
  }

  return <div className="swarm-shell">
    <header className="topbar">
      <div><div className="eyebrow">XD-UAV / FIXED-WING MISSION CONTROL</div><h1>固定翼集群任务控制台</h1></div>
      <div className="topbar-actions"><a href="/">返回地图与规划试验</a><span className={`runtime-pill ${runtime?.execution_available ? 'ready' : runtime?.rosbridge_reachable ? 'reachable' : ''}`}>{runtimeLabel}</span><span className={`mode-pill ${mode}`}>{mode === 'mock' ? '离线编排' : 'ROS 执行'}</span></div>
    </header>
    <main className="workspace">
      <aside className="mission-panel">
        <section className="panel intro-panel">
          <div className="panel-kicker">任务规划与安全放行</div><h2>各机地形规划 · 集群统一检查</h2>
          <p>创建飞机前先在地图选定起始位置；每次下发任务时，起点自动同步为飞机当前位置并按 DEM 保持 80 m 净空。每架飞机独立规划，集群安全检查通过后才允许起飞。</p>
          <div className="contract-row"><span>航线格式</span><strong>{contractSummary}</strong></div><div className="contract-row"><span>任务接口</span><strong>/&lt;uav&gt;/planning/task_path</strong></div>
        </section>
        <section className="panel">
          <div className="section-title"><span>任务去向</span><small>执行系统上线后也不会自动补发旧任务</small></div>
          <div className="segmented"><button className={mode === 'mock' ? 'active' : ''} onClick={() => switchMode('mock')}>离线编排</button><button disabled={!runtime?.rosbridge_reachable} className={mode === 'ros' ? 'active ros' : ''} onClick={() => switchMode('ros')}>ROS 执行</button></div>
          <p className="fine-print">{mode === 'mock' ? '可新建、编辑和校验任务；结果是 Mock 回显，不表示飞机已收到。' : 'ROS 执行系统可接收任务；有 Gazebo/PX4 或实机执行端时才会产生真实运动。'}</p>
        </section>
        <section className="panel fleet-panel">
          {pendingMissionDraft && <div className="imported-draft">
            <strong>已接收地图任务草稿</strong>
            <span>WGS84 / MSL · {pendingMissionDraft.draft.terrain?.source || '在线 DEM'}</span>
            <small>先创建 {nextVehicleExample?.uav_name || '下一架飞机'}，再为它新建任务；旧页面预览航线不会直接成为飞行航线。</small>
          </div>}
          <div className="section-title"><span>在线飞机</span><small>{vehicles.length} / {fleet?.capacity || 8}</small></div>
          <div className="spawn-controls">
            <button disabled={busy || !nextVehicleExample} onClick={() => {
              const next = !selectingSpawn
              setSelectingSpawn(next)
              if (next) setNotice({ type: 'info', text: `请在地图上点击 ${nextVehicleExample.uav_name} 的起始位置` })
            }}>{selectingSpawn ? '取消地图选点' : nextVehicleExample ? `选择 ${nextVehicleExample.uav_name} 起点` : '已达到示例容量'}</button>
            <button disabled={busy || !nextVehicleExample || spawnDraft?.uav_name !== nextVehicleExample.uav_name || vehicles.length >= (fleet?.capacity || 8)} onClick={handleAddVehicle}>{nextVehicleExample ? `创建 ${nextVehicleExample.uav_name}` : '无法继续创建'}</button>
            <div className="spawn-summary">{spawnDraft ? `待创建：${spawnDraft.uav_name} · X ${spawnDraft.world_origin.x.toFixed(1)} · Y ${spawnDraft.world_origin.y.toFixed(1)} · 地面 ${spawnDraft.world_origin.z.toFixed(1)} m` : '尚未选起点；不再使用默认出生位置'}</div>
          </div>
          <div className="fleet-list">{vehicles.map((vehicle) => {
            const hasTask = routes.some((route) => route.uav_name === vehicle.uav_name)
            return <div className="fleet-row" key={vehicle.uav_name}>
              <div><strong style={{ color: vehicleColor(vehicle.uav_name) }}>● {vehicle.uav_name}</strong><small>{FLEET_STATES[vehicle.lifecycle] || '未知'} · {vehicle.connected ? '已连接' : mode === 'mock' ? '演示' : '未连接'} · {vehicle.armed ? '已解锁' : '已上锁'}</small></div>
              <button disabled={hasTask} onClick={() => addTaskForVehicle(vehicle.uav_name)}>{hasTask ? '已有任务' : '新建任务'}</button>
              <button className="danger-ghost" title={mode === 'ros' ? 'Gazebo Classic 固定翼不支持热删除；使用下方安全重启统一清空' : ''} disabled={busy || mode === 'ros' || vehicle.armed || !vehicle.on_ground} onClick={() => handleRemoveVehicle(vehicle)}>{mode === 'ros' ? '随重启清除' : '删除飞机'}</button>
            </div>
          })}{!vehicles.length && <div className="empty-state">尚未发现在线飞机</div>}</div>
          <button className="restart-button" disabled={busy || restarting || !canRestart} onClick={handleRestart}>{mode === 'mock' ? '清空演示机队' : restarting ? '正在安全重启…' : '安全重启并清空全部飞机'}</button>
        </section>
        <section className="panel route-editor">
          <div className="section-title"><span>任务草稿</span><small>{routes.length} 架 · world / m</small></div>
          {routes.map((route, routeIndex) => <article className="route-card" style={{ '--route-color': vehicleColor(route.uav_name) }} key={route.uav_name}>
            <div className="route-head"><strong>{route.uav_name}</strong><span>任务编号 {route.goal_id}</span></div>
            <div className="route-tools"><button onClick={() => copyTask(routeIndex)}>复制给空闲飞机</button><button className="danger-ghost" onClick={() => deleteTask(routeIndex)}>仅删除任务</button></div>
            <div className="route-meta-grid"><NumberField label="任务编号" value={route.goal_id} onChange={(value) => updateRoute(routeIndex, 'goal_id', value)} /><NumberField label="标称空速 m/s" value={route.nominal_speed_mps} step={0.5} onChange={(value) => updateRoute(routeIndex, 'nominal_speed_mps', value)} /></div>
            {route.points.map((point, pointIndex) => {
              const last = route.points.length - 1
              const isStart = pointIndex === 0
              const pointLabel = pointIndex === 0 ? '起点' : pointIndex === last ? '终点' : `航点 ${pointIndex}`
              return <div className="point-block" key={pointIndex}>
                <div className="point-actions"><button disabled={isStart} className={selectedRoute === routeIndex && selectedPoint === pointIndex ? 'point-select selected' : 'point-select'} onClick={() => { setSelectedRoute(routeIndex); setSelectedPoint(pointIndex) }}>{isStart ? '起点 · 下发时自动同步当前位置' : `${pointLabel} · 地图点选`}</button>{pointIndex > 0 && pointIndex < last && <button className="mini-danger" onClick={() => removeWaypoint(routeIndex, pointIndex)}>删除航点</button>}</div>
                <div className="point-grid"><NumberField label="X" value={point.x} disabled={isStart} onChange={(value) => updatePoint(routeIndex, pointIndex, 'x', value)} /><NumberField label="Y" value={point.y} disabled={isStart} onChange={(value) => updatePoint(routeIndex, pointIndex, 'y', value)} /><NumberField label="Z 高度" value={point.z} disabled={isStart} onChange={(value) => updatePoint(routeIndex, pointIndex, 'z', value)} /></div>
                {pointIndex < last && <button className="add-waypoint" onClick={() => addWaypoint(routeIndex, pointIndex)}>＋ 在此段插入航点</button>}
              </div>
            })}
          </article>)}
          {!routes.length && <div className="empty-state">请从在线飞机清单新建任务</div>}
        </section>
        <div className={`notice ${notice.type}`}>{notice.text}</div>
        <div className="dispatch-row"><button className="dispatch-button" disabled={busy || routes.length < 2} onClick={handleDispatch}>{busy ? '处理中…' : mode === 'mock' ? '检查任务' : '下发并规划'}</button><button className="refresh-button" disabled={busy} onClick={async () => { try { await refreshFleetNow(); if (hasDispatched) setSnapshot(await getSwarmSnapshot(mode === 'mock')) } catch (error) { setNotice({ type: 'error', text: error.message }) } }}>刷新状态</button></div>
        <section className="flight-actions"><NumberField label="相对起飞高度 m" value={takeoffAltitude} step={5} onChange={setTakeoffAltitude} /><button className="takeoff-button" disabled={busy || !canTakeoff} onClick={handleTakeoff}>一键解锁并起飞</button><button className="land-button" disabled={busy || !vehicles.length} onClick={handleLand}>一键安全降落</button><small>固定翼收到降落指令后会继续飞行建立进近，以“已落地、未解锁”为完成；空间交叉但错峰到达仅提示，预测同时接近仍禁止起飞。</small></section>
      </aside>
      <section className="visual-panel">
        <div className="map-wrap">
          {visualMode === '2d'
            ? <SwarmMap terrain={terrain} snapshot={snapshot} previewRoutes={previewRoutes} fleet={fleet} spawnDraft={spawnDraft} onPick={handleMapPick} />
            : <SwarmThreeD raster={terrainRaster} snapshot={snapshot} previewRoutes={previewRoutes} fleet={fleet} mode={mode} />}
          <div className="visual-toggle"><button className={visualMode === '2d' ? 'active' : ''} onClick={() => switchVisualMode('2d')}>二维地图</button><button className={visualMode === '3d' ? 'active' : ''} onClick={() => switchVisualMode('3d')}>三维实况</button></div>
          {vehicles.length > 0 && <div className="map-vehicle-legend">{vehicles.map((vehicle) => <span key={vehicle.uav_name}><i style={{ '--vehicle-color': vehicleColor(vehicle.uav_name) }} />{vehicle.uav_name}</span>)}</div>}
          <div className="map-caption"><span>Davosdorf DEM</span><span>{visualMode === '2d' ? '规划线 + 实际飞机与尾迹' : 'world / MSL 三维实况'}</span><span className={terrain?.canonical_dem_match ? 'verified' : 'unverified'}>{terrain?.canonical_dem_match ? 'DEM 已与 ROS 核对' : 'DEM 待核对'}</span></div>
        </div>
        <div className="telemetry-grid">
          <section className={`status-card ${statusTone}`}><div className="status-orb" /><div><span className="card-label">集群放行检查</span><strong>{!status ? '等待任务' : !status.analysis_ready ? '规划未齐' : !status.conflict_free ? `发现 ${blockingConflicts.length || conflicts.length} 个阻断冲突` : spatialWarnings.length ? `可放行 · ${spatialWarnings.length} 个空间交叉提醒` : '航迹可安全放行'}</strong><small>{status?.detail || '下发至少两架飞机的任务后生成结果'}</small></div></section>
          <section className="vehicles-card"><div className="card-label">规划与飞行状态</div><div className="vehicle-list">{(status?.vehicles || []).map((vehicle) => {
            const live = fleet?.runtime?.[vehicle.uav_name] || {}
            return <div className="vehicle-row" key={vehicle.uav_name}><span className="vehicle-name">{vehicle.uav_name}</span><span>{PLAN_STATES[vehicle.planner_status?.state] || '未知'}</span><span>{vehicle.path_points} 点</span><span>{Number(live.groundspeed_mps || 0).toFixed(1)} m/s</span></div>
          })}{!status?.vehicles?.length && <div className="empty-state">暂无规划结果</div>}</div></section>
          <section className="conflict-card"><div className="card-label">安全间隔与错峰提示</div><div className="conflict-list">{conflicts.map((conflict, index) => <div className="conflict-row" key={`${conflict.first_uav}-${conflict.second_uav}-${index}`}><span className={`conflict-kind type-${conflict.conflict_type}`}>{conflict.conflict_type === 2 ? '时空阻断' : '空间提醒'}</span><strong>{conflict.first_uav} ↔ {conflict.second_uav}</strong><span>{Number(conflict.separation_m).toFixed(1)} / {Number(conflict.threshold_m).toFixed(1)} m</span><span>{conflict.predicted_time_valid ? `预计 T+${Number(conflict.predicted_time_s).toFixed(1)}s 同时接近` : '预计错峰通过'}</span></div>)}{status?.analysis_ready && !conflicts.length && <div className="empty-state safe-text">当前规划筛查未发现安全间隔问题</div>}{!status?.analysis_ready && <div className="empty-state">等待全部任务生成有效航迹</div>}</div></section>
          <section className="profile-card"><div className="card-label">规划航迹高度</div><ElevationStrip snapshot={snapshot} /></section>
        </div>
      </section>
    </main>
  </div>
}
