"""Execute in Unreal Editor Python console with exec(open(path).read()).

Requires Python Editor Script Plugin + Editor Scripting Utilities and a
compatible Colosseum project. Scene root is configurable through AG_SCENE.
Creates geometry; does not install Unreal, plugins or third-party assets.
"""
import json
import os
from pathlib import Path
import unreal

scene_path = Path(os.environ.get("AG_SCENE", str(Path(__file__).with_name("scene.json"))))
spec = json.loads(scene_path.read_text())
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
level_path=spec.get("level_path","/Game/AirGroundRange")
if spec.get("create_level",False):
    exists=unreal.EditorAssetLibrary.does_asset_exist(level_path)
    active=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    already_loaded=active is not None and active.get_path_name().startswith(level_path+'.')
    ok=True if already_loaded else (unreal.EditorLevelLibrary.load_level(level_path) if exists else unreal.EditorLevelLibrary.new_level(level_path))
    if not ok:
        raise RuntimeError("Could not create owned simulation level: "+level_path)
existing = [a for a in actors.get_all_level_actors() if str(a.get_actor_label()).startswith("AG_")]
if existing:
    if spec.get("create_level",False):
        actors.destroy_actors(existing)  # owned generated level, AG_ actors only
    else:
        raise RuntimeError("AG_ scene already exists. Use a new level to preserve existing actors.")
starts=[a for a in actors.get_all_level_actors() if isinstance(a,unreal.PlayerStart)]
if len(starts)>1:
    raise RuntimeError("Use a level with at most one PlayerStart to establish an unambiguous NED origin")
start=starts[0] if starts else actors.spawn_actor_from_class(unreal.PlayerStart,unreal.Vector(0,0,0))
start.set_actor_location(unreal.Vector(0,0,0),False,False)
start.set_actor_rotation(unreal.Rotator(0,0,0),False)


def vector(values):
    # NED meters -> Unreal centimeters, UE world Z positive upwards.
    return unreal.Vector(values[0]*100, values[1]*100, -values[2]*100)


def solid_material(name,color):
    path="/Game/AirGround/Materials/"+name
    if unreal.EditorAssetLibrary.does_asset_exist(path):return unreal.load_asset(path)
    material=unreal.AssetToolsHelpers.get_asset_tools().create_asset(name,"/Game/AirGround/Materials",unreal.Material,unreal.MaterialFactoryNew())
    node=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector)
    node.set_editor_property("constant",unreal.LinearColor(color[0],color[1],color[2],1.))
    unreal.MaterialEditingLibrary.connect_material_property(node,"",unreal.MaterialProperty.MP_BASE_COLOR)
    unreal.MaterialEditingLibrary.recompile_material(material)
    return material


grass=solid_material("M_AG_Grass",[.14,.23,.07])
road=solid_material("M_AG_Road",[.07,.075,.08])
drone_material=solid_material("M_AG_Drone",[.015,.018,.025])


def spawn(label, position, size, mesh_path="/Engine/BasicShapes/Cube.Cube", parent=None):
    mesh = unreal.load_asset(mesh_path)
    if not mesh:
        raise RuntimeError("Mesh missing: " + mesh_path)
    a = actors.spawn_actor_from_class(unreal.StaticMeshActor, vector(position))
    a.set_actor_label(label)
    if spec.get("require_airsim",True):
        a.rename(label)
    c = a.static_mesh_component
    c.set_mobility(unreal.ComponentMobility.MOVABLE)
    c.set_static_mesh(mesh)
    if label=="AG_Ground":c.set_material(0,grass)
    elif label=="AG_Road":c.set_material(0,road)
    elif "drone" in label.lower():c.set_material(0,drone_material)
    # Built-in meshes are 100 cm. Asset meshes are scaled by their actual bounds.
    bounds = mesh.get_bounds().box_extent
    extents = [bounds.x*2, bounds.y*2, bounds.z*2]
    if min(extents) <= 0:
        raise RuntimeError("Mesh has degenerate bounds")
    a.set_actor_scale3d(unreal.Vector(*[size[i]*100/extents[i] for i in range(3)]))
    # Imported meshes often use a ground pivot. Keep the actor's visible
    # bounds centred at the scene's declared body position.
    origin=mesh.get_bounds().origin
    scale=a.get_actor_scale3d()
    target=vector(position)
    a.set_actor_location(unreal.Vector(target.x-origin.x*scale.x,target.y-origin.y*scale.y,
                                      target.z-origin.z*scale.z),False,False)
    a.tags = [unreal.Name(label), unreal.Name("AirGroundRange")]
    if parent:
        a.attach_to_actor(parent, unreal.Name(""), unreal.AttachmentRule.KEEP_WORLD,
                          unreal.AttachmentRule.KEEP_WORLD, unreal.AttachmentRule.KEEP_WORLD, False)
    return a


spawn("AG_Ground", [0,0,0.5], [10000,10000,1])
spawn("AG_Road", [0,0,-0.03], [180,10,0.05])
spawn("AG_Occluder_01", [5,5,-4], [10,8,8])
spawn("AG_Occluder_02", [-45,20,-5], [10,15,10])
for e in spec["entities"]:
    pos, size = e["position"], e["size"]
    asset = e.get("mesh") or spec.get("asset_meshes", {}).get(e["class"])
    body_size=[.35,.22,.14] if e["class"]=="drone" and not asset else size
    root = spawn(e["id"],pos,body_size,asset or "/Engine/BasicShapes/Cube.Cube")
    root.tags = list(root.tags) + [unreal.Name("class:"+e["class"])]
    if asset:
        continue
    if e["class"] == "drone":
        for n,angle in enumerate([45,-45]):
            arm=spawn(e["id"]+"_arm_"+str(n),pos,[1.7,.06,.05],parent=root)
            arm.set_actor_rotation(unreal.Rotator(pitch=0.,yaw=angle,roll=0.),False)
        # Four distinguishable rotors attached to one moving actor root.
        for i,(x,y) in enumerate([(-.6,-.6),(-.6,.6),(.6,-.6),(.6,.6)]):
            spawn(e["id"]+"_rotor_"+str(i),[pos[0]+x,pos[1]+y,pos[2]-.3],
                  [.4,.4,.04],"/Engine/BasicShapes/Cylinder.Cylinder",root)
    else:
        for i,(x,y) in enumerate([(-size[0]*.3,-size[1]*.45),(-size[0]*.3,size[1]*.45),
                                   (size[0]*.3,-size[1]*.45),(size[0]*.3,size[1]*.45)]):
            spawn(e["id"]+"_wheel_"+str(i),[pos[0]+x,pos[1]+y,-.45],
                  [.7,.35,.7],"/Engine/BasicShapes/Sphere.Sphere",root)
        if e["class"] in {"tank","m142"}:
            spawn(e["id"]+"_upper",[pos[0],pos[1],pos[2]-size[2]*.6],
                  [size[0]*.6,size[1]*.6,size[2]*.4],parent=root)

for light_class,label,location in [(unreal.DirectionalLight,"AG_Sun",[0,0,10000]),
                                  (unreal.SkyLight,"AG_SkyLight",[0,0,0]),
                                  (unreal.SkyAtmosphere,"AG_Atmosphere",[0,0,0])]:
    actor=actors.spawn_actor_from_class(light_class,unreal.Vector(*location))
    actor.set_actor_label(label)
    if light_class==unreal.DirectionalLight:
        actor.set_actor_rotation(unreal.Rotator(pitch=-45.,yaw=-25.,roll=0.),False)
        actor.light_component.set_editor_property("intensity",5.)
        actor.light_component.set_editor_property("atmosphere_sun_light",True)
    elif light_class==unreal.SkyLight:
        component=actor.get_component_by_class(unreal.SkyLightComponent)
        component.set_mobility(unreal.ComponentMobility.MOVABLE)
        component.set_editor_property('source_type',unreal.SkyLightSourceType.SLS_SPECIFIED_CUBEMAP)
        component.set_cubemap(unreal.load_asset('/Engine/MapTemplates/Sky/DaylightAmbientCubemap.DaylightAmbientCubemap'))
        component.set_intensity(1.5)
# Soft fill lights keep PBR surfaces readable without enabling expensive GI.
for index,yaw in enumerate([135.,225.]):
    fill=actors.spawn_actor_from_class(unreal.DirectionalLight,unreal.Vector(0,0,8000))
    fill.set_actor_label('AG_Fill_'+str(index))
    fill.set_actor_rotation(unreal.Rotator(pitch=-50.,yaw=yaw,roll=0.),False)
    fill.light_component.set_editor_property('intensity',1.5)
    fill.light_component.set_editor_property('cast_shadows',False)
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
settings = world.get_world_settings()
game_mode = unreal.load_class(None,"/Script/AirSim.AirSimGameMode")
if game_mode:
    settings.set_editor_property("default_game_mode",game_mode)
elif spec.get("require_airsim",False):
    raise RuntimeError("AirSim/Colosseum game mode unavailable")
for name in (["AG_ObserverCamera","AG_FrontCamera"] if spec.get("spawn_cameras",False) else []):
    camera=actors.spawn_actor_from_class(unreal.CameraActor,vector(spec["observer"]["route"][0]))
    camera.set_actor_label(name)
    camera.camera_component.set_editor_property("field_of_view",90.)
if not spec.get('require_airsim',True):
    for key in ['down','front']:
        path='/Game/AirGround/Cameras/RT_AG_'+key
        target=unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else unreal.AssetToolsHelpers.get_asset_tools().create_asset(
            'RT_AG_'+key,'/Game/AirGround/Cameras',unreal.TextureRenderTarget2D,unreal.TextureRenderTargetFactoryNew())
        target.set_editor_property('size_x',960);target.set_editor_property('size_y',540)
        target.set_editor_property('render_target_format',unreal.TextureRenderTargetFormat.RTF_RGBA8)
        capture=actors.spawn_actor_from_class(unreal.SceneCapture2D,vector(spec['observer']['route'][0]))
        capture.set_actor_label('AG_Capture_'+key)
        component=capture.get_component_by_class(unreal.SceneCaptureComponent2D)
        component.set_editor_property('texture_target',target)
        component.set_editor_property('capture_every_frame',False)
        component.set_editor_property('capture_on_movement',False)
        component.set_editor_property('fov_angle',90.)
if not unreal.EditorLoadingAndSavingUtils.save_map(world,level_path):
    raise RuntimeError("Could not save generated map")
unreal.EditorAssetLibrary.save_directory("/Game/AirGround",only_if_is_dirty=True,recursive=True)
unreal.log("Saved "+level_path+". Free/placeholder assets require domain validation before recognition acceptance.")
Path(__file__).with_name("build_result.json").write_text(json.dumps({"level":level_path,
    "entities":[{"id":e["id"],"class":e["class"],"mesh":e.get("mesh") or spec.get("asset_meshes",{}).get(e["class"]),
                 "placeholder":not bool(e.get("mesh") or spec.get("asset_meshes",{}).get(e["class"]))} for e in spec["entities"]],
    "generated_actor_count":sum(str(a.get_actor_label()).startswith("AG_") for a in actors.get_all_level_actors())},indent=2))
