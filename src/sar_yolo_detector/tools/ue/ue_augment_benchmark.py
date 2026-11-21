"""Add an owned benchmark district to a Save-As copy; preserve the base level."""
import hashlib
import json
import math
from pathlib import Path
import unreal

ROOT=Path(__file__).resolve().parent
spec=json.loads((ROOT/'benchmark_scene.json').read_text())
base='/Game/AirGround/AirGroundRange';destination=spec['level_path']
editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
world=editor.get_editor_world()
if not destination.startswith('/Game/AirGround/Benchmarks/'):
    raise RuntimeError('Benchmark must use its own asset directory')
base_file=Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_content_dir()))/'AirGround/AirGroundRange.umap'
before=hashlib.sha256(base_file.read_bytes()).hexdigest()
if unreal.EditorAssetLibrary.does_asset_exist(destination):
    if not world.get_path_name().startswith(destination+'.'):
        if not unreal.EditorLevelLibrary.load_level(destination):raise RuntimeError('Cannot load benchmark copy')
elif world.get_path_name().startswith(base+'.'):
    unreal.EditorAssetLibrary.make_directory('/Game/AirGround/Benchmarks')
    if not unreal.EditorLoadingAndSavingUtils.save_map(world,destination):raise RuntimeError('Cannot Save-As benchmark copy')
    if not unreal.EditorLevelLibrary.load_level(destination):raise RuntimeError('Cannot switch to benchmark copy')
else:raise RuntimeError('Load the base map or existing benchmark copy first')
world=editor.get_editor_world();owned=unreal.Name('SAR_BENCHMARK_V1')
# Only the explicitly owned additions are regenerated; base AG_ actors survive.
old=[a for a in actors.get_all_level_actors() if owned in a.tags]
if old:actors.destroy_actors(old)


def material(name,low,high,roughness=.8):
    folder='/Game/AirGround/Benchmarks/Materials';path=folder+'/'+name
    if unreal.EditorAssetLibrary.does_asset_exist(path):return unreal.load_asset(path)
    unreal.EditorAssetLibrary.make_directory(folder)
    m=unreal.AssetToolsHelpers.get_asset_tools().create_asset(name,folder,unreal.Material,unreal.MaterialFactoryNew())
    a=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionConstant3Vector)
    b=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionConstant3Vector)
    a.set_editor_property('constant',unreal.LinearColor(*low,1));b.set_editor_property('constant',unreal.LinearColor(*high,1))
    noise=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionNoise)
    noise.set_editor_property('scale',.025)
    if name=='M_BM_Water':
        position=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionWorldPosition)
        clock=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionTime)
        speed=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionConstant3Vector)
        speed.set_editor_property('constant',unreal.LinearColor(0,20,0,1))
        mul=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionMultiply)
        add=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionAdd)
        for source,pin in [(clock,'A'),(speed,'B')]:unreal.MaterialEditingLibrary.connect_material_expressions(source,'',mul,pin)
        for source,pin in [(position,'A'),(mul,'B')]:unreal.MaterialEditingLibrary.connect_material_expressions(source,'',add,pin)
        unreal.MaterialEditingLibrary.connect_material_expressions(add,'',noise,'Position')
    blend=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionLinearInterpolate)
    for node,pin in [(a,'A'),(b,'B'),(noise,'Alpha')]:unreal.MaterialEditingLibrary.connect_material_expressions(node,'',blend,pin)
    unreal.MaterialEditingLibrary.connect_material_property(blend,'',unreal.MaterialProperty.MP_BASE_COLOR)
    r=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionConstant)
    r.set_editor_property('r',roughness)
    unreal.MaterialEditingLibrary.connect_material_property(r,'',unreal.MaterialProperty.MP_ROUGHNESS)
    unreal.MaterialEditingLibrary.recompile_material(m);return m

mats={
 'grass':material('M_BM_Grass',[.035,.09,.025],[.17,.25,.06]),
 'road':material('M_BM_Road',[.035,.04,.045],[.09,.10,.11]),
 'water':material('M_BM_Water',[.025,.07,.10],[.055,.15,.17],.18),
 'concrete':material('M_BM_Concrete',[.18,.18,.17],[.4,.39,.35]),
 'tree':material('M_BM_Tree',[.015,.04,.008],[.08,.17,.025]),
 'drone':material('M_BM_Drone',[.005,.006,.007],[.035,.04,.045])}


def spawn(label,position,size,mesh='/Engine/BasicShapes/Cube.Cube',mat=None,parent=None):
    asset=unreal.load_asset(mesh)
    if not asset:raise RuntimeError('Missing benchmark asset '+mesh)
    v=unreal.Vector(position[0]*100,position[1]*100,-position[2]*100)
    actor=actors.spawn_actor_from_class(unreal.StaticMeshActor,v);actor.set_actor_label(label)
    c=actor.static_mesh_component;c.set_mobility(unreal.ComponentMobility.MOVABLE);c.set_static_mesh(asset)
    if mat:c.set_material(0,mat)
    b=asset.get_bounds();extent=[b.box_extent.x*2,b.box_extent.y*2,b.box_extent.z*2]
    scale=unreal.Vector(*[s*100/e for s,e in zip(size,extent)]);actor.set_actor_scale3d(scale)
    origin=b.origin;actor.set_actor_location(v-unreal.Vector(origin.x*scale.x,origin.y*scale.y,origin.z*scale.z),False,False)
    actor.tags=[owned,unreal.Name('AirGroundRange')]
    if parent:actor.attach_to_actor(parent,unreal.Name(''),unreal.AttachmentRule.KEEP_WORLD,unreal.AttachmentRule.KEEP_WORLD,unreal.AttachmentRule.KEEP_WORLD,False)
    return actor

spawn('AG_BM_Grass',[230,150,-.02],[190,260,.03],mat=mats['grass'])
spawn('AG_BM_Road',[220,150,-.07],[10,220,.05],mat=mats['road'])
spawn('AG_BM_OcclusionRoad',[320,150,-.07],[10,200,.05],mat=mats['road'])
spawn('AG_BM_Water',[260,150,-.06],[28,220,.05],mat=mats['water'])
for index,y in enumerate(range(55,250,12)):
    spawn('AG_BM_LaneMark_%02d'%index,[220,y,-.105],[.15,5,.02],mat=mats['concrete'])
for index,y in enumerate([130,170,210]):
    width=[6,12,24][index]
    spawn('AG_BM_Occluder_%d'%index,[320,y,-7],[12,width,1],mat=mats['concrete'])
    for side in [-1,1]:spawn('AG_BM_Pillar_%d_%d'%(index,side),[320+side*5.5,y,-3.5],[1,1,7],mat=mats['concrete'])
for i in range(14):
    x=239+(i%2)*4;y=65+i*12
    spawn('AG_BM_Trunk_%02d'%i,[x,y,-3],[.5,.5,6],'/Engine/BasicShapes/Cylinder.Cylinder',mats['concrete'])
    spawn('AG_BM_Canopy_%02d'%i,[x,y,-7],[7,7,6],'/Engine/BasicShapes/Sphere.Sphere',mats['tree'])
for i in range(6):
    spawn('AG_BM_Building_%02d'%i,[295,65+i*34,-(6+i%3*3)],[18,22,12+i%3*6],mat=mats['concrete'])
for entity in spec['entities']:
    if not entity['id'].startswith('AG_BM_'):continue
    mesh=entity.get('mesh');root=spawn(entity['id'],entity['position'],entity['size'] if mesh else [.35,.22,.14],
        mesh or '/Engine/BasicShapes/Cube.Cube',None if mesh else mats['drone'])
    root.tags=list(root.tags)+[unreal.Name('class:'+entity['class'])]
    root.set_actor_rotation(unreal.Rotator(pitch=0,yaw=entity.get('yaw_deg',0),roll=0),False)
    if not mesh:
        for i,angle in enumerate([45,-45]):
            arm=spawn(entity['id']+'_arm_'+str(i),entity['position'],[1.7,.06,.05],mat=mats['drone'],parent=root)
            arm.set_actor_rotation(unreal.Rotator(pitch=0,yaw=angle,roll=0),False)
        for i,(x,y) in enumerate([(-.6,-.6),(-.6,.6),(.6,-.6),(.6,.6)]):
            p=entity['position'];spawn(entity['id']+'_rotor_'+str(i),[p[0]+x,p[1]+y,p[2]-.3],[.4,.4,.04],
                '/Engine/BasicShapes/Cylinder.Cylinder',mats['drone'],root)
if not world.get_path_name().startswith(destination+'.'):raise RuntimeError('Refusing to save additions into base level')
if not unreal.EditorLevelLibrary.save_current_level():raise RuntimeError('Cannot save augmented benchmark map')
unreal.EditorAssetLibrary.save_directory('/Game/AirGround/Benchmarks',only_if_is_dirty=True,recursive=True)
after=hashlib.sha256(base_file.read_bytes()).hexdigest()
if before!=after:raise RuntimeError('Base map changed unexpectedly')
result=dict(passed=True,level=destination,base_level=base,base_sha256_before=before,base_sha256_after=after,
 generated_actors=sum(owned in a.tags for a in actors.get_all_level_actors()),
 zones=['textured_grass','road_and_lane_marks','visual_water_surface','three_overhead_occluders','tree_canopies','buildings','moving_targets_and_decoys'],
 water_model='procedural RGB surface, no hydrodynamics or infrared sensor model')
(ROOT/'benchmark_build_result.json').write_text(json.dumps(result,indent=2));unreal.log('Benchmark district saved: '+destination)
