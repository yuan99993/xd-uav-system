"""Owned Save-As benchmark variant: flat grass and vivid vehicle body paint."""
import hashlib
from pathlib import Path
import unreal

DESTINATION='/Game/AirGround/Benchmarks/EOIRPlainGrassBrightCars_20261123'
MATERIALS='/Game/AirGround/Benchmarks/EOIRPlainGrassBrightCars/Materials'

def create_visual_variant():
    editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
    actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if editor.get_game_world() is not None:raise RuntimeError('Stop Play before changing benchmark materials')
    world=editor.get_editor_world();original=world.get_path_name().split('.')[0]
    if original!='/Game/AirGround/Benchmarks/AirGroundStressV1':
        raise RuntimeError('Visual variant must start from the original stress map')
    if unreal.EditorAssetLibrary.does_asset_exist(DESTINATION):
        raise RuntimeError('Visual variant already exists; refusing to overwrite')
    content=Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_content_dir()))
    original_file=content/(original.removeprefix('/Game/')+'.umap')
    before=hashlib.sha256(original_file.read_bytes()).hexdigest()
    if not unreal.EditorLoadingAndSavingUtils.save_map(world,DESTINATION):raise RuntimeError('Cannot Save-As visual variant')
    world=editor.get_editor_world()
    if not world.get_path_name().startswith(DESTINATION+'.'):
        if not unreal.EditorLoadingAndSavingUtils.load_map(DESTINATION):raise RuntimeError('Cannot load copied map')
    unreal.EditorAssetLibrary.make_directory(MATERIALS)
    def solid(name,color,roughness):
        material=unreal.AssetToolsHelpers.get_asset_tools().create_asset(name,MATERIALS,unreal.Material,unreal.MaterialFactoryNew())
        if not material:raise RuntimeError('Cannot create material '+name)
        node=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector)
        node.set_editor_property('constant',unreal.LinearColor(*color,1.))
        unreal.MaterialEditingLibrary.connect_material_property(node,'',unreal.MaterialProperty.MP_BASE_COLOR)
        for prop,value in [(unreal.MaterialProperty.MP_ROUGHNESS,roughness),(unreal.MaterialProperty.MP_SPECULAR,.08)]:
            scalar=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant)
            scalar.set_editor_property('r',value)
            unreal.MaterialEditingLibrary.connect_material_property(scalar,'',prop)
        unreal.MaterialEditingLibrary.recompile_material(material)
        unreal.EditorAssetLibrary.save_loaded_asset(material)
        return material
    grass_color=[.045,.28,.025]
    grass=solid('M_EOIR_PlainGrass',grass_color,1.)
    colors={'AG_BM_Car_Primary':[.95,.018,.005],
            'AG_BM_Car_Occlusion':[1.,.19,.005],
            'AG_BM_Car_Decoy':[.005,.62,.95]}
    paints={label:solid('M_EOIR_'+label,color,.38) for label,color in colors.items()}
    changed=[]
    for actor in actors.get_all_level_actors():
        label=str(actor.get_actor_label());component=actor.get_component_by_class(unreal.StaticMeshComponent)
        if component is None:continue
        if label in ('AG_Ground','AG_BM_Grass'):
            original_materials=[component.get_material(i).get_path_name() if component.get_material(i) else None for i in range(component.get_num_materials())]
            for i in range(component.get_num_materials()):component.set_material(i,grass)
            changed.append(dict(actor=label,kind='ground',rgb_linear=grass_color,original_materials=original_materials))
        elif label in paints:
            slots=[str(s) for s in component.get_material_slot_names()]
            names=[str(component.get_material(i).get_name()) if component.get_material(i) else '' for i in range(component.get_num_materials())]
            eligible=[]
            for i,name in enumerate(names):
                text=(slots[i]+' '+name).lower()
                if any(word in text for word in ('glass','window','wheel','tire','tyre','rubber','lamp','light','chrome','interior')):continue
                eligible.append(i)
            preferred=[i for i in eligible if any(word in (slots[i]+' '+names[i]).lower() for word in ('paint','body','color'))]
            chosen=preferred or eligible[:1]
            if not chosen:raise RuntimeError('No safe vehicle body slot found: '+label)
            for i in chosen:component.set_material(i,paints[label])
            changed.append(dict(actor=label,kind='car',rgb_linear=colors[label],slots=slots,
                                original_materials=names,painted_slots=chosen))
    cars={row['actor'] for row in changed if row['kind']=='car'}
    if cars!=set(colors) or not any(row['kind']=='ground' for row in changed):
        raise RuntimeError('Expected ground and all three benchmark cars')
    if not unreal.EditorLevelLibrary.save_current_level():raise RuntimeError('Could not save visual variant')
    after=hashlib.sha256(original_file.read_bytes()).hexdigest()
    if before!=after:raise RuntimeError('Original benchmark map changed')
    return dict(level=DESTINATION,base_level=original,base_sha256_before=before,base_sha256_after=after,
        changes=changed,roads_and_occluders_preserved=True,motion_and_scoring_thresholds_changed=False)
