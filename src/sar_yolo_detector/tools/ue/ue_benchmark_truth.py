"""Evaluation-only projection log, outside RGB transport and perception topics.

Boxes are projected 3-D world AABBs, NOT segmentation masks or detection GT.
Visibility is a centre collision-ray proxy, explicitly labelled as such.
"""
import json
import math
from pathlib import Path
import unreal
from benchmark_visibility import trace_blocked
from benchmark_truth_writer import TruthWriter

_STREAMS={}


def record(root,camera,actors,request,metadata):
    session=request['session'];folder=Path(root)/'BenchmarkTruth';folder.mkdir(exist_ok=True)
    stream=_STREAMS.get(session)
    if stream is None:
        stream=TruthWriter(folder/(session+'.jsonl'));_STREAMS[session]=stream
    w,h=metadata['width'],metadata['height'];f=w/(2*math.tan(math.radians(metadata['horizontal_fov_deg'])/2))
    location=camera.get_actor_location();forward=camera.get_actor_forward_vector();right=camera.get_actor_right_vector();up=camera.get_actor_up_vector()
    def project(point):
        v=point-location;depth=v.x*forward.x+v.y*forward.y+v.z*forward.z
        return None if depth<=1 else [w/2+f*(v.x*right.x+v.y*right.y+v.z*right.z)/depth,h/2-f*(v.x*up.x+v.y*up.y+v.z*up.z)/depth]
    items=[]
    for name in request['objects']:
        if not name.startswith('AG_BM_') or name not in actors:continue
        actor=actors[name];parts=[actor]+list(actor.get_attached_actors())
        lower=[float('inf')]*3;upper=[-float('inf')]*3
        for part in parts:
            c,e=part.get_actor_bounds(False)
            for i,(v,r) in enumerate(zip([c.x,c.y,c.z],[e.x,e.y,e.z])):
                lower[i]=min(lower[i],v-r);upper[i]=max(upper[i],v+r)
        centre=unreal.Vector(*[(a+b)/2 for a,b in zip(lower,upper)])
        pixels=[]
        for x in [lower[0],upper[0]]:
            for y in [lower[1],upper[1]]:
                for z in [lower[2],upper[2]]:
                    p=project(unreal.Vector(x,y,z))
                    if p is not None:pixels.append(p)
        point=project(centre);box=None
        if len(pixels)==8:
            xs=[p[0] for p in pixels];ys=[p[1] for p in pixels]
            box=[min(xs),min(ys),max(xs)-min(xs),max(ys)-min(ys)]
        visibility='unknown';error=None
        try:
            hit_result=unreal.SystemLibrary.line_trace_single(camera,location,centre,
                unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,True,[camera]+parts,unreal.DrawDebugTrace.NONE,True)
            visibility='centre_ray_blocked' if trace_blocked(hit_result) else 'centre_ray_clear'
        except Exception as exc:error=str(exc)
        items.append(dict(entity_id=name,centre_world_ned_m=[centre.x/100,centre.y/100,-centre.z/100],centre_px=point,projected_world_aabb=box,
            on_screen=point is not None and 0<=point[0]<w and 0<=point[1]<h,
            visibility_proxy=visibility,visibility_error=error))
    stream.write(json.dumps(dict(capture_wall_start=metadata['capture_wall_start'],frame_seq=request['frame_seq'],
        width=w,height=h,objects=items,semantics='evaluation_only_projected_3d_aabb_and_centre_collision_ray_not_pixel_masks'))+'\n')


def close(session):
    stream=_STREAMS.pop(session,None)
    if stream:stream.close()
