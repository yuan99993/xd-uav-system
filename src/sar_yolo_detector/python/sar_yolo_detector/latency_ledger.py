"""Capture-identity joins and additive latency accounting, never percentile subtraction."""
import math
from collections import defaultdict
BOUNDARIES=('capture_submit_wall','readback_complete_wall','shm_write_complete_wall',
            'tracker_wake_wall','vision_complete_wall','control_publish_wall')
SEGMENTS=('source_readback_ms','readback_to_shm_ms','shm_to_tracker_ms',
          'tracker_vision_ms','vision_to_control_ms')

def breakdown(timestamps):
    values=[timestamps.get(name) for name in BOUNDARIES]
    if any(not isinstance(v,(float,int)) or not math.isfinite(v) for v in values):
        return dict(valid=False,reason='missing_or_nonfinite_boundary')
    deltas=[(b-a)*1000. for a,b in zip(values,values[1:])]
    if min(deltas)<0:return dict(valid=False,reason='nonmonotonic_clock_or_wrong_capture',raw_segments_ms=dict(zip(SEGMENTS,deltas)))
    return dict(valid=True,segments_ms=dict(zip(SEGMENTS,deltas)),
                total_ms=(values[-1]-values[0])*1000.,closure_error_ms=sum(deltas)-(values[-1]-values[0])*1000.)

def join_controls(controls,events):
    by_capture=defaultdict(list)
    for event in events:by_capture[event['capture_ns']].append(event)
    for values in by_capture.values():values.sort(key=lambda x:x['node_state_publish_wall'])
    output=[]
    for control in controls:
        row=dict(control)
        key=control.get('capture_ns',int(round(control['capture_stamp']*1e9)))
        candidates=[e for e in by_capture.get(key,()) if e['node_state_publish_wall']<=control['control_output_wall']]
        candidates=[e for e in candidates if 'state_processing_ms' not in e or abs(e['state_processing_ms']-control['frame_processing_ms'])<1e-6]
        if candidates:
            event=candidates[-1]
            times=dict(event['timestamps'],control_publish_wall=control['control_output_wall'])
            row.update(latency_timestamps=times,latency_ledger=breakdown(times),node_timing=event)
        else:row['latency_ledger']=dict(valid=False,reason='no_matching_node_event')
        output.append(row)
    return output
