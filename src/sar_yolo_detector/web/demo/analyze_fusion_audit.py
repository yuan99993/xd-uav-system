#!/usr/bin/env python3
"""Align recorded state, camera and evidence by original capture timestamp."""
import argparse
import bisect
import collections
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',help='Directory created by capture_fusion_audit.py')
    parser.add_argument('--tolerance-ms',type=float,default=1.)
    args=parser.parse_args()
    if args.tolerance_ms<0:parser.error('--tolerance-ms cannot be negative')
    directory=Path(args.directory);channels=collections.defaultdict(list)
    with (directory/'messages.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            item=json.loads(line)
            if item.get('capture_stamp') is not None:
                channels[item['channel']].append(item)
    for values in channels.values():values.sort(key=lambda item:item['capture_stamp'])
    camera=channels['camera_image'];evidence=channels['fusion_evidence']
    camera_times=[x['capture_stamp'] for x in camera];evidence_times=[x['capture_stamp'] for x in evidence]
    tolerance=args.tolerance_ms/1000.
    def nearest(values,times,stamp):
        i=bisect.bisect_left(times,stamp)
        choices=values[max(0,i-1):min(len(values),i+1)]
        item=min(choices,key=lambda x:abs(x['capture_stamp']-stamp)) if choices else None
        return item if item and abs(item['capture_stamp']-stamp)<=tolerance else None
    counts=collections.Counter();previous=None
    with (directory/'alignment.jsonl').open('x',encoding='utf-8') as output:
        for state in channels['state']:
            stamp=state['capture_stamp'];camera_item=nearest(camera,camera_times,stamp);evidence_item=nearest(evidence,evidence_times,stamp)
            payload=state['payload'];current=payload['selected_id'] if payload['has_selection'] else -1
            changed=previous is not None and current!=previous;previous=current
            diagnostics=evidence_item['payload'].get('diagnostics',{}) if evidence_item else None
            veto=diagnostics.get('strong_takeover_vetoes',{}) if isinstance(diagnostics,dict) else None
            row={'capture_stamp':stamp,'selected_id':current,'selected_id_changed':changed,
                 'control_valid':payload['valid'],'camera_header_matched':camera_item is not None,
                 'fusion_evidence_matched':evidence_item is not None,
                 'evidence_selected_id':evidence_item['payload'].get('selected_id') if evidence_item else None,
                 'veto_reasons':veto,'missing_evidence':'unknown' if evidence_item is None else None}
            output.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
            counts['state_frames']+=1;counts['id_transitions']+=int(changed)
            counts['camera_matched']+=int(camera_item is not None);counts['evidence_matched']+=int(evidence_item is not None)
            counts['evidence_with_veto']+=int(bool(veto))
    summary={'counts':dict(counts),'tolerance_ms':args.tolerance_ms,
             'limitations':['Selection changes and vetoes are not proof of false locks.',
              'Missing evidence is unknown; fusion evidence may be rate limited.',
              'Nearest capture timestamp is a numerical alignment, not a truth label.']}
    (directory/'alignment_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary))


if __name__=='__main__':main()
