"""Refresh or verify the archived OSM water, station and bridge sources."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import urllib.request

SOURCE = Path(__file__).resolve().parent / 'env/skku_commute/landmarks_source'
QUERIES = {
    'features': '[out:json][timeout:60];(node[railway=station](37.28,126.94,37.60,127.04);way[water=river](37.50,126.96,37.54,127.02););out geom;',
    'water': '[out:json][timeout:60];relation(152336);out geom;',
    'bridges': '[out:json][timeout:60];nwr["man_made"="bridge"][name](37.50,126.96,37.54,127.02);out center tags;',
}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--refresh',action='store_true')
    args=parser.parse_args()
    SOURCE.mkdir(parents=True,exist_ok=True)
    for name,query in QUERIES.items():
        target=SOURCE/(name+'.json')
        if target.exists() and not args.refresh:
            data=json.loads(target.read_text())
            if data.get('remark') or not data.get('elements'):
                raise ValueError(f'Invalid cached OSM response: {target}')
            print(f'Reuse {target}')
            continue
        url='https://overpass-api.de/api/interpreter'
        request=urllib.request.Request(url,data=query.encode(),headers={'Content-Type':'text/plain','User-Agent':'SKKU-SUMO/1.0'})
        with urllib.request.urlopen(request,timeout=90) as response:
            body=response.read()
        data=json.loads(body)
        if data.get('remark') or not data.get('elements'):
            raise ValueError(f'Incomplete OSM response: {data.get("remark")}')
        temporary=target.with_suffix('.part');temporary.write_bytes(body);temporary.replace(target)
        metadata=SOURCE/('request.json' if name=='features' else name+'_request.json')
        metadata.write_text(json.dumps(dict(query=query,url=url,retrieved_utc=datetime.now(timezone.utc).isoformat(),
                                           sha256=hashlib.sha256(body).hexdigest()),ensure_ascii=False,indent=2)+'\n')
        print(f'Saved {target}')


if __name__=='__main__':main()
