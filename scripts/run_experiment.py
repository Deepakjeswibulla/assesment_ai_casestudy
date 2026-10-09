"""Reproducible run wrapper with optional lightweight comparison."""
import argparse, json, hashlib
from pathlib import Path
from src.pipeline import run

def compare(new_path,old_path):
 new=json.loads((new_path/'result.json').read_text()); old=json.loads((old_path/'result.json').read_text())
 def counts(x): return {'rooms':len(x.get('rooms',[])),'walls':sum(len(r.get('walls',[])) for r in x.get('rooms',[])),'openings':sum(len(r.get('openings',[])) for r in x.get('rooms',[]))}
 a,b=counts(new),counts(old)
 new_manifest=json.loads((new_path/'run_manifest.json').read_text());old_manifest=json.loads((old_path/'run_manifest.json').read_text())
 return {'compared_to':str(old_path),'new_counts':a,'old_counts':b,'count_deltas':{k:a[k]-b[k] for k in a},'result_hashes_equal':hashlib.sha256((new_path/'result.json').read_bytes()).hexdigest()==hashlib.sha256((old_path/'result.json').read_bytes()).hexdigest(),'configuration_hashes':{'new':new_manifest.get('configuration_sha256'),'old':old_manifest.get('configuration_sha256'),'changed':new_manifest.get('configuration_sha256')!=old_manifest.get('configuration_sha256')},'new_calibration_status':new.get('calibration',{}).get('status'),'old_calibration_status':old.get('calibration',{}).get('status')}

p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True);p.add_argument('--compare-to');a=p.parse_args();out=Path(a.output)
if out.exists() and any(out.iterdir()): raise SystemExit('Refusing to overwrite non-empty experiment output')
run(Path(a.input),out)
if a.compare_to:
 result=compare(out,Path(a.compare_to));(out/'comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
