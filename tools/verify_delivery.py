#!/usr/bin/env python3
"""Verify every archived file; optionally extract into a NEW directory."""
import argparse,hashlib,json,zipfile
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__);p.add_argument('archive',type=Path)
p.add_argument('--extract-to',type=Path);args=p.parse_args()
with zipfile.ZipFile(args.archive) as z:
 manifest=json.loads(z.read('MANIFEST.sha256.json'))
 assert len(z.namelist())==len(set(z.namelist())), 'Duplicate archive names'
 assert set(z.namelist())==set(manifest)|{'MANIFEST.sha256.json'}
 for name,sha in manifest.items():
  assert not name.startswith('/') and '..' not in Path(name).parts and '\\' not in name and ':' not in name
  assert hashlib.sha256(z.read(name)).hexdigest()==sha, 'Hash mismatch: '+name
 if args.extract_to:
  target=args.extract_to.resolve();assert not target.exists(),'Extraction destination must be new'
  assert all((target/name).resolve().is_relative_to(target) for name in z.namelist())
  target.mkdir(parents=True);z.extractall(target)
print(json.dumps({'verified_files':len(manifest),'sha256':hashlib.sha256(args.archive.read_bytes()).hexdigest(),'extracted_to':str(args.extract_to) if args.extract_to else None}))
