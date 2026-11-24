#!/usr/bin/env python3
"""Package project sources, documentation, archived evidence and dependency support."""
import hashlib,json,zipfile
from pathlib import Path

root=Path(__file__).resolve().parents[1]
out=root/'dist';out.mkdir(parents=True,exist_ok=True)
archive=out/'ros2-nav2-diffbot.zip'
files=[]
for directory in ['src','docs','tools','support','results','artifacts']:
 for p in (root/directory).rglob('*'):
  if not p.is_file():continue
  rel=p.relative_to(root)
  if p.is_relative_to(root/'artifacts/m8/delivery') or p.is_relative_to(root/'artifacts/m8/github_upload') or any(x in rel.parts for x in ['.git','__pycache__']):continue
  if p.is_symlink():raise RuntimeError('Unexpected symlink: '+str(rel))
  files.append(p)
for name in ['README.md','THIRD_PARTY_NOTICES.md','LICENSE','.gitignore','.gitattributes']:
 files.append(root/name)
files=sorted(set(files));manifest={str(p.relative_to(root)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in files:z.write(p,p.relative_to(root).as_posix())
 z.writestr('MANIFEST.sha256.json',json.dumps(manifest,indent=2))
with zipfile.ZipFile(archive) as z:
 assert z.testzip() is None
 assert set(z.namelist())==set(manifest)|{'MANIFEST.sha256.json'}
 assert all(hashlib.sha256(z.read(name)).hexdigest()==sha for name,sha in manifest.items())
digest=hashlib.sha256(archive.read_bytes()).hexdigest()
(out/'SHA256SUMS.txt').write_text(digest+'  '+archive.name+'\n',encoding='utf-8')
print(json.dumps({'archive':str(archive),'files':len(manifest),'bytes':archive.stat().st_size,'sha256':digest}))
