import json
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile
from lxml import etree
import hashlib

HERE=Path(__file__).resolve().parent
THESIS=Path(r'D:/file_used/西柚/实验室_任务/硕士论文相关')
ROS=THESIS/'小论文/ros代码调试/1'
files=[ROS/'gazebo_world_sources/AlohaMini/simulation/src/Aloha/urdf/Aloha.urdf']
files += [ROS/'ros_motion_planning/src/sim_env/urdf/aloha_mini'/x for x in
 ['aloha_mini_mecanum_front_arm.urdf','aloha_mini_mecanum.urdf','aloha_mini.urdf']]
out=[]
for f in files:
    r=ET.parse(f).getroot()
    joints=[]
    for e in r.findall('joint'):
        item={'name':e.get('name'),'type':e.get('type')}
        for key in ['parent','child','origin','axis','limit','mimic']:
            c=e.find(key)
            if c is not None:item[key]=c.attrib
        joints.append(item)
    geometry=[]
    for l in r.findall('link'):
        item={'name':l.get('name')}
        for k in ['visual','collision']:
            arr=[]
            for c in l.findall(k):
                el={}
                if c.find('origin') is not None:el['origin']=c.find('origin').attrib
                if c.find('geometry') is not None:el['geometry']=[{x.tag:x.attrib} for x in c.find('geometry')]
                arr.append(el)
            item[k]=arr
        geometry.append(item)
    out.append({'file':str(f),'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'robot':r.get('name'),'joints':joints,'geometry':geometry})
(HERE/'urdf_inventory.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
for item in out:
    print('\nFILE',item['file'],'ROBOT',item['robot'])
    for j in item['joints']:print(json.dumps(j,ensure_ascii=False))
    print('BASE_GEOMETRY',json.dumps([g for g in item['geometry'] if any(x in g['name'].lower() for x in ['base','wheel','lift','body'])],ensure_ascii=False))

docfiles=[THESIS/'开题报告_代鹏.docx',THESIS/'中期检查/代鹏中期检查报告.docx']
ns={'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main'}
extracted=[]
for f in docfiles:
    if not f.exists():
        print('MISSING',str(f));continue
    root=etree.fromstring(ZipFile(f).read('word/document.xml'))
    texts=[''.join(p.xpath('.//w:t/text()',namespaces=ns)) for p in root.xpath('//w:p',namespaces=ns)]
    matches=[]
    for i,t in enumerate(texts):
        if any(k in t for k in ['六自由度','6自由度','五自由度','5自由度','麦克纳姆','Aloha','ALOHA','0.4','机械臂','全向']):
            matches.append({'paragraph_index':i,'text':t})
    extracted.append({'path':str(f),'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'matches':matches})
    print('\nDOCX',str(f),json.dumps(matches,ensure_ascii=False))
(HERE/'proposal_interim_extract.json').write_text(json.dumps(extracted,ensure_ascii=False,indent=2),encoding='utf-8')
