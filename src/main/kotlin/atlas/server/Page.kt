package atlas.server

const val PAGE = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>关节坐标册</title>
<style>
body{font-family:-apple-system,"PingFang SC",sans-serif;margin:0;background:#f4f6f8;color:#222}
header{background:#1f2d3d;color:#fff;padding:12px 20px;font-size:20px;font-weight:600}
main{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px}
section{background:#fff;border:1px solid #dde3ea;border-radius:8px;padding:12px}
h2{font-size:15px;margin:0 0 8px;color:#1f2d3d}
textarea{width:100%;box-sizing:border-box;font-family:monospace;font-size:12px}
input,select,button{font-size:13px;margin:2px;padding:4px 8px}
button{background:#1f6feb;color:#fff;border:0;border-radius:4px;cursor:pointer}
button.ghost{background:#e8edf3;color:#1f2d3d}
pre{background:#0d1220;color:#c9e1ff;padding:8px;border-radius:6px;overflow:auto;font-size:11px;max-height:260px}
canvas{background:#fbfcfe;border:1px solid #e3e9f0;border-radius:6px}
.tag{display:inline-block;background:#eef3fa;border:1px solid #cfddF0;border-radius:10px;padding:1px 8px;margin:1px;font-size:11px}
.warn{color:#b35900}.err{color:#c00}.ok{color:#0a7d2c}
table{border-collapse:collapse;font-size:12px;width:100%}
td,th{border:1px solid #e0e6ee;padding:3px 6px;text-align:left}
</style>
</head>
<body>
<header>关节坐标册 <span style="font-size:12px;font-weight:400">URDF + 标定变换 + 现场快照的坐标来源审阅</span></header>
<main>
<section>
<h2>1. URDF 文档（未知内容往返保留）</h2>
<input id="urdfName" placeholder="名称" value="demo">
<button onclick="importUrdf()">导入 URDF</button>
<button class="ghost" onclick="loadUrdfList()">刷新列表</button>
<textarea id="urdfXml" rows="6" placeholder="&lt;robot name=...&gt;...&lt;/robot&gt;"></textarea>
<div id="urdfList"></div>
<div>
版本差异: <input id="diffFrom" size="2" value="1"> → <input id="diffTo" size="2" value="2">
<button class="ghost" onclick="showDiff()">查看差异</button>
<a id="exportLink" href="#" target="_blank">导出当前 XML</a>
</div>
<pre id="diffOut"></pre>
</section>
<section>
<h2>2. 标定变换（独立版本化）</h2>
<button class="ghost" onclick="loadCals()">刷新</button>
<button onclick="addCal()">新增标定集</button>
<textarea id="calJson" rows="4">{"name":"现场标定A","robotSerial":"DEMO-1","validFrom":1000,"validTo":2000,"transforms":[{"parent":"base_link","child":"tool_cal","xyz":[0,0,0.1],"rpy":[0,0,0]}]}</textarea>
<div id="calList"></div>
</section>
<section>
<h2>3. 坐标查询（完整链 / 关节值 / 单位 / 矩阵 / 来源）</h2>
from <input id="qFrom" value="base_link" size="10"> to <input id="qTo" value="tip" size="10">
at <input id="qAt" size="8" placeholder="epoch ms"> serial <input id="qSerial" size="8" value="DEMO-1">
单位 <select id="qUnits"><option value="rad">rad</option><option value="deg">deg</option></select>
关节值 <input id="qJoints" size="16" placeholder="j1=0.5,j2=-0.3">
<button onclick="runQuery()">查询</button>
<pre id="queryOut"></pre>
<canvas id="axes" width="360" height="240"></canvas>
</section>
<section>
<h2>4. link/joint 图 与 环路残差</h2>
<button class="ghost" onclick="drawGraph()">绘制图</button>
<button class="ghost" onclick="showCycles()">计算环路残差</button>
<div id="graphIssues"></div>
<canvas id="graph" width="520" height="320"></canvas>
<pre id="cycleOut"></pre>
</section>
<section>
<h2>5. 现场快照冻结 / 对比另一套标定</h2>
<input id="snapName" placeholder="快照名" value="现场快照-1">
<input id="snapJoints" size="20" placeholder='{"j1":0.5,"j2":-0.3}' value='{"j1":0.5,"j2":-0.3}'>
<input id="snapCal" size="3" placeholder="标定ID">
<button onclick="freezeSnap()">冻结快照</button>
<div id="snapList"></div>
对比: 快照ID <input id="cmpSnap" size="3"> vs 标定ID <input id="cmpCal" size="3">
<button class="ghost" onclick="compareSnap()">比较</button>
<pre id="cmpOut"></pre>
</section>
<section>
<h2>6. 编辑草案（版本号并发控制，仅获批补丁可导出）</h2>
urdfId <input id="dUrdf" size="3" value="1"> baseVersion <input id="dBase" size="3" value="1">
<textarea id="dOps" rows="3">[{"type":"setOrigin","joint":"j1","xyz":[0,0,0.2],"rpy":[0,0,0]}]</textarea>
<button onclick="createDraft()">创建草案</button>
<button classghost class="ghost" onclick="loadDrafts()">刷新</button>
<div id="draftList"></div>
<pre id="draftOut"></pre>
</section>
</main>
<script>
var urdfId = 1;
function J(x){return JSON.stringify(x,null,1)}
function api(m,u,b){return fetch(u,{method:m,headers:{'Content-Type':'application/json'},body:b?JSON.stringify(b):undefined}).then(function(r){return r.text().then(function(t){if(!r.ok)throw new Error(r.status+' '+t);return t})})}
function curUrdf(){return urdfId}
function loadUrdfList(){api('GET','/api/urdf').then(function(t){var a=JSON.parse(t);if(a.length)urdfId=a[0].id;
document.getElementById('urdfList').innerHTML=a.map(function(d){return '<span class="tag">#'+d.id+' '+d.name+' v'+d.version+'</span>'}).join('');
document.getElementById('exportLink').href='/api/urdf/'+urdfId+'/export';drawGraph()})}
function importUrdf(){api('POST','/api/urdf',{name:document.getElementById('urdfName').value,xml:document.getElementById('urdfXml').value}).then(loadUrdfList).catch(function(e){alert(e.message)})}
function showDiff(){api('GET','/api/urdf/'+urdfId+'/diff?from='+document.getElementById('diffFrom').value+'&to='+document.getElementById('diffTo').value).then(function(t){document.getElementById('diffOut').textContent=J(JSON.parse(t))}).catch(function(e){document.getElementById('diffOut').textContent=e.message})}
function loadCals(){api('GET','/api/calibrations').then(function(t){var a=JSON.parse(t);
document.getElementById('calList').innerHTML=a.map(function(c){return '<span class="tag">#'+c.id+' '+c.name+' v'+c.version+' serial='+(c.robotSerial||'*')+' ['+(c.validFrom||'-∞')+','+(c.validTo||'+∞')+'] 变换×'+c.transforms.length+'</span>'}).join('')})}
function addCal(){api('POST','/api/calibrations',JSON.parse(document.getElementById('calJson').value)).then(loadCals).catch(function(e){alert(e.message)})}
function qp(){var s='?urdfId='+curUrdf();var at=document.getElementById('qAt').value;var sr=document.getElementById('qSerial').value;var j=document.getElementById('qJoints').value;
if(at)s+='&at='+at;if(sr)s+='&serial='+sr;if(j)s+='&joints='+encodeURIComponent(j);return s}
function runQuery(){var u='/api/query'+qp()+'&from='+encodeURIComponent(document.getElementById('qFrom').value)+'&to='+encodeURIComponent(document.getElementById('qTo').value)+'&units='+document.getElementById('qUnits').value;
api('GET',u).then(function(t){var r=JSON.parse(t);document.getElementById('queryOut').textContent=J(r);
if(r.candidates&&r.candidates.length)drawAxes(r.candidates[0].matrix)}).catch(function(e){document.getElementById('queryOut').textContent=e.message})}
function drawAxes(m){var c=document.getElementById('axes'),g=c.getContext('2d');g.clearRect(0,0,c.width,c.height);
var ox=180,oy=120,sc=60;var ca=Math.cos(0.5),sa=Math.sin(0.5);
function proj(x,y,z){var X=x*ca-z*sa;var Y=y-0.35*(x*sa+z*ca);return[ox+X*sc,oy-Y*sc]}
var o=proj(m[3],m[7],m[11]);
var cols=['#d33','#0a0','#26c'];var names=['X','Y','Z'];
for(var i=0;i<3;i++){var p=proj(m[3]+m[i],m[7]+m[4+i],m[11]+m[8+i]);
g.strokeStyle=cols[i];g.lineWidth=2;g.beginPath();g.moveTo(o[0],o[1]);g.lineTo(p[0],p[1]);g.stroke();
g.fillStyle=cols[i];g.fillText(names[i],p[0]+4,p[1]+4)}
g.fillStyle='#333';g.fillText('原点 ('+m[3].toFixed(3)+', '+m[7].toFixed(3)+', '+m[11].toFixed(3)+')',8,16)}
function drawGraph(){api('GET','/api/graph'+qp()).then(function(t){var d=JSON.parse(t);
document.getElementById('graphIssues').innerHTML=(d.issues||[]).map(function(i){return '<span class="tag warn">'+i+'</span>'}).join('');
var c=document.getElementById('graph'),g=c.getContext('2d');g.clearRect(0,0,c.width,c.height);
var n=d.nodes.length,cx=260,cy=160,R=Math.min(220,120);var pos={};
d.nodes.forEach(function(nd,i){var a=2*Math.PI*i/Math.max(n,1)-Math.PI/2;pos[nd]=[cx+R*Math.cos(a),cy+R*Math.sin(a)]});
d.edges.forEach(function(e){var a=pos[e.parent],b=pos[e.child];if(!a||!b)return;
g.strokeStyle=e.source==='urdf'?'#1f6feb':'#b35900';if(e.duplicateDeclaration)g.setLineDash([4,3]);else g.setLineDash([]);
g.beginPath();g.moveTo(a[0],a[1]);g.lineTo(b[0],b[1]);g.stroke();g.setLineDash([]);
var mx=(a[0]+b[0])/2,my=(a[1]+b[1])/2;g.fillStyle='#555';g.font='9px sans-serif';
g.fillText((e.jointName||e.source)+(e.duplicateDeclaration?' ⚠dup':''),mx,my)});
d.nodes.forEach(function(nd){var p=pos[nd];g.fillStyle='#fff';g.strokeStyle='#1f2d3d';
g.beginPath();g.arc(p[0],p[1],14,0,7);g.fill();g.stroke();g.fillStyle='#1f2d3d';g.font='9px sans-serif';
g.fillText(nd.substring(0,6),p[0]-12,p[1]+3)})}).catch(function(){})}
function showCycles(){api('GET','/api/cycles'+qp()).then(function(t){document.getElementById('cycleOut').textContent=J(JSON.parse(t))}).catch(function(e){document.getElementById('cycleOut').textContent=e.message})}
function loadSnaps(){api('GET','/api/snapshots').then(function(t){var a=JSON.parse(t);
document.getElementById('snapList').innerHTML=a.map(function(s){return '<span class="tag">#'+s.id+' '+s.name+' @'+s.capturedAt+' cal='+(s.calibrationId||'-')+'</span>'}).join('')})}
function freezeSnap(){var cal=document.getElementById('snapCal').value;
api('POST','/api/snapshots',{name:document.getElementById('snapName').value,robotSerial:document.getElementById('qSerial').value||null,jointValues:JSON.parse(document.getElementById('snapJoints').value||'{}'),capturedAt:Date.now(),calibrationId:cal?parseInt(cal):null}).then(loadSnaps).catch(function(e){alert(e.message)})}
function compareSnap(){api('GET','/api/compare?urdfId='+curUrdf()+'&snapshotId='+document.getElementById('cmpSnap').value+'&calibrationId='+document.getElementById('cmpCal').value).then(function(t){document.getElementById('cmpOut').textContent=J(JSON.parse(t))}).catch(function(e){document.getElementById('cmpOut').textContent=e.message})}
function loadDrafts(){api('GET','/api/drafts').then(function(t){var a=JSON.parse(t);
document.getElementById('draftList').innerHTML=a.map(function(d){return '<div class="tag">#'+d.id+' urdf='+d.urdfId+' base=v'+d.baseVersion+' <b>'+d.status+'</b> '+
'<button class="ghost" onclick="draftAct('+d.id+',\'approve\')">批准</button><button class="ghost" onclick="draftAct('+d.id+',\'apply\')">应用</button><button class="ghost" onclick="draftAct('+d.id+',\'reject\')">拒绝</button></div>'}).join('')})}
function createDraft(){api('POST','/api/drafts',{urdfId:parseInt(document.getElementById('dUrdf').value),baseVersion:parseInt(document.getElementById('dBase').value),ops:JSON.parse(document.getElementById('dOps').value)}).then(loadDrafts).catch(function(e){alert(e.message)})}
function draftAct(id,act){api('POST','/api/drafts/'+id+'/'+act).then(function(t){document.getElementById('draftOut').textContent=t;loadDrafts();loadUrdfList()}).catch(function(e){document.getElementById('draftOut').textContent=e.message;loadDrafts()})}
loadUrdfList();loadCals();loadSnaps();loadDrafts();
</script>
</body>
</html>
"""
