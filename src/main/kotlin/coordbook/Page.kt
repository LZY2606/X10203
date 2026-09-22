package coordbook

object Page {
    val HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>关节坐标册</title>
<style>
 body{font-family:system-ui,'PingFang SC',sans-serif;margin:0;background:#0f1420;color:#dfe6f3}
 header{background:#1a2333;padding:14px 22px;border-bottom:1px solid #2a3a55}
 h1{font-size:20px;margin:0;color:#7fd4ff}
 main{display:grid;grid-template-columns:1fr 1fr;gap:14px;padding:14px}
 section{background:#161e2e;border:1px solid #2a3a55;border-radius:8px;padding:12px}
 h2{font-size:14px;color:#9fb6d8;margin:0 0 8px}
 table{border-collapse:collapse;width:100%;font-size:12px}
 td,th{border:1px solid #2a3a55;padding:3px 6px;text-align:left}
 input,select,textarea,button{background:#0f1420;color:#dfe6f3;border:1px solid #3a4d70;border-radius:4px;padding:4px 6px;font-size:12px}
 button{cursor:pointer;background:#1d2c44}button:hover{background:#27395a}
 .ok{color:#7fe0a8}.bad{color:#ff8f8f}.warn{color:#ffd27f}
 .mono{font-family:ui-monospace,monospace;font-size:11px;white-space:pre}
 .full{grid-column:1/3}
 label{font-size:12px;color:#9fb6d8;margin-right:4px}
</style>
</head>
<body>
<header><h1>关节坐标册</h1>
<div style="margin-top:8px">
 <label>序列号</label><input id="serial" value="SN-001" size="8">
 <label>时刻(epoch s)</label><input id="time" placeholder="空=不限" size="12">
 <label>快照</label><select id="snapshot"><option value="">无</option></select>
 <button onclick="loadState()">刷新</button>
 <button onclick="freeze()">冻结当前快照</button>
 <a id="exportLink" href="/api/export?serial=SN-001" style="color:#7fd4ff;font-size:12px;margin-left:10px">导出 XML(仅获批补丁)</a>
</div></header>
<main>
 <section><h2>Link/Joint 图</h2><svg id="graph" width="100%" height="300"></svg></section>
 <section><h2>坐标查询</h2>
  <label>from</label><input id="from" value="base_link" size="10">
  <label>to</label><input id="to" value="tool0" size="10">
  <button onclick="runQuery()">查询</button>
  <div id="queryOut"></div>
 </section>
 <section><h2>三维坐标轴简图</h2><canvas id="axes" width="360" height="260"></canvas><div id="axesInfo" class="mono"></div></section>
 <section><h2>环路残差</h2><div id="cycles"></div></section>
 <section><h2>版本与差异</h2><div id="versions"></div>
  <button onclick="doDiff()">比较选中版本</button><div id="diffOut" class="mono"></div></section>
 <section><h2>快照对比(冻结 vs 另一套标定)</h2>
  <label>对比时刻</label><input id="altTime" size="12" placeholder="epoch s">
  <button onclick="doCompare()">对比</button><div id="compareOut" class="mono"></div></section>
 <section class="full"><h2>编辑草案(版本号并发控制)</h2>
  <textarea id="patch" rows="4" style="width:70%">{"ops":[{"op":"set_joint_origin","joint":"j4","xyz":[0.2,0,0.06],"rpy":[0,0,0]}]}</textarea>
  <button onclick="submitDraft()">提交草案</button>
  <div id="drafts"></div>
 </section>
</main>
<script>
let state=null;
const $=id=>document.getElementById(id);
function serial(){return $('serial').value}
function timeParam(){const t=$('time').value.trim();return t?('&time='+t):''}
async function api(p,opts){const r=await fetch(p,opts);return r.json()}

async function loadState(){
 state=await api('/api/state?serial='+serial()+timeParam());
 $('exportLink').href='/api/export?serial='+serial();
 drawGraph();renderCycles();renderVersions();renderDrafts();
 const ss=$('snapshot');ss.innerHTML='<option value="">无</option>';
 state.snapshots.forEach(s=>{ss.innerHTML+='<option value="'+s.id+'">'+s.name+' (t='+s.time+')</option>'});
}

function drawGraph(){
 const svg=$('graph');const frames=new Set();
 state.edges.forEach(e=>{frames.add(e.from);frames.add(e.to)});
 const names=[...frames];
 // layered layout by BFS depth from first link
 const depth={};const adj={};
 state.edges.forEach(e=>{(adj[e.from]=adj[e.from]||[]).push(e.to);(adj[e.to]=adj[e.to]||[]).push(e.from)});
 const root=state.links[0]||names[0];depth[root]=0;const q=[root];
 while(q.length){const u=q.shift();(adj[u]||[]).forEach(v=>{if(!(v in depth)){depth[v]=depth[u]+1;q.push(v)}})}
 names.forEach(n=>{if(!(n in depth))depth[n]=99});
 const layers={};names.forEach(n=>{(layers[depth[n]]=layers[depth[n]]||[]).push(n)});
 const pos={};const W=520,H=280;
 Object.keys(layers).forEach(d=>{const l=layers[d];l.forEach((n,i)=>{pos[n]={x:60+d*(W-120)/Math.max(1,Object.keys(layers).length-1),y:40+i*(H-80)/Math.max(1,l.length-1||1)}})});
 let s='';
 state.edges.forEach(e=>{const a=pos[e.from],b=pos[e.to];if(!a||!b)return;
  const col=e.kind==='calibration'?'#ffd27f':(e.duplicate?'#ff8f8f':'#4d7dbb');
  s+='<line x1="'+a.x+'" y1="'+a.y+'" x2="'+b.x+'" y2="'+b.y+'" stroke="'+col+'" stroke-width="'+(e.duplicate?2.5:1.5)+'" stroke-dasharray="'+(e.kind==='calibration'?'5,3':'')+'"/>';
  s+='<text x="'+((a.x+b.x)/2)+'" y="'+((a.y+b.y)/2-3)+'" fill="#8fa8cc" font-size="9">'+e.id+'</text>'});
 names.forEach(n=>{const p=pos[n];const isLink=state.links.includes(n);
  s+='<circle cx="'+p.x+'" cy="'+p.y+'" r="9" fill="'+(isLink?'#2c4a77':'#6b5a2a')+'" stroke="#7fd4ff"/>';
  s+='<text x="'+p.x+12+'" y="'+p.y+3+'" fill="#dfe6f3" font-size="10">'+n+'</text>'});
 svg.innerHTML=s;
}

async function runQuery(){
 const snap=$('snapshot').value;
 const p='/api/query?serial='+serial()+'&from='+$('from').value+'&to='+$('to').value+timeParam()+(snap?'&snapshot='+snap:'');
 const r=await api(p);
 let h='<div>候选路径: '+r.candidateCount+' 条(全部保留,不按遍历顺序取舍)</div>';
 (r.candidates||[]).forEach((c,i)=>{
  h+='<h2>候选 '+(i+1)+' — 状态 '+c.state+' 误差 t='+c.errorTranslation.toExponential(2)+' m, r='+(c.errorRotationDeg||0).toExponential(2)+'°</h2>';
  h+='<table><tr><th>边</th><th>方向</th><th>关节值</th><th>单位</th><th>来源</th><th>状态</th></tr>';
  c.chain.forEach(s=>{h+='<tr><td>'+s.edge+'</td><td>'+(s.forward?'正向':'反向')+'</td><td>'+(s.value===null?'—':s.value)+'</td><td>'+(s.unit||'—')+'</td><td>'+s.source+'</td><td>'+s.state+'</td></tr>'});
  h+='</table>';
  if(c.matrix){h+='<div class="mono">'+c.matrix.map(r=>r.map(v=>(+v).toFixed(4).padStart(9)).join(' ')).join('\n')+'</div>'}
 });
 $('queryOut').innerHTML=h;
 const best=(r.candidates||[]).find(c=>c.matrix);
 if(best)drawAxes(best.matrix,'候选1 末端姿态');
}

function drawAxes(m,label){
 const c=$('axes'),g=c.getContext('2d');g.clearRect(0,0,c.width,c.height);
 const ox=180,oy=140,sc=70;
 function proj(v){return{x:ox+sc*(v[0]-0.5*v[1]),y:oy-sc*(v[2]-0.35*v[1])}}
 const t=[m[0][3],m[1][3],m[2][3]];
 const o=proj(t);
 g.strokeStyle='#3a4d70';g.beginPath();g.moveTo(0,oy);g.lineTo(c.width,oy);g.moveTo(ox,0);g.lineTo(ox,c.height);g.stroke();
 const cols=['#ff6b6b','#7fe0a8','#5aa8ff'];const names=['X','Y','Z'];
 for(let i=0;i<3;i++){
  const v=[t[0]+m[0][i],t[1]+m[1][i],t[2]+m[2][i]];const p=proj(v);
  g.strokeStyle=cols[i];g.lineWidth=2;g.beginPath();g.moveTo(o.x,o.y);g.lineTo(p.x,p.y);g.stroke();
  g.fillStyle=cols[i];g.fillText(names[i],p.x+4,p.y);
 }
 $('axesInfo').textContent=label+' 平移=['+t.map(v=>(+v).toFixed(3)).join(', ')+']';
}

function renderCycles(){
 let h='';
 if(!state.cycles.length)h='<div class="ok">无环路(树结构)</div>';
 state.cycles.forEach(c=>{
  const cls=c.consistent?'ok':(c.state!=='ok'?'warn':'bad');
  h+='<div class="'+cls+'">环 ['+c.edges.join(' → ')+'] 闭合边='+c.closingEdge+
   (c.state!=='ok'?' 状态='+c.state:' 残差 t='+c.translationError.toExponential(3)+' m, r='+c.rotationErrorDeg.toExponential(3)+'° '+(c.consistent?'一致':'不一致'))+'</div>';
 });
 if(state.contradictionEdgeSet.length)h+='<div class="bad">最小矛盾边集: '+state.contradictionEdgeSet.join(', ')+'</div>';
 $('cycles').innerHTML=h;
}

function renderVersions(){
 let h='<div>当前 URDF 版本: v'+state.docVersion+'</div>';
 state.urdfVersions.forEach(v=>{h+='<label><input type="checkbox" class="vbox" value="'+v.version+'">v'+v.version+'</label> '});
 h+='<h2>标定(独立版本化)</h2><table><tr><th>名称</th><th>v</th><th>父→子</th><th>单位</th><th>生效区间</th><th>当前状态</th></tr>';
 state.calibrations.forEach(c=>{h+='<tr><td>'+c.name+'</td><td>'+c.version+'</td><td>'+c.parent+'→'+c.child+'</td><td>'+c.angleUnit+'</td><td>['+(c.validFrom??'-∞')+', '+(c.validTo??'+∞')+']</td><td>'+c.activity+'</td></tr>'});
 $('versions').innerHTML=h+'</table>';
}

async function doDiff(){
 const vs=[...document.querySelectorAll('.vbox:checked')].map(x=>x.value);
 if(vs.length!==2){$('diffOut').textContent='请选择两个版本';return}
 const r=await api('/api/diff?serial='+serial()+'&v1='+vs[0]+'&v2='+vs[1]);
 $('diffOut').textContent=(r.changes&&r.changes.length)?r.changes.join('\n'):'(无结构差异)';
}

async function freeze(){
 const name=prompt('快照名称','snapshot-'+Date.now());if(!name)return;
 const jv={};state.edges.forEach(e=>{if(e.kind==='joint'&&e.jointType!=='fixed'&&e.value!==null)jv[e.jointName]=e.value});
 const t=$('time').value.trim();
 await api('/api/snapshot',{method:'POST',body:JSON.stringify({name,serial:serial(),jointValues:jv,time:t?+t:null})});
 loadState();
}

async function doCompare(){
 const snap=$('snapshot').value;if(!snap){alert('先选择快照');return}
 const r=await api('/api/compare?snapshot='+snap+'&from='+$('from').value+'&to='+$('to').value+'&time='+$('altTime').value);
 $('compareOut').textContent=JSON.stringify(r,null,1);
}

async function submitDraft(){
 const r=await fetch('/api/draft',{method:'POST',body:JSON.stringify({serial:serial(),baseVersion:state.docVersion,patch:JSON.parse($('patch').value)})});
 if(r.status===409){alert('版本冲突:草案基于过期版本');return}
 loadState();
}
async function approve(id){const r=await fetch('/api/draft/'+id+'/approve',{method:'POST'});if(r.status===409)alert('并发冲突:基版本已过期');loadState()}
async function reject(id){await fetch('/api/draft/'+id+'/reject',{method:'POST'});loadState()}
function renderDrafts(){
 let h='<table><tr><th>id</th><th>基版本</th><th>状态</th><th>补丁</th><th></th></tr>';
 state.drafts.forEach(d=>{h+='<tr><td>'+d.id+'</td><td>v'+d.baseVersion+'</td><td>'+d.status+'</td><td class="mono">'+d.patch.replace(/</g,'&lt;')+'</td><td>'+(d.status==='pending'?'<button onclick="approve('+d.id+')">批准</button><button onclick="reject('+d.id+')">拒绝</button>':'')+'</td></tr>'});
 $('drafts').innerHTML=h+'</table>';
}
loadState();
</script>
</body></html>"""
}
