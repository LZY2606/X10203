'use strict';
const $ = (id) => document.getElementById(id);
let state = { robot: null, graph: null, snapshots: [], calibs: [], lastResult: null };

const SAMPLE_LOOP = `<?xml version="1.0"?>
<!-- 演示：固定关节形成的闭合环 + 未知元素 jcb:note + 角度单位 degree -->
<robot name="loop-arm" xmlns:jcb="http://example.com/jcb">
  <jcb:note sn="现场保留的未知元素">基座在工位北侧，原点为地脚螺栓中心</jcb:note>
  <link name="base"/>
  <link name="l0"/>
  <link name="l1"/>
  <link name="l2"/>
  <link name="l3"/>
  <joint name="j0" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="l0"/>
  </joint>
  <joint name="j1" type="revolute">
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
    <parent link="l0"/><child link="l1"/>
  </joint>
  <joint name="j2" type="revolute">
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2" upper="2" effort="10" velocity="1"/>
    <mimic joint="j1" multiplier="0.5" offset="0"/>
    <parent link="l1"/><child link="l2"/>
  </joint>
  <joint name="j3" type="fixed">
    <origin xyz="0.3 0 0" rpy="0 0 0"/>
    <parent link="l2"/><child link="l3"/>
  </joint>
  <!-- 闭合边：固定关节从 l3 回 base；这是第二条一致路径 -->
  <joint name="j_loop" type="fixed">
    <origin xyz="-0.6 0 0" rpy="0 0 0"/>
    <parent link="l3"/><child link="base"/>
  </joint>
</robot>`;

const SAMPLE_MIMIC = `<?xml version="1.0"?>
<robot name="mimic-arm">
  <link name="base"/>
  <link name="a"/>
  <link name="b"/>
  <link name="c"/>
  <joint name="ja" type="revolute">
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="5" velocity="1"/>
    <parent link="base"/><child link="a"/>
  </joint>
  <joint name="jb" type="revolute">
    <origin xyz="0.1 0 0"/><axis xyz="0 0 1"/>
    <mimic joint="ja" multiplier="2" offset="0.1"/>
    <parent link="a"/><child link="b"/>
  </joint>
  <joint name="jc" type="revolute">
    <origin xyz="0.1 0 0"/><axis xyz="0 0 1"/>
    <mimic joint="jb" multiplier="1" offset="0"/>
    <parent link="b"/><child link="c"/>
  </joint>
</robot>`;

function toast(msg, kind) {
  const div = document.createElement('div');
  div.className = 'toast-item ' + (kind || 'err');
  div.textContent = msg;
  $('toast').appendChild(div);
  setTimeout(() => div.remove(), 6000);
}

async function api(path, opts) {
  const res = await fetch(path, Object.assign({
    headers: { 'Content-Type': 'application/json' }
  }, opts));
  const text = await res.text();
  let data = null;
  try { data = text ? JSON.parse(text) : null; } catch (e) { data = text; }
  if (!res.ok) {
    const msg = (data && data.error) ? data.error : ('HTTP ' + res.status);
    const err = new Error(msg);
    err.status = res.status; throw err;
  }
  return data;
}

function loadSample(kind) {
  $('urdfXml').value = kind === 'mimic' ? SAMPLE_MIMIC : SAMPLE_LOOP;
  if (kind === 'mimic') {
    $('robotSerial').value = 'SN-MIMIC-002';
    $('qTo').value = 'c'; $('qFrom').value = 'base';
    $('snapValues').value = '{"ja":{"value":15,"unit":"deg"}}';
  } else {
    $('robotSerial').value = 'SN-ARM-001';
    $('qTo').value = 'l3'; $('qFrom').value = 'base';
    $('snapValues').value = '{"j1":{"value":30,"unit":"deg"}}';
  }
}

async function importRobot() {
  try {
    const body = {
      serial: $('robotSerial').value.trim(),
      name: $('robotName').value.trim() || null,
      urdfXml: $('urdfXml').value,
    };
    const data = await api('/api/robots', { method: 'POST', body: JSON.stringify(body) });
    state.graph = data.graph;
    toast('导入成功', 'ok');
    renderIssues(data.issues.concat(data.graph.issues || []));
    await refreshRobots(body.serial);
  } catch (e) { renderImportError(e.message); toast(e.message); }
}

function renderImportError(msg) {
  $('importIssues').innerHTML = '<span class="tag err">解析失败</span> <span class="muted">' + esc(msg) + '</span>';
}
function esc(s) {
  return String(s == null ? '' : s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
}

function issueHtml(list) {
  if (!list || !list.length) return '<span class="tag ok">无问题</span>';
  return list.map(i => {
    const cls = i.severity === 'error' ? 'err' : 'warn';
    return '<div><span class="tag ' + cls + '">' + esc(i.kind) + '</span> ' + esc(i.message) + '</div>';
  }).join('');
}
function renderIssues(list) { $('importIssues').innerHTML = issueHtml(list); }

async function refreshRobots(selectSerial) {
  const robots = await api('/api/robots');
  const sel = $('robotSelect');
  sel.innerHTML = robots.map(r =>
    '<option value="' + esc(r.serial) + '">' + esc(r.serial) + ' — ' + esc(r.name) + '</option>').join('');
  if (selectSerial) sel.value = selectSerial;
  if (sel.value) { $('robotSerial').value = sel.value; await selectRobot(); }
}

async function selectRobot() {
  const serial = $('robotSelect').value;
  if (!serial) return;
  $('robotSerial').value = serial;
  state.robot = serial;
  const xml = await (await fetch('/api/robots/' + encodeURIComponent(serial) + '/urdf')).text();
  $('urdfXml').value = xml;
  await Promise.all([refreshGraph(), refreshSnapshots(), loadCalibs()]);
}

async function refreshGraph() {
  const serial = state.robot || $('robotSelect').value;
  if (!serial) return;
  state.graph = await api('/api/robots/' + encodeURIComponent(serial) + '/graph');
  renderGraph(state.graph);
  renderIssues(state.graph.issues || []);
  const frames = state.graph.nodes.map(n => n.id);
  $('axisFrame').innerHTML = frames.map(f => '<option>' + esc(f) + '</option>').join('');
}

async function refreshSnapshots() {
  const serial = state.robot || $('robotSelect').value;
  if (!serial) return;
  state.snapshots = await api('/api/robots/' + encodeURIComponent(serial) + '/snapshots');
  $('snapTable').innerHTML = state.snapshots.map(s =>
    '<tr><td>' + s.id + '</td><td>' + esc(s.label) +
    '</td><td class="muted">' + esc(s.takenAt || '（无时间）') +
    '</td><td>' + (s.frozen ? '<span class="tag calib">已冻结</span>' : '<span class="tag warn">可变</span>') +
    '</td><td>' + (s.frozen ? '' : '<button onclick="freezeSnap(' + s.id + ')">冻结</button>') +
    '</td></tr>').join('');
  const opts = '<option value="">（不用）</option>' + state.snapshots.map(s =>
    '<option value="' + s.id + '">' + s.id + ' ' + esc(s.label) + (s.frozen ? ' ❄' : '') + '</option>').join('');
  $('qSnap').innerHTML = opts;
  $('qSnapB').innerHTML = '<option value="">（同 A）</option>' + opts;
}

async function createSnapshot() {
  try {
    const serial = $('robotSelect').value;
    let values = {};
    try { values = JSON.parse($('snapValues').value || '{}'); } catch (e) { throw new Error('关节值 JSON 解析失败: ' + e.message); }
    const body = {
      label: $('snapLabel').value, takenAt: $('snapTime').value || null, values,
    };
    await api('/api/robots/' + encodeURIComponent(serial) + '/snapshots',
      { method: 'POST', body: JSON.stringify(body) });
    toast('快照已创建', 'ok');
    await refreshSnapshots();
  } catch (e) { toast(e.message); }
}

async function freezeSnap(id) {
  try {
    await api('/api/snapshots/' + id + '/freeze', { method: 'POST' });
    toast('快照 #' + id + ' 已冻结；之后可用不同标定做 A/B 比较', 'ok');
    await refreshSnapshots();
  } catch (e) { toast(e.message); }
}

async function loadCalibs() {
  const serial = $('robotSelect').value;
  if (!serial) return;
  state.calibs = await api('/api/calibrations?serial=' + encodeURIComponent(serial));
  $('calibTable').innerHTML = state.calibs.map(c => {
    const cls = c.status === 'APPROVED' ? 'ok' : c.status === 'DRAFT' ? 'warn' : 'err';
    const btns = c.status === 'DRAFT'
      ? '<button onclick="approve(\'' + esc(c.id) + '\',' + c.version + ')">批准</button> ' +
        '<button class="warn" onclick="reject(\'' + esc(c.id) + '\',' + c.version + ')">驳回</button>'
      : '<span class="muted">v' + c.baseVersion + ' 基线</span>';
    return '<tr><td>' + esc(c.id) + '@v' + c.version +
      '</td><td>' + esc(c.parentFrame) + '→' + esc(c.childFrame) +
      '</td><td><span class="tag ' + cls + '">' + c.status + '</span>' +
      '</td><td>' + btns + '</td></tr>';
  }).join('');
  const latest = state.calibs.filter(c => c.status === 'APPROVED').reduce((m, c) =>
    (!m || c.version > m.version) ? c : m, null);
  if (latest) $('calibBaseVer').value = latest.version;
}

async function saveDraft() {
  try {
    const serial = $('robotSelect').value;
    const body = {
      id: $('calibId').value.trim(),
      robotSerial: serial,
      baseVersion: parseInt($('calibBaseVer').value || '0', 10),
      parentFrame: $('calibParent').value.trim(),
      childFrame: $('calibChild').value.trim(),
      xyz: parseNums($('calibXyz').value),
      rpyRad: parseNums($('calibRpy').value),
      validFrom: $('calibFrom').value || null,
      validTo: $('calibTo').value || null,
    };
    await api('/api/calibrations/drafts', { method: 'POST', body: JSON.stringify(body) });
    toast('草案已保存（未获批，不会进入导出）', 'ok');
    await loadCalibs(); await refreshGraph();
  } catch (e) {
    toast(e.status === 409 ? '并发冲突: ' + e.message : e.message);
  }
}
function parseNums(s) { return s.split(',').map(x => parseFloat(x.trim())); }

async function approve(id, version) {
  try {
    await api('/api/calibrations/' + encodeURIComponent(id) + '/approve',
      { method: 'POST', body: JSON.stringify({ version }) });
    toast(id + '@v' + version + ' 已批准', 'ok');
    await loadCalibs(); await refreshGraph();
  } catch (e) { toast(e.status === 409 ? '并发冲突: ' + e.message : e.message); }
}
async function reject(id, version) {
  await api('/api/calibrations/' + encodeURIComponent(id) + '/reject',
    { method: 'POST', body: JSON.stringify({ version }) });
  toast('已驳回', 'ok'); await loadCalibs();
}

function exportUrdf() {
  const serial = $('robotSelect').value;
  if (!serial) { toast('请先选择机器人'); return; }
  const ids = $('qCalibA').value.split(',').map(s => s.trim()).filter(Boolean);
  const qs = ids.map(i => 'calib=' + encodeURIComponent(i)).join('&');
  window.open('/api/robots/' + encodeURIComponent(serial) + '/export' + (qs ? '?' + qs : ''), '_blank');
}

function parseIdList(s) {
  return (s || '').split(',').map(x => x.trim()).filter(Boolean);
}

async function runQuery() {
  try {
    const serial = $('robotSelect').value;
    if (!serial) throw new Error('请先导入并选择机器人');
    let values = {};
    try { values = JSON.parse($('qValues').value || '{}'); } catch (e) { throw new Error('临时关节值 JSON 错误'); }
    const aIds = parseIdList($('qCalibA').value);
    const bIdsRaw = $('qCalibB').value.trim();
    const body = {
      from: $('qFrom').value.trim(), to: $('qTo').value.trim(),
      snapshotId: $('qSnap').value ? parseInt($('qSnap').value, 10) : null,
      calibrationIds: aIds.length ? aIds : null,
      values,
      toleranceDeg: 0.01,
    };
    if (bIdsRaw) {
      body.compareCalibrationIds = parseIdList(bIdsRaw);
      body.compareSnapshotId = $('qSnapB').value ? parseInt($('qSnapB').value, 10) : null;
    }
    const data = await api('/api/robots/' + encodeURIComponent(serial) + '/query',
      { method: 'POST', body: JSON.stringify(body) });
    if (data.base) {
      state.lastResult = data.base;
      state.lastCompare = data.other;
      renderEvaluation('A — ' + esc(body.calibrationIds || '最新获批'), data.base, $('queryResult'));
      renderDiff(data);
    } else {
      state.lastResult = data;
      renderEvaluation($('qCalibA').value || '最新获批', data, $('queryResult'));
      $('diffView').innerHTML = '<span class="muted">在“比较标定 B”填入另一套标定版本后再次查询即可看到差异。</span>';
    }
    drawAxes();
  } catch (e) { toast(e.message); }
}

function fmt(n) {
  if (Math.abs(n) < 1e-12) return '0.000000';
  return n.toFixed(6);
}
function matrixHtml(m) {
  return m.rows.map(r => r.map(fmt).join('  ')).join('\n');
}

function statusTag(s, missing) {
  if (s === 'OK') return '<span class="tag ok">OK</span>';
  if (s === 'MISSING_JOINT_VALUES')
    return '<span class="tag err">缺关节值: ' + esc(missing.join(', ')) + '</span>';
  return '<span class="tag err">不可达（断链/不同连通分量）</span>';
}

function renderEvaluation(title, ev, host) {
  const calibInfo = (ev.effectiveCalibrations || []).map(c => {
    const cls = c.effective ? 'ok' : 'warn';
    return '<div><span class="tag ' + cls + '">' + c.state + '</span> ' +
      esc(c.id) + '@v' + c.version + ' <span class="muted">' + esc(c.reason) + '</span></div>';
  }).join('') || '<div class="muted">本次查询未使用标定。</div>';

  const cand = (ev.candidates || []).map((c, i) => {
    const chain = c.steps.map((st, k) => {
      const jv = st.jointValue == null ? '固定' :
        ('q=' + fmt(st.jointValue) + ' ' + st.unit);
      const srcTag = st.edgeId.startsWith('calib:') ?
        '<span class="tag calib">标定</span>' : '<span class="tag">URDF</span>';
      return '<tr><td>' + (k + 1) + '</td><td>' + srcTag + '</td><td>' +
        esc(st.from) + ' → ' + esc(st.to) + '</td><td class="muted">' +
        esc(st.jointName || '-') + '</td><td>' + jv + '</td></tr>';
    }).join('');
    return '<div class="candidate"><h3>候选路径 #' + (i + 1) +
      (c.usesCalibration ? ' <span class="tag calib">含标定</span>' : '') +
      ' <span class="badge">可动关节 ' + c.movableJoints.length + ' 步，' + c.steps.length + ' 条边</span></h3>' +
      '<table><thead><tr><th>#</th><th>来源</th><th>frame 链</th><th>关节</th><th>值/单位</th></tr></thead>' +
      '<tbody>' + chain + '</tbody></table>' +
      '<details><summary>变换矩阵 T_'+esc(ev.from)+'_'+esc(ev.to)+'</summary>' +
      '<div class="matrix">' + matrixHtml(c.matrix) + '</div></details></div>';
  }).join('');

  const spread = ev.spreadTranslationM != null
    ? '多候选最大偏差: 平移 ' + fmt(ev.spreadTranslationM) + ' m，旋转 ' +
      fmt(ev.spreadRotationDeg) + '°（并列保留，不按遍历顺序取一）'
    : '';

  const cycles = (ev.inconsistentCycles || []).length
    ? '<div style="margin-top:8px"><span class="tag err">存在不一致环</span></div>'
    : (ev.cycles && ev.cycles.length ? '<div style="margin-top:8px"><span class="tag ok">所有环路一致</span></div>' : '');

  host.innerHTML =
    '<div class="muted">标定选择：' + esc(ev.selectorLabel) + '</div>' +
    '<div style="margin:6px 0">' + statusTag(ev.status, ev.missingJoints) + ' ' + cycles + '</div>' +
    '<div class="muted" style="margin:4px 0">' + spread + '</div>' +
    '<details open><summary>标定生效状态</summary><div style="margin-top:6px">' + calibInfo + '</div></details>' +
    '<div style="margin-top:10px">' + (cand || '<span class="tag err">无候选路径</span>') + '</div>';

  renderCyclePanel(ev);
}

function renderCyclePanel(ev) {
  const rows = (ev.cycles || []).map(c => {
    const tag = c.consistent ? '<span class="tag ok">一致</span>' : '<span class="tag err">矛盾</span>';
    return '<div class="candidate"><h3>' + tag + ' 环：' +
      esc(c.frames.join(' → ')) + '</h3>' +
      '<div>边集：' + c.edges.map(e =>
        '<span class="tag ' + (e.startsWith('calib:') ? 'calib' : '') + '">' + esc(e) + '</span>').join(' ') +
      '</div><div style="margin-top:5px">闭合残差：平移 <b>' + fmt(c.translationResidualM) +
      ' m</b>，旋转 <b>' + fmt(c.rotationResidualDeg) + '°</b> ' +
      '<span class="muted">(容差 ' + fmt(c.toleranceM) + ' m / ' + fmt(c.toleranceDeg) + '°)</span></div>' +
      '<details><summary>残差矩阵</summary><div class="matrix">' + matrixHtml(c.matrix) + '</div></details></div>';
  }).join('') || '<span class="muted">该图是树结构，没有环路。</span>';
  const hits = (ev.minContradictingEdgeSets || []).length
    ? '<h3>最小矛盾边集（修正其中任意一组即可消除全部环矛盾）</h3>' +
      ev.minContradictingEdgeSets.map(set =>
        '<div class="pillset">' + set.map(e =>
          '<span class="tag err">' + esc(e) + '</span>').join('') + '</div>').join('')
    : '';
  $('cycleView').innerHTML =
    '<div class="muted" style="margin-bottom:8px">' + esc(ev.cycleNote || '') + '</div>' +
    rows + hits;
}

function renderDiff(data) {
  if (!data.diffs || !data.diffs.length) {
    $('diffView').innerHTML = '<span class="muted">两套标定之间没有差异。</span>'; return;
  }
  $('diffView').innerHTML =
    '<div class="grid2"><div><h3>A 结果</h3><div id="diffA"></div></div>' +
    '<div><h3>B 结果</h3><div id="diffB"></div></div></div>' +
    '<table><thead><tr><th>边</th><th>版本 A→B</th><th>变化</th></tr></thead><tbody>' +
    data.diffs.map(d => '<tr><td>' + esc(d.parent) + ' → ' + esc(d.child) +
      '</td><td>v' + (d.oldVersion == null ? '∅' : d.oldVersion) + ' → v' +
      (d.newVersion == null ? '∅' : d.newVersion) +
      '</td><td>' + d.changes.map(esc).join('<br>') + '</td></tr>').join('') +
    '</tbody></table>';
  renderMini('diffA', data.base);
  renderMini('diffB', data.other);
}
function renderMini(id, ev) {
  const c = ev.candidates && ev.candidates[0];
  $(id).innerHTML = c
    ? '<div class="matrix">' + matrixHtml(c.matrix) + '</div>' +
      '<div class="muted">候选数 ' + ev.candidates.length + '</div>'
    : '<span class="tag err">无路径</span>';
}

function showTab(name) {
  ['graph', 'axes', 'cycle', 'diff'].forEach(t => {
    $('view-' + t).style.display = t === name ? 'block' : 'none';
    $('tab-' + t).classList.toggle('active', t === name);
  });
  if (name === 'axes') drawAxes();
}

// ---------- 简易力导向 link/joint 图 ----------
function renderGraph(g) {
  const svg = $('graphSvg');
  const W = svg.clientWidth || 900, H = 420;
  svg.setAttribute('viewBox', '0 0 ' + W + ' ' + H);
  const nodes = g.nodes.map((n, i) => ({
    id: n.id, kind: n.kind,
    x: W / 2 + 220 * Math.cos(2 * Math.PI * i / Math.max(g.nodes.length, 1)),
    y: H / 2 + 180 * Math.sin(2 * Math.PI * i / Math.max(g.nodes.length, 1)),
    vx: 0, vy: 0,
  }));
  const byId = Object.fromEntries(nodes.map(n => [n.id, n]));
  const links = g.links.map(l => ({ s: byId[l.source], t: byId[l.target], raw: l }))
    .filter(l => l.s && l.t);
  // base 固定在左侧
  const root = byId['base'] || nodes[0];
  if (root) { root.x = 90; root.y = H / 2; root.fixed = true; }

  for (let iter = 0; iter < 400; iter++) {
    for (const a of nodes) for (const b of nodes) {
      if (a === b) continue;
      let dx = a.x - b.x, dy = a.y - b.y;
      let d2 = dx * dx + dy * dy + 0.01;
      const f = 2600 / d2;
      let d = Math.sqrt(d2);
      a.vx += f * dx / d; a.vy += f * dy / d;
    }
    for (const l of links) {
      const dx = l.t.x - l.s.x, dy = l.t.y - l.s.y;
      const d = Math.sqrt(dx * dx + dy * dy) + 0.01;
      const target = 130, f = (d - target) * 0.02;
      l.s.vx += f * dx / d; l.s.vy += f * dy / d;
      l.t.vx -= f * dx / d; l.t.vy -= f * dy / d;
    }
    for (const n of nodes) {
      if (n.fixed) { n.vx = 0; n.vy = 0; continue; }
      n.vx *= 0.85; n.vy *= 0.85;
      n.x += n.vx * 0.05; n.y += n.vy * 0.05;
      n.x = Math.max(40, Math.min(W - 40, n.x));
      n.y = Math.max(30, Math.min(H - 30, n.y));
    }
  }

  const NS = 'http://www.w3.org/2000/svg';
  svg.innerHTML = '';
  const defs = document.createElementNS(NS, 'defs');
  defs.innerHTML = '<marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="3" orient="auto">' +
    '<path d="M0,0 L7,3 L0,6 Z" fill="#7f8db0"/></marker>' +
    '<marker id="arrowC" markerWidth="8" markerHeight="8" refX="7" refY="3" orient="auto">' +
    '<path d="M0,0 L7,3 L0,6 Z" fill="#b98bff"/></marker>';
  svg.appendChild(defs);
  for (const l of links) {
    const line = document.createElementNS(NS, 'line');
    line.setAttribute('x1', l.s.x); line.setAttribute('y1', l.s.y);
    line.setAttribute('x2', l.t.x); line.setAttribute('y2', l.t.y);
    const calib = l.raw.kind === 'calibration';
    line.setAttribute('stroke', calib ? '#b98bff' : '#7f8db0');
    line.setAttribute('stroke-width', calib ? 2.2 : 1.3);
    if (calib) line.setAttribute('stroke-dasharray', '6 4');
    line.setAttribute('marker-end', calib ? 'url(#arrowC)' : 'url(#arrow)');
    const title = document.createElementNS(NS, 'title');
    title.textContent = l.raw.label + ' [' + l.raw.kind + ']';
    line.appendChild(title);
    svg.appendChild(line);
    const mx = (l.s.x + l.t.x) / 2, my = (l.s.y + l.t.y) / 2;
    const txt = document.createElementNS(NS, 'text');
    txt.setAttribute('x', mx); txt.setAttribute('y', my - 4);
    txt.setAttribute('fill', calib ? '#d3b4ff' : '#93a0b8');
    txt.setAttribute('font-size', '10'); txt.setAttribute('text-anchor', 'middle');
    txt.textContent = l.raw.label.replace(/^URDF joint /, '');
    svg.appendChild(txt);
  }
  for (const n of nodes) {
    const c = document.createElementNS(NS, 'circle');
    c.setAttribute('cx', n.x); c.setAttribute('cy', n.y);
    c.setAttribute('r', n.kind === 'link' ? 7 : 5);
    c.setAttribute('fill', n.kind === 'link' ? '#4ea1ff' : '#e6b455');
    const t = document.createElementNS(NS, 'title'); t.textContent = n.id + ' (' + n.kind + ')';
    c.appendChild(t); svg.appendChild(c);
    const label = document.createElementNS(NS, 'text');
    label.setAttribute('x', n.x + 10); label.setAttribute('y', n.y + 4);
    label.setAttribute('fill', '#e6ebf5'); label.setAttribute('font-size', '11');
    label.textContent = n.id; svg.appendChild(label);
  }
}

// ---------- 3D 坐标轴简图：用查询结果把每个 frame 的原点/姿态画在 from 坐标系 ----------
let yawView = -0.8, pitchView = 0.5;
function project(p) {
  const cy = Math.cos(yawView), sy = Math.sin(yawView);
  const cp = Math.cos(pitchView), sp = Math.sin(pitchView);
  const x1 = cy * p[0] + sy * p[1];
  const y1 = -sy * p[0] + cy * p[1];
  const z1 = p[2];
  const y2 = cp * y1 - sp * z1;
  const z2 = sp * y1 + cp * z1;
  return [x1, -z2];
}

function drawAxes() {
  const canvas = $('axesCanvas');
  if (!canvas || $('view-axes').style.display === 'none') return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  ctx.fillStyle = '#0b101b'; ctx.fillRect(0, 0, W, H);
  ctx.translate(W / 2, H / 2 + 40);
  const scale = 260;

  function drawTriad(T, label) {
    const o = [T[3], T[7], T[11]];
    const axes = [
      { c: '#ef6b6b', v: [T[0], T[1], T[2]] },
      { c: '#3fbf7f', v: [T[4], T[5], T[6]] },
      { c: '#4ea1ff', v: [T[8], T[9], T[10]] },
    ];
    const po = project(o);
    for (const a of axes) {
      const tip = [o[0] + a.v[0] * 0.12, o[1] + a.v[1] * 0.12, o[2] + a.v[2] * 0.12];
      const pt = project(tip);
      ctx.strokeStyle = a.c; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(po[0] * scale, po[1] * scale);
      ctx.lineTo(pt[0] * scale, pt[1] * scale); ctx.stroke();
    }
    ctx.fillStyle = '#e6ebf5'; ctx.font = '11px sans-serif';
    ctx.fillText(label, po[0] * scale + 5, po[1] * scale - 5);
  }

  drawTriad([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1], $('qFrom').value || 'from');
  const ev = state.lastResult;
  if (ev && ev.candidates) {
    const drawn = new Set();
    for (const c of ev.candidates) {
      for (const st of c.steps) {
        const name = st.to;
        if (drawn.has(name)) continue;
        drawn.add(name);
        const rows = st.matrix.rows;
        const T = rows.flat();
        drawTriad(T, name + (st.edgeId.startsWith('calib:') ? ' ★' : ''));
      }
      // 终态
      const end = ev.to;
      if (!drawn.has(end)) { drawn.add(end); drawTriad(c.matrix.rows.flat(), end); }
    }
  }
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = '#93a0b8'; ctx.font = '11px sans-serif';
  ctx.fillText('鼠标拖动可旋转视角（红 X / 绿 Y / 蓝 Z，★ 标定来源）', 14, H - 12);
}

(function () {
  const canvas = document.getElementById('axesCanvas');
  if (!canvas) return;
  let dragging = false, lx = 0, ly = 0;
  canvas.addEventListener('mousedown', e => { dragging = true; lx = e.clientX; ly = e.clientY; });
  window.addEventListener('mouseup', () => dragging = false);
  window.addEventListener('mousemove', e => {
    if (!dragging) return;
    yawView += (e.clientX - lx) * 0.01;
    pitchView = Math.max(-1.4, Math.min(1.4, pitchView + (e.clientY - ly) * 0.01));
    lx = e.clientX; ly = e.clientY; drawAxes();
  });
})();

async function refreshAll() {
  await refreshRobots($('robotSelect').value);
}

(async function init() {
  loadSample('sample');
  await refreshRobots();
})();
