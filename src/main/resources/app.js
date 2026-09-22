"use strict";
const S = { robots: [], robot: null, serial: null, graph: null, frames: [], edges: [],
  report: null, versions: [], snapshots: [], patches: [], chosenCandidate: null,
  yaw: 0.6, pitch: 0.35 };

const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? "").replace(/[&<>"]/g, c =>
  ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

async function api(path, opts) {
  const res = await fetch(path, opts);
  if (!res.ok) {
    let msg = res.statusText;
    try { msg = JSON.stringify(await res.json()); } catch (e) {}
    throw new Error(res.status + " " + msg);
  }
  return res.json();
}

function atParam() {
  const v = $("atTime").value;
  return v ? "?at=" + new Date(v).toISOString() : "";
}
function calibParam() {
  const cid = $("calibSel").value;
  return cid ? "&calibrationVersionId=" + encodeURIComponent(cid) : "";
}

async function init() {
  S.robots = await api("/api/robots");
  $("robotSel").innerHTML = S.robots.map(r =>
    `<option value="${esc(r.serial)}">${esc(r.serial)} — ${esc(r.name)}</option>`).join("");
  if (S.robots.length) {
    $("robotSel").value = S.robots[0].serial;
    const now = new Date();
    $("atTime").value = localIso(now);
    await loadRobot();
  }
  setupAxisDrag();
}
function localIso(d) {
  const p = (n) => String(n).padStart(2, "0");
  return `${d.getFullYear()}-${p(d.getMonth()+1)}-${p(d.getDate())}T${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}

async function loadRobot() {
  S.serial = $("robotSel").value;
  const data = await api(`/api/robot/${encodeURIComponent(S.serial)}` + atParam() + calibParam());
  S.robot = data; S.frames = data.frames; S.edges = data.edges;
  S.report = data.report; S.versions = data.calibrationVersions;
  S.snapshots = data.snapshots; S.patches = data.patches;
  renderFrameSelectors(); renderIssues(); renderGraph(); renderLoops();
  renderCalibs(); renderSnapshots(); renderPatches(); renderPatchEdges();
  drawAxes(0);
  $("queryResult").innerHTML = ""; $("compareResult").innerHTML = "";
  $("diffResult").innerHTML = ""; $("exportPreview").textContent = "";
}

function renderFrameSelectors() {
  const opts = S.frames.map(f =>
    `<option value="${esc(f.name)}">${esc(f.name)}${f.source !== "urdf" ? " (" + esc(f.source) + ")" : ""}</option>`);
  $("fromFrame").innerHTML = opts;
  $("toFrame").innerHTML = opts;
  const names = S.frames.map(f => f.name);
  if (names.includes("base")) $("fromFrame").value = "base";
  const tip = names.find(n => /tool|tcp|ee/i.test(n));
  if (tip) $("toFrame").value = tip;
  renderJointInputs();
}

function movableEdges() { return S.edges.filter(e => e.movable); }
function renderJointInputs() {
  const seen = new Set();
  const rows = movableEdges().filter(e => e.jointName && !seen.has(e.jointName) && seen.add(e.jointName))
    .map(e => `<label class="mono" style="font-size:12px">${esc(e.jointName)}
      <input data-j="${esc(e.jointName)}" value="0" style="width:90px"/></label>`).join("");
  $("jointInputs").innerHTML = `<div class="muted">现场关节值（旋转按所选角度单位，平移为 m）</div><div class="row">${rows}</div>`;
}
function readJointValues() {
  const vals = {}, units = {};
  const au = $("angleUnit").value;
  $("jointInputs").querySelectorAll("input[data-j]").forEach(inp => {
    const name = inp.getAttribute("data-j");
    if (inp.value.trim() !== "") {
      vals[name] = parseFloat(inp.value);
      units[name] = au;
    }
  });
  return { vals, units };
}

// ---------- 力导向图 ----------
function renderGraph() {
  const svg = $("graphSvg"); const W = 340, H = 320;
  const nodes = S.frames.map((f, i) => ({ id: f.id, name: f.name, source: f.source,
    x: W/2 + 120*Math.cos(i*2.399), y: H/2 + 120*Math.sin(i*2.399), vx:0, vy:0 }));
  const byId = Object.fromEntries(nodes.map(n => [n.id, n]));
  const links = S.edges.map(e => ({ s: byId[e.a], t: byId[e.b], kind: e.kind, id: e.id }))
    .filter(l => l.s && l.t);
  for (let k = 0; k < 220; k++) {
    nodes.forEach(n => { n.vx += (W/2 - n.x) * 0.002; n.vy += (H/2 - n.y) * 0.002; });
    links.forEach(l => {
      const dx = l.t.x - l.s.x, dy = l.t.y - l.s.y;
      const d = Math.hypot(dx, dy) || 1; const f = (d - 70) * 0.01;
      const fx = dx/d*f, fy = dy/d*f;
      l.s.vx += fx; l.s.vy += fy; l.t.vx -= fx; l.t.vy -= fy;
    });
    for (let i = 0; i < nodes.length; i++) for (let j = i+1; j < nodes.length; j++) {
      const dx = nodes[j].x-nodes[i].x, dy = nodes[j].y-nodes[i].y;
      const d2 = dx*dx+dy*dy + 0.01;
      if (d2 < 1200) { const f = 900/d2;
        nodes[i].vx -= dx/d2*f*10; nodes[i].vy -= dy/d2*f*10;
        nodes[j].vx += dx/d2*f*10; nodes[j].vy += dy/d2*f*10; }
    }
    nodes.forEach(n => {
      n.x = Math.max(20, Math.min(W-20, n.x + n.vx));
      n.y = Math.max(16, Math.min(H-16, n.y + n.vy));
      n.vx *= 0.8; n.vy *= 0.8;
    });
  }
  const edgeColor = (kind) => kind.startsWith("calib-override") ? "#e8b341"
    : kind === "calib-new" ? "#3fbf7f" : kind === "duplicate-decl" ? "#ef6b6b" : "#5aa7ff";
  svg.innerHTML = links.map(l =>
    `<line x1="${l.s.x.toFixed(1)}" y1="${l.s.y.toFixed(1)}" x2="${l.t.x.toFixed(1)}" y2="${l.t.y.toFixed(1)}"
      stroke="${edgeColor(l.kind)}" stroke-width="${l.kind === "duplicate-decl" ? 3 : 1.5}" opacity="0.8"/>`)
    .join("") + nodes.map(n => {
      const fill = n.source === "dangling" ? "#ef6b6b"
        : n.source === "urdf(duplicate)" ? "#e8b341" : "#cfe0f7";
      return `<g><circle cx="${n.x.toFixed(1)}" cy="${n.y.toFixed(1)}" r="6" fill="${fill}"/>
        <text x="${(n.x+8).toFixed(1)}" y="${(n.y+3).toFixed(1)}" fill="#9fb3d1" font-size="9">${esc(n.name)}</text></g>`;
    }).join("");
  $("graphLegend").innerHTML =
    `<span class="tag info">URDF</span> <span class="tag warn">标定覆盖</span>
     <span class="tag ok">标定新增</span> <span class="tag err">重复/断链</span>
     连通分量 ${S.report.components.length} 个`;
}

// ---------- 状态与环路 ----------
function renderIssues() {
  const tag = (lv) => lv === "error" ? "err" : lv === "warning" ? "warn" : "info";
  const rows = S.robot.issues.map(i =>
    `<div><span class="tag ${tag(i.level)}">${esc(i.level)}</span>
     <code>${esc(i.code)}</code> ${esc(i.message)}</div>`).join("");
  const r = S.report;
  const broken = r.brokenLinks.map(b => `<div class="errText">✖ ${esc(b)}</div>`).join("");
  const dup = r.duplicateFrames.map(f =>
    `<div class="warnText">⚠ 重复 frame：${esc(f)}</div>`).join("");
  const dd = r.duplicateDeclarations.map(d =>
    `<div class="errText">⧉ ${esc(d)}</div>`).join("");
  $("issues").innerHTML = rows + broken + dup + dd +
    `<div class="muted">连通分量：${r.components.map(c => c.join(", ")).map(esc).join(" ｜ ")}</div>`;
}

function renderLoops() {
  const r = S.report;
  if (!r.loops.length) { $("loops").innerHTML = `<span class="okText">图中无环路。</span>`; return; }
  $("loops").innerHTML = r.loops.map(l => `
    <div class="cand ${l.consistent ? "" : "chosen"}">
      <span class="tag ${l.consistent ? "ok" : "err"}">${l.consistent ? "一致" : "矛盾"}</span>
      <div class="mono" style="font-size:11px">${l.edges.map(esc).join(" → ")}</div>
      <div class="muted">位移残差 ${l.translationError.toExponential(3)} m ｜
        角度残差 ${l.rotationErrorDeg.toExponential(3)}°</div>
    </div>`).join("") + (r.minimumContradiction ? `
    <div class="cand chosen"><b>最小矛盾边集（${esc(r.minimumContradiction.method)}）</b>
      <div class="mono errText">${r.minimumContradiction.edgeSet.map(esc).join(", ")}</div>
      <div class="muted">${esc(r.minimumContradiction.note)}</div></div>` : "");
}

// ---------- 查询 ----------
async function runQuery() {
  const { vals, units } = readJointValues();
  const body = { serial: S.serial, from: $("fromFrame").value, to: $("toFrame").value,
    values: vals, units, at: new Date($("atTime").value).toISOString(),
    calibrationVersionId: $("calibSel").value || null };
  const q = await api("/api/query", { method: "POST",
    headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  S.lastQuery = q;
  const statusText = { UNIQUE: "唯一路径", MULTIPLE: "多条一致/并列路径",
    INCOMPLETE: "存在缺关节值的不完整路径", DISCONNECTED: "断链：两 frame 不连通",
    FROM_FRAME_MISSING: "起始 frame 不存在", TO_FRAME_MISSING: "目标 frame 不存在" }[q.status] || q.status;
  const jv = q.jointValues.map(j => {
    const cls = j.status === "OK" ? "okText" : j.status === "MISSING" ? "warnText" : "errText";
    return `<tr><td class="mono">${esc(j.jointName)}</td>
      <td class="${cls}">${j.value == null ? "缺失" : j.value.toFixed(6)}</td>
      <td>${esc(j.unit)}</td><td>${esc(j.providedUnit ?? "-")}</td>
      <td>${j.fromMimic ? "mimic: " + esc(j.mimicChain.join("→")) : "-"}</td>
      <td class="${cls}">${esc(j.status)}</td></tr>`;
  }).join("");
  $("queryResult").innerHTML = `
    <div class="row"><span class="tag ${q.status === "DISCONNECTED" ? "err" : "info"}">${esc(statusText)}</span>
      ${q.spreadTranslationMeters != null ? `<span class="muted">候选间最大偏差：${q.spreadTranslationMeters.toExponential(3)} m / ${q.spreadRotationDeg.toFixed(4)}°</span>` : ""}
    </div>
    <details><summary>关节值表（${q.jointValues.length}）</summary>
      <table><tr><th>关节</th><th>SI 值</th><th>单位</th><th>输入单位</th><th>mimic 链</th><th>状态</th></tr>${jv}</table>
    </details>
    <div id="candList"></div>`;
  $("candList").innerHTML = q.candidates.map(renderCandidate).join("");
}

function renderCandidate(c) {
  const chosen = S.lastQuery.chosenIndex === c.index;
  const matrix = c.transform.rows.map(r => r.map(v => v.toFixed(4).padStart(9)).join(" ")).join("\n");
  const steps = c.steps.map(s => `<tr>
      <td>${esc(s.from)} → ${esc(s.to)}${s.reversed ? " (逆)" : ""}</td>
      <td>${esc(s.explanation)}</td></tr>`).join("");
  return `<div class="cand ${chosen ? "chosen" : ""}">
    <b>候选 #${c.index} ${chosen ? "（代表解）" : ""}</b>
    <span class="tag ${c.complete ? "ok" : "warn"}">${c.complete ? "完整" : "不完整"}</span>
    <div class="mono muted">${esc(c.chainSummary)}</div>
    <details><summary>完整链 / 来源（${c.steps.length} 跳）</summary>
      <table><tr><th>跳转</th><th>定义来源与关节值</th></tr>${steps}</table></details>
    ${c.problems.length ? `<div class="warnText">${c.problems.map(esc).join("<br/>")}</div>` : ""}
    <div class="row"><div class="matrix">${esc(matrix)}</div>
      <button onclick='showCandidate3d(${c.index})'>3D 查看</button></div>
  </div>`;
}

// ---------- 3D 坐标轴 ----------
function project(x, y, z) {
  const cy = Math.cos(S.yaw), sy = Math.sin(S.yaw);
  const cp = Math.cos(S.pitch), sp = Math.sin(S.pitch);
  let x1 = x * cy - z * sy, z1 = x * sy + z * cy;
  let y1 = y * cp - z1 * sp, z2 = y * sp + z1 * cp;
  const f = 160 / (1 + z2 * 0.0);
  return [190 + x1 * f, 150 - y1 * f];
}
function drawAxesFromMatrix(M, scale = 0.9) {
  const cv = $("axis3d"); const ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, cv.width, cv.height);
  drawFrame(ctx, matIdentity(), 0.55, true);
  if (M) drawFrame(ctx, M, scale, false);
}
function matIdentity() {
  return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
}
function drawFrame(ctx, M, scale, dashed) {
  const o = [M[0][3], M[1][3], M[2][3]];
  const ax = [["#ef6b6b", 0], ["#3fbf7f", 1], ["#5aa7ff", 2]];
  ax.forEach(([color, i]) => {
    const end = [o[0] + M[0][i] * scale, o[1] + M[1][i] * scale, o[2] + M[2][i] * scale];
    const p0 = project(o[0], o[1], o[2]), p1 = project(end[0], end[1], end[2]);
    ctx.strokeStyle = color; ctx.lineWidth = dashed ? 1 : 3;
    ctx.setLineDash(dashed ? [4, 4] : []);
    ctx.beginPath(); ctx.moveTo(p0[0], p0[1]); ctx.lineTo(p1[0], p1[1]); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = color;
    ctx.fillText(["X", "Y", "Z"][i], p1[0] + 3, p1[1] + 3);
  });
}
function drawAxes(which) { drawAxesFromMatrix(which === 0 ? null : matIdentity()); }
function showCandidate3d(idx) {
  const c = S.lastQuery.candidates.find(x => x.index === idx);
  S.chosenCandidate = c;
  drawAxesFromMatrix(c.transform.rows);
}
function drawAxesFromChosen() {
  if (S.lastQuery && S.lastQuery.chosenIndex != null) {
    showCandidate3d(S.lastQuery.chosenIndex);
  } else drawAxes(0);
}
function setupAxisDrag() {
  const cv = $("axis3d"); let drag = false, lx = 0, ly = 0;
  cv.addEventListener("mousedown", e => { drag = true; lx = e.clientX; ly = e.clientY; });
  window.addEventListener("mouseup", () => drag = false);
  window.addEventListener("mousemove", e => {
    if (!drag) return;
    S.yaw += (e.clientX - lx) * 0.01; S.pitch += (e.clientY - ly) * 0.01;
    S.pitch = Math.max(-1.4, Math.min(1.4, S.pitch));
    lx = e.clientX; ly = e.clientY;
    drawAxesFromMatrix(S.chosenCandidate ? S.chosenCandidate.transform.rows : null);
  });
}

// ---------- 快照 ----------
async function freezeSnapshot() {
  const { vals, units } = readJointValues();
  const body = { serial: S.serial, label: $("snapLabel").value || "现场快照",
    values: vals, units, calibrationVersionId: $("calibSel").value || null,
    note: "冻结时刻 " + $("atTime").value };
  await api("/api/snapshots", { method: "POST",
    headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  await loadRobot();
}
function renderSnapshots() {
  const sOpts = S.snapshots.map(s => `<option value="${esc(s.id)}">${esc(s.label)} @ ${esc(s.frozenAt)}</option>`).join("");
  $("snapSel").innerHTML = sOpts;
  const cOpts = S.versions.map(v =>
    `<option value="${esc(v.id)}">v${v.version} ${esc(v.note)} [${esc(v.validityStatus)}]</option>`).join("");
  $("otherCalibSel").innerHTML = cOpts;
  $("snapshotList").innerHTML = S.snapshots.map(s =>
    `<div class="muted">📌 ${esc(s.label)} — ${esc(s.frozenAt)}
      ${s.calibrationVersionId ? "（绑定标定 " + esc(s.calibrationVersionId.slice(0,8)) + "）" : ""}<br/>
      q: ${esc(JSON.stringify(s.jointValues))}</div>`).join("");
}
async function compareSnapshot() {
  const pairs = [ { from: $("fromFrame").value, to: $("toFrame").value } ];
  const body = { snapshotId: $("snapSel").value,
    otherCalibrationVersionId: $("otherCalibSel").value, pairs };
  const res = await api("/api/compare", { method: "POST",
    headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  $("compareResult").innerHTML =
    `<div class="tag ${res.validity.startsWith("ACTIVE") || res.validity.startsWith("BOUNDARY_START") ? "ok" : "warn"}">${esc(res.validity)}</div>` +
    res.queries.map(q => {
      const c = q.candidates[q.chosenIndex ?? 0];
      const matrix = c ? c.transform.rows.map(r => r.map(v => v.toFixed(4).padStart(9)).join(" ")).join("\n") : "无可用路径";
      return `<div class="cand"><b>${esc(q.from)} → ${esc(q.to)}</b>
        <span class="tag ${q.status === "DISCONNECTED" ? "err" : "info"}">${esc(q.status)}</span>
        <div class="matrix">${esc(matrix)}</div>
        <details><summary>${q.candidates.length} 条候选链</summary>
          ${q.candidates.map(cc => `<div class="mono muted">#${cc.index} ${esc(cc.chainSummary)}
            ${cc.problems.length ? "⚠ " + esc(cc.problems.join("; ")) : ""}</div>`).join("")}
        </details></div>`;
    }).join("");
}

// ---------- 标定版本/差异/草案 ----------
function renderCalibs() {
  $("calibSel").innerHTML = `<option value="">按时间自动生效</option>` +
    S.versions.map(v => `<option value="${esc(v.id)}">v${v.version} ${esc(v.note)}</option>`).join("");
  $("diffTarget").innerHTML = S.versions.map(v =>
    `<option value="${v.version}">v${v.version} ${esc(v.note)}</option>`).join("");
  const tagCls = (st) => ({ ACTIVE: "ok", BOUNDARY_START: "ok", FUTURE: "info",
    EXPIRED: "warn", SERIAL_MISMATCH: "err", BOUNDARY_END: "warn" }[st] || "info");
  $("calibList").innerHTML = S.versions.map(v =>
    `<div class="cand">
      <b>v${v.version}</b> ${esc(v.note)}
      <span class="tag ${tagCls(v.validityStatus)}">${esc(v.validityStatus)}</span>
      ${v.approved ? '<span class="tag ok">已批准</span>' : '<span class="tag warn">草案</span>'}
      <div class="muted">${esc(v.validityReason)}</div>
      <div class="muted">窗口 [${esc(v.validFrom)}, ${esc(v.validUntil)}) ｜ ${v.edges.length} 条边
        ｜ 序列号 ${esc(v.robotSerial)}</div>
      ${!v.approved ? `<button onclick="approveVersion('${esc(v.id)}')">批准</button>` : ""}
    </div>`).join("");
}
async function approveVersion(id) {
  await api(`/api/versions/${encodeURIComponent(id)}/approve`, { method: "POST" });
  await loadRobot();
}
async function runDiff() {
  const target = $("diffTarget").value;
  const d = await api(`/api/diff/${encodeURIComponent(S.serial)}?target=${target}`);
  const edgeLine = (e) => `${esc(e.kind)} ${esc(e.jointName ?? (e.parentFrame + "->" + e.childFrame))} xyz=${esc(e.xyz.join(","))} rpy=${esc(e.rpyRad.join(","))}`;
  $("diffResult").innerHTML =
    `<div class="muted">v${d.baseVersion ?? "-"} → v${d.targetVersion}
      ${d.windowChanged ? "｜<span class='warnText'>时间窗口变化</span>" : ""}
      ${d.serialChanged ? "｜<span class='errText'>序列号变化</span>" : ""}</div>
    <div class="okText">新增 ${d.added.length}：${d.added.map(edgeLine).map(esc).join("<br/>")}</div>
    <div class="errText">移除 ${d.removed.length}：${d.removed.map(edgeLine).map(esc).join("<br/>")}</div>
    <div class="warnText">修改 ${d.changed.length}：${d.changed.map(c =>
      `<div class="mono" style="font-size:11px">${esc(c.edgeId)}<br/>旧 ${edgeLine(c.before)}<br/>新 ${edgeLine(c.after)}</div>`).join("")}</div>`;
}
function renderDraftEditor() {
  $("draftBox").style.display = "block";
  const d = new Date($("atTime").value); d.setHours(0,0,0,0);
  const e = new Date(d); e.setDate(e.getDate() + 30);
  $("draftFrom").value = localIso(d); $("draftUntil").value = localIso(e);
  const tmpl = S.edges.filter(x => x.jointName).slice(0, 1).map(x => ({
    id: "e-" + Math.random().toString(36).slice(2, 7),
    kind: "override", jointName: x.jointName,
    parentFrame: x.a, childFrame: x.b,
    xyz: [0.02, 0, 0], rpyRad: [0, 0, 0], note: "草案修正" }));
  $("draftEdges").value = JSON.stringify(tmpl, null, 2);
}
async function saveDraft() {
  let edges;
  try { edges = JSON.parse($("draftEdges").value); }
  catch (e) { $("draftMsg").innerHTML = `<span class="errText">JSON 解析失败：${esc(e.message)}</span>`; return; }
  const body = { serial: S.serial, note: $("draftNote").value,
    validFrom: new Date($("draftFrom").value).toISOString(),
    validUntil: new Date($("draftUntil").value).toISOString(), edges };
  try {
    await api("/api/versions", { method: "POST",
      headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
    $("draftMsg").textContent = "草案已创建（版本号并发控制：重复版本号会被拒绝）";
    await loadRobot();
  } catch (e) { $("draftMsg").innerHTML = `<span class="errText">${esc(e.message)}</span>`; }
}

// ---------- 补丁与导出 ----------
function renderPatchEdges() {
  const cid = $("calibSel").value;
  const v = S.versions.find(x => x.id === cid) || S.versions[0];
  if (!v) { $("patchEdgeSel").innerHTML = ""; return; }
  $("patchEdgeSel").innerHTML = v.edges.map(e =>
    `<option value="${esc(e.id)}">${esc(e.id)} ${esc(e.kind)} ${esc(e.jointName ?? e.parentFrame+"->"+e.childFrame)}</option>`).join("");
}
$("calibSel")?.addEventListener?.("change", renderPatchEdges);
function renderPatches() {
  $("patchList").innerHTML = S.patches.map(p =>
    `<div><span class="tag ${p.approved ? "ok" : "warn"}">${p.approved ? "已批准" : "未批准"}</span>
      ${esc(p.id.slice(0,8))} 边: ${esc(p.edgeIds.join(","))}
      ${!p.approved ? `<button onclick="approvePatch('${esc(p.id)}')">批准</button>` : ""}</div>`).join("");
}
async function createPatch() {
  const edgeIds = Array.from($("patchEdgeSel").selectedOptions).map(o => o.value);
  const cid = $("calibSel").value || (S.versions[0] && S.versions[0].id);
  await api("/api/patches", { method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ serial: S.serial, calibrationVersionId: cid, edgeIds, note: "页面登记" }) });
  await loadRobot();
}
async function approvePatch(id) {
  await api(`/api/patches/${encodeURIComponent(id)}/approve`, { method: "POST" });
  await loadRobot();
}
async function exportXml() {
  const cid = $("calibSel").value || (S.versions[0] && S.versions[0].id);
  const meta = await api(`/api/export/${encodeURIComponent(S.serial)}?calibrationVersionId=${encodeURIComponent(cid)}`);
  const text = await (await fetch(`/api/export/${encodeURIComponent(S.serial)}/xml?calibrationVersionId=${encodeURIComponent(cid)}`)).text();
  $("exportPreview").textContent =
    `[派生文件，非原始 URDF]\n已应用: ${meta.appliedEdges.join(", ") || "无"}\n` +
    `未批准跳过: ${meta.skippedUnapproved.join(", ") || "无"}\n\n` + text;
}

init().catch(e => { document.body.insertAdjacentHTML("afterbegin",
  `<div class="errText">初始化失败：${esc(e.message)}</div>`); });
