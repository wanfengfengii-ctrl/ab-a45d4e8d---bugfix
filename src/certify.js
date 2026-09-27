/**
 * 覆盖认证业务模块（纯本地、连续平面判定，无栅格化、无固定采样）。
 *
 * 方法：将工作区凸多边形用全部覆盖带边线做半平面剖分（线排列），
 * 得到有限个单元；同一单元内覆盖数恒定，用质心做精确包含计数：
 *   - 覆盖数 = 0  → 漏拍单元（gap）
 *   - 覆盖数 ≥ 3  → 三重曝光单元（triple）
 * 再用全部排列顶点（单元顶点 + 工作区顶点 + 覆盖带角点）做闭集计数，
 * 捕获零面积的三重接触（点/线段状三重曝光）。
 * 边界接触计入覆盖：所有包含判断均为闭集语义。
 */
import Decimal from './geometry/decimal.js';
import {
  EPS, D, pt,
  lineFromPoints, lineValue, signedArea,
  splitConvex, sanitizePolygon,
  buildRect, rectContains, convexContains,
} from './geometry/core.js';

export const LIMITS = { minStrips: 3, maxStrips: 12, maxVertices: 64 };

const SIDE_NAMES = ['底边', '右边', '顶边', '左边'];

/* ---------------- 输入校验 ---------------- */

export function validateInput(input) {
  const errors = [];
  const wa = input?.workarea;
  if (!Array.isArray(wa) || wa.length < 3) {
    errors.push('工作区至少需要 3 个顶点');
  } else {
    if (wa.length > LIMITS.maxVertices) errors.push(`工作区顶点数不能超过 ${LIMITS.maxVertices}`);
    let coordsOk = true;
    wa.forEach((v, i) => {
      if (!Array.isArray(v) || v.length !== 2 || !v.every((n) => Number.isFinite(n))) {
        errors.push(`工作区顶点 #${i + 1} 非法（应为有限数 [x, y]）`);
        coordsOk = false;
      }
    });
    if (coordsOk) {
      let scale = 1;
      for (const [x, y] of wa) scale = Math.max(scale, Math.abs(x), Math.abs(y));
      const tol = 1e-9 * scale * scale;
      let area2 = 0;
      let allLeft = true;
      let allRight = true;
      const n = wa.length;
      for (let i = 0; i < n; i++) {
        const [ax, ay] = wa[i];
        const [bx, by] = wa[(i + 1) % n];
        const [cx, cy] = wa[(i + 2) % n];
        area2 += ax * by - bx * ay;
        const cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx);
        if (cross <= tol) allLeft = false;
        if (cross >= -tol) allRight = false;
      }
      if (area2 <= tol && area2 >= -tol) {
        errors.push('工作区面积过小或顶点共线');
      } else if (area2 < 0 && allRight) {
        errors.push('工作区顶点必须按逆时针（CCW）顺序给出');
      } else if (area2 > 0 && !allLeft) {
        errors.push('工作区必须是严格凸多边形（每个内角均小于 180°）');
      } else if (area2 < 0 && !allRight) {
        errors.push('工作区既不是凸多边形，顶点顺序也可能不是逆时针');
      }
    }
  }
  const strips = input?.strips;
  if (!Array.isArray(strips) || strips.length < LIMITS.minStrips || strips.length > LIMITS.maxStrips) {
    errors.push(`覆盖带数量须为 ${LIMITS.minStrips}–${LIMITS.maxStrips} 条（当前 ${Array.isArray(strips) ? strips.length : 0} 条）`);
  } else {
    strips.forEach((s, i) => {
      const tag = `覆盖带 R${i + 1}`;
      if (!s || typeof s !== 'object') { errors.push(`${tag}：参数缺失`); return; }
      for (const k of ['cx', 'cy', 'w', 'h', 'angle']) {
        if (!Number.isInteger(s[k])) { errors.push(`${tag}：${k} 必须为整数（当前 ${s[k]}）`); return; }
      }
      if (s.w <= 0) errors.push(`${tag}：宽 w 必须为正整数`);
      if (s.h <= 0) errors.push(`${tag}：高 h 必须为正整数`);
    });
  }
  return errors;
}

/* ---------------- 直线工具 ---------------- */

function canonical(line) {
  let { a, b, c } = line;
  if (a.lt(EPS.neg()) || (a.abs().lte(EPS) && b.isNegative())) {
    a = a.neg(); b = b.neg(); c = c.neg();
  }
  return { a, b, c };
}

function sameLine(L1, L2, eps, cTol) {
  return (
    L1.a.minus(L2.a).abs().lte(eps) &&
    L1.b.minus(L2.b).abs().lte(eps) &&
    L1.c.minus(L2.c).abs().lte(cTol)
  );
}

/* ---------------- 主认证流程 ---------------- */

export function certify(input) {
  const errors = validateInput(input);
  if (errors.length) return { ok: false, errors };

  const workarea = input.workarea.map(([x, y]) => pt(x, y));
  const strips = input.strips.map((s, i) => ({ ...buildRect(s), params: s, index: i + 1 }));

  // 坐标量级 → 各级容差
  let scale = 1;
  for (const p of workarea) {
    scale = Math.max(scale, Math.abs(p.x.toNumber()), Math.abs(p.y.toNumber()));
  }
  for (const s of input.strips) {
    scale = Math.max(scale, Math.abs(s.cx) + s.w, Math.abs(s.cy) + s.h);
  }
  const S = D(scale);
  const eps = EPS;                            // 剖分/分类容差（1e-24）
  const sliverEps = D('1e-18').mul(S).mul(S); // 退化碎屑面积阈值
  const lineTol = D('1e-15').mul(S);          // 边界证据匹配容差
  const vtxTol = D('1e-9').mul(S);            // 顶点闭集判定容差

  // 1) 覆盖带边线（规范定向 + 共线合并，记录归属作为证据）
  const stripLines = [];
  strips.forEach((s, si) => {
    s.edges.forEach((e, k) => {
      const canon = canonical(e);
      const found = stripLines.find((L) => sameLine(L, canon, eps, lineTol));
      if (found) found.owners.push({ strip: si + 1, side: SIDE_NAMES[k] });
      else stripLines.push({ ...canon, owners: [{ strip: si + 1, side: SIDE_NAMES[k] }] });
    });
  });
  // 工作区边线（仅用于边界证据标注）
  const waLines = workarea.map((p, i) => ({
    ...canonical(lineFromPoints(p, workarea[(i + 1) % workarea.length])),
    owners: [{ workarea: true, edge: i }],
  }));

  // 2) 线排列剖分：工作区多边形被每条覆盖带边线切割
  let cells = [workarea];
  for (const line of stripLines) {
    const next = [];
    for (const cell of cells) {
      // 快速通道：整体位于某一侧
      let minV = null;
      let maxV = null;
      for (const p of cell) {
        const v = lineValue(line, p);
        if (minV === null || v.lt(minV)) minV = v;
        if (maxV === null || v.gt(maxV)) maxV = v;
      }
      if (minV.gt(eps) || maxV.lt(eps.neg())) { next.push(cell); continue; }
      const { pos, neg } = splitConvex(cell, line, eps);
      const pPos = sanitizePolygon(pos, eps, sliverEps);
      const pNeg = sanitizePolygon(neg, eps, sliverEps);
      if (pPos) next.push(pPos);
      if (pNeg) next.push(pNeg);
      if (!pPos && !pNeg) next.push(cell); // 单元整体贴线：保留原单元
    }
    cells = next;
  }

  // 3) 单元分类：质心包含计数（单元内覆盖数恒定）
  const cellRecs = cells.map((verts) => {
    let cx = new Decimal(0);
    let cy = new Decimal(0);
    for (const p of verts) { cx = cx.plus(p.x); cy = cy.plus(p.y); }
    const centroid = { x: cx.div(verts.length), y: cy.div(verts.length) };
    const covering = [];
    strips.forEach((s, i) => {
      if (rectContains(s.edges, centroid, eps)) covering.push(i + 1);
    });
    return {
      vertices: verts,
      centroid,
      count: covering.length,
      covering,
      area: signedArea(verts).abs(),
    };
  });

  const stats = {
    stripCount: strips.length,
    vertexCount: workarea.length,
    cellCount: cellRecs.length,
    workArea: signedArea(workarea).abs(),
    gapArea: new Decimal(0),
    tripleArea: new Decimal(0),
    singleArea: new Decimal(0),
    doubleArea: new Decimal(0),
    maxMultiplicity: 0,
  };
  const gapCells = [];
  const tripleCells = [];
  for (const c of cellRecs) {
    if (c.count > stats.maxMultiplicity) stats.maxMultiplicity = c.count;
    if (c.count === 0) { gapCells.push(c); stats.gapArea = stats.gapArea.plus(c.area); }
    else if (c.count === 1) stats.singleArea = stats.singleArea.plus(c.area);
    else if (c.count === 2) stats.doubleArea = stats.doubleArea.plus(c.area);
    else { tripleCells.push(c); stats.tripleArea = stats.tripleArea.plus(c.area); }
  }

  // 4) 零面积三重接触检测（闭集，边界接触计入）：
  //    (a) 排列顶点（单元顶点 + 工作区顶点 + 覆盖带角点）计数 → 孤立接触点；
  //    (b) 落在覆盖带边线上的排列边，以其中点计数 → 连续接触线段。
  //    排列边的开线段内部覆盖数恒定（越过其他带边线只发生在排列顶点处），
  //    故将相邻的三重边沿同一直线串成极大线段，避免用两个端点代替整段接触线。
  const vtxKey = (p) => `${Math.round(p.x.toNumber() * 1e9)},${Math.round(p.y.toNumber() * 1e9)}`;
  const edgeKey = (k1, k2) => (k1 < k2 ? `${k1}|${k2}` : `${k2}|${k1}`);
  const vertMap = new Map();
  const addVert = (p) => { const k = vtxKey(p); if (!vertMap.has(k)) vertMap.set(k, p); };
  for (const c of cellRecs) for (const p of c.vertices) addVert(p);
  for (const p of workarea) addVert(p);
  for (const s of strips) for (const p of s.corners) addVert(p);

  // 正面积三重单元的边/顶点索引：排列边或顶点若邻接三重单元，即已由区域风险表达。
  // 排列的凸单元之间只共享整条边或顶点，故集合判定等价于闭集包含（且 O(1)）。
  const tripleEdgeKeys = new Set();
  const tripleVertKeys = new Set();
  for (const c of tripleCells) {
    for (const p of c.vertices) tripleVertKeys.add(vtxKey(p));
    for (let i = 0; i < c.vertices.length; i++) {
      tripleEdgeKeys.add(edgeKey(vtxKey(c.vertices[i]), vtxKey(c.vertices[(i + 1) % c.vertices.length])));
    }
  }

  // 顶点闭集覆盖数
  const vtxCover = new Map(); // key → { covering }
  let vertexMax = 0;
  for (const [k, v] of vertMap) {
    if (!convexContains(workarea, v, vtxTol)) continue;
    const covering = [];
    strips.forEach((s, i) => {
      if (rectContains(s.edges, v, vtxTol)) covering.push(i + 1);
    });
    vtxCover.set(k, covering);
    if (covering.length > vertexMax) vertexMax = covering.length;
  }

  // 收集落在覆盖带边线上的排列边（去重无向边）
  const edgeMap = new Map();
  for (const c of cellRecs) {
    for (let i = 0; i < c.vertices.length; i++) {
      const p = c.vertices[i];
      const q = c.vertices[(i + 1) % c.vertices.length];
      const k1 = vtxKey(p);
      const k2 = vtxKey(q);
      if (k1 === k2) continue;
      const ek = edgeKey(k1, k2);
      if (edgeMap.has(ek)) continue;
      const line = stripLines.find((L) =>
        lineValue(L, p).abs().lte(lineTol) && lineValue(L, q).abs().lte(lineTol));
      if (!line) continue; // 仅为工作区边或剖分内部边：不可能产生零面积三重接触
      edgeMap.set(ek, { p, q, line, key: ek });
    }
  }

  // 每条边中点闭集计数；落在正面积三重单元内的边已由区域风险表达，跳过
  for (const e of edgeMap.values()) {
    const mid = { x: e.p.x.plus(e.q.x).div(2), y: e.p.y.plus(e.q.y).div(2) };
    const covering = [];
    strips.forEach((s, i) => {
      if (rectContains(s.edges, mid, vtxTol)) covering.push(i + 1);
    });
    e.covering = covering;
    // 中点覆盖数 <3 时不可能邻接三重单元；否则先查三重单元边集合（O(1)），
    // 未命中再以闭集包含兜底处理容差边界
    e.inTripleCell = covering.length >= 3
      && (tripleEdgeKeys.has(e.key)
        || tripleCells.some((c) => convexContains(c.vertices, mid, vtxTol)));
  }

  // 沿同一条边线，把中点覆盖数 ≥3 的边串成连通分量 → 接触线段
  const lineDir = (L) => ({ x: L.b.neg(), y: L.a }); // 单位法向 (a,b) → 单位方向
  const contactSegments = [];
  const edgesByLine = new Map();
  for (const e of edgeMap.values()) {
    if (e.covering.length < 3 || e.inTripleCell) continue;
    if (!edgesByLine.has(e.line)) edgesByLine.set(e.line, []);
    edgesByLine.get(e.line).push(e);
  }
  for (const [L, eds] of edgesByLine) {
    const d = lineDir(L);
    const tOf = (p) => p.x.mul(d.x).plus(p.y.mul(d.y));
    const vToEdges = new Map();
    eds.forEach((e, idx) => {
      for (const k of [vtxKey(e.p), vtxKey(e.q)]) {
        if (!vToEdges.has(k)) vToEdges.set(k, []);
        vToEdges.get(k).push(idx);
      }
    });
    const used = new Array(eds.length).fill(false);
    for (let s0 = 0; s0 < eds.length; s0++) {
      if (used[s0]) continue;
      const comp = [];
      const stack = [s0];
      used[s0] = true;
      while (stack.length) {
        const idx = stack.pop();
        comp.push(eds[idx]);
        for (const k of [vtxKey(eds[idx].p), vtxKey(eds[idx].q)]) {
          for (const j of vToEdges.get(k)) {
            if (!used[j]) { used[j] = true; stack.push(j); }
          }
        }
      }
      // 连通分量沿直线的极值点即线段端点
      let pLo = null;
      let pHi = null;
      let tLo = null;
      let tHi = null;
      const stripSet = new Set();
      let mult = 0;
      const consider = (p) => {
        const t = tOf(p);
        if (tLo === null || t.lt(tLo)) { tLo = t; pLo = p; }
        if (tHi === null || t.gt(tHi)) { tHi = t; pHi = p; }
      };
      for (const e of comp) {
        consider(e.p); consider(e.q);
        e.covering.forEach((n) => stripSet.add(n));
        if (e.covering.length > mult) mult = e.covering.length;
      }
      // 端点处可能有额外覆盖带以角点相抵，层数取闭线段上的最大值
      for (const p of [pLo, pHi]) {
        const cov = vtxCover.get(vtxKey(p));
        if (cov && cov.length > mult) mult = cov.length;
        if (cov) cov.forEach((n) => stripSet.add(n));
      }
      contactSegments.push({
        line: L, pLo, pHi, tLo, tHi,
        multiplicity: mult,
        strips: [...stripSet].sort((a, b) => a - b),
      });
    }
  }
  let edgeMax = 0;
  for (const e of edgeMap.values()) {
    if (!e.inTripleCell && e.covering.length > edgeMax) edgeMax = e.covering.length;
  }
  if (vertexMax > stats.maxMultiplicity) stats.maxMultiplicity = vertexMax;
  if (edgeMax > stats.maxMultiplicity) stats.maxMultiplicity = edgeMax;

  // 孤立三重接触点：覆盖数 ≥3、不在三重区域内、且不落在任何接触线段上
  const contacts = [];
  const pointOnSegment = (p, seg) => {
    if (lineValue(seg.line, p).abs().gt(lineTol)) return false;
    const t = lineDir(seg.line).x.mul(p.x).plus(lineDir(seg.line).y.mul(p.y));
    return t.gte(seg.tLo.minus(vtxTol)) && t.lte(seg.tHi.plus(vtxTol));
  };
  for (const [k, v] of vertMap) {
    const covering = vtxCover.get(k);
    if (!covering || covering.length < 3) continue;
    // 顶点集合查找 O(1)，未命中再以闭集包含兜底
    if (tripleVertKeys.has(k)) continue;
    if (tripleCells.some((c) => convexContains(c.vertices, v, vtxTol))) continue;
    if (contactSegments.some((seg) => pointOnSegment(v, seg))) continue;
    contacts.push({ point: v, covering });
  }

  // 5) 边界证据：单元边 ↔ 边线归属匹配
  const allLines = [...stripLines, ...waLines];
  const ownerLabel = (o) => (o.workarea ? `工作区·边${o.edge + 1}` : `R${o.strip}·${o.side}`);
  function boundaryOf(verts) {
    const labels = new Set();
    for (let i = 0; i < verts.length; i++) {
      const p = verts[i];
      const q = verts[(i + 1) % verts.length];
      const len2 = p.x.minus(q.x).pow(2).plus(p.y.minus(q.y).pow(2));
      if (len2.lte(lineTol.mul(lineTol))) continue;
      for (const L of allLines) {
        if (lineValue(L, p).abs().lte(lineTol) && lineValue(L, q).abs().lte(lineTol)) {
          for (const o of L.owners) labels.add(ownerLabel(o));
        }
      }
    }
    return [...labels];
  }
  function pointEvidence(p) {
    const labels = new Set();
    for (const L of allLines) {
      if (lineValue(L, p).abs().lte(lineTol)) for (const o of L.owners) labels.add(ownerLabel(o));
    }
    return [...labels];
  }
  function segmentEvidence(seg) {
    const labels = new Set();
    // 承载接触线的覆盖带边线（三条带的边界证据均保留）
    for (const o of seg.line.owners) labels.add(ownerLabel(o));
    // 与接触线共线的工作区边
    for (const L of waLines) {
      if (sameLine(L, seg.line, eps, lineTol)
        && lineValue(L, seg.pLo).abs().lte(lineTol)
        && lineValue(L, seg.pHi).abs().lte(lineTol)) {
        for (const o of L.owners) labels.add(ownerLabel(o));
      }
    }
    // 端点处相抵的其他带边线/工作区边
    for (const p of [seg.pLo, seg.pHi]) {
      for (const label of pointEvidence(p)) labels.add(label);
    }
    return [...labels];
  }

  // 6) 风险区域汇总
  const num = (d) => d.toNumber();
  const vertsOf = (c) => c.vertices.map((p) => [num(p.x), num(p.y)]);
  const risks = [];
  gapCells.forEach((c, i) => {
    risks.push({
      id: `G${i + 1}`,
      kind: 'gap',
      shape: 'region',
      multiplicity: 0,
      area: num(c.area),
      representative: [num(c.centroid.x), num(c.centroid.y)],
      vertices: vertsOf(c),
      boundary: boundaryOf(c.vertices),
      strips: [],
    });
  });
  tripleCells.forEach((c, i) => {
    risks.push({
      id: `T${i + 1}`,
      kind: 'triple',
      shape: 'region',
      multiplicity: c.count,
      area: num(c.area),
      representative: [num(c.centroid.x), num(c.centroid.y)],
      vertices: vertsOf(c),
      boundary: boundaryOf(c.vertices),
      strips: c.covering,
    });
  });
  contacts.forEach((ct, i) => {
    const p = [num(ct.point.x), num(ct.point.y)];
    risks.push({
      id: `P${i + 1}`,
      kind: 'triple',
      shape: 'point',
      multiplicity: ct.covering.length,
      area: 0,
      representative: p,
      vertices: [p],
      boundary: pointEvidence(ct.point),
      strips: ct.covering,
    });
  });
  contactSegments.forEach((seg, i) => {
    const lo = [num(seg.pLo.x), num(seg.pLo.y)];
    const hi = [num(seg.pHi.x), num(seg.pHi.y)];
    risks.push({
      id: `L${i + 1}`,
      kind: 'triple',
      shape: 'segment',
      multiplicity: seg.multiplicity,
      area: 0,
      representative: [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2],
      endpoints: [lo, hi],
      vertices: [lo, hi],
      boundary: segmentEvidence(seg),
      strips: seg.strips,
    });
  });

  // 首个风险区域：漏拍优先于三重曝光，再按最小顶点字典序（x 小者优先，再 y）
  const lexMin = (verts) => {
    let mx = Infinity;
    let my = Infinity;
    for (const [x, y] of verts) {
      if (x < mx - 1e-12 || (Math.abs(x - mx) <= 1e-12 && y < my)) { mx = x; my = y; }
    }
    return [mx, my];
  };
  const kindOrder = { gap: 0, triple: 1 };
  const sorted = [...risks].sort((a, b) => {
    if (kindOrder[a.kind] !== kindOrder[b.kind]) return kindOrder[a.kind] - kindOrder[b.kind];
    const ka = lexMin(a.vertices);
    const kb = lexMin(b.vertices);
    return ka[0] - kb[0] || ka[1] - kb[1];
  });
  const firstRisk = sorted[0] ?? null;

  const coveredArea = stats.workArea.minus(stats.gapArea);
  const report = {
    ok: risks.length === 0,
    errors: [],
    stats: {
      stripCount: stats.stripCount,
      vertexCount: stats.vertexCount,
      cellCount: stats.cellCount,
      workArea: num(stats.workArea),
      coveredArea: num(coveredArea),
      coverageRatio: num(coveredArea.div(stats.workArea)),
      gapArea: num(stats.gapArea),
      tripleArea: stats.tripleArea.toNumber(),
      singleArea: stats.singleArea.toNumber(),
      doubleArea: stats.doubleArea.toNumber(),
      maxMultiplicity: stats.maxMultiplicity,
    },
    firstRisk,
    risks: sorted,
    gaps: sorted.filter((r) => r.kind === 'gap'),
    triples: sorted.filter((r) => r.kind === 'triple'),
    // 渲染与明细数据
    workarea: input.workarea.map(([x, y]) => [x, y]),
    strips: strips.map((s) => ({
      index: s.index,
      ...s.params,
      corners: s.corners.map((p) => [num(p.x), num(p.y)]),
      area: s.params.w * s.params.h,
    })),
    cells: cellRecs.map((c) => ({ vertices: vertsOf(c), count: c.count })),
  };
  return report;
}
