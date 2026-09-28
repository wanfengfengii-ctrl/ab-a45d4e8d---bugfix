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

  // 4) 闭集检查：捕获零面积三重接触（接触点 / 接触线段）。
  //    仅查排列顶点会把「整条共边三重接触」误拆成两个端点接触点，
  //    因此还要沿覆盖带边线上的单元边扫描开线段中点，并把共线、
  //    同覆盖集合的相邻区间合并为极大接触线段。
  const vtxKey = (p) => `${Math.round(p.x.toNumber() * 1e9)},${Math.round(p.y.toNumber() * 1e9)}`;
  const vertMap = new Map();
  const addVert = (p) => { const k = vtxKey(p); if (!vertMap.has(k)) vertMap.set(k, p); };
  for (const c of cellRecs) for (const p of c.vertices) addVert(p);
  for (const p of workarea) addVert(p);
  for (const s of strips) for (const p of s.corners) addVert(p);

  const coveringAt = (p, tol) => {
    const covering = [];
    strips.forEach((s, i) => {
      if (rectContains(s.edges, p, tol)) covering.push(i + 1);
    });
    return covering;
  };

  // 预索引正面积三重单元的顶点键 / 边键：排列中属于三重单元闭包的顶点与边
  // 直接并入三重区域（接触点/接触线段只统计三重区域之外的零面积接触），
  // 避免对每个候选点做 O(三重单元数) 的 Decimal 包含判定。
  const edgeKeyOf = (p, q) => {
    const pk = vtxKey(p);
    const qk = vtxKey(q);
    return pk < qk ? `${pk}|${qk}` : `${qk}|${pk}`;
  };
  const tripleVtxKeys = new Set();
  const tripleEdgeKeys = new Set();
  for (const c of tripleCells) {
    for (const p of c.vertices) tripleVtxKeys.add(vtxKey(p));
    for (let i = 0; i < c.vertices.length; i++) {
      tripleEdgeKeys.add(edgeKeyOf(c.vertices[i], c.vertices[(i + 1) % c.vertices.length]));
    }
  }

  // 4a) 排列顶点上的闭集接触
  const contacts = [];
  let contactMax = 0;
  for (const v of vertMap.values()) {
    if (!convexContains(workarea, v, vtxTol)) continue;
    const covering = coveringAt(v, vtxTol);
    if (covering.length > contactMax) contactMax = covering.length;
    if (covering.length >= 3 && !tripleVtxKeys.has(vtxKey(v))) {
      contacts.push({ point: v, covering });
    }
  }

  // 4b) 单元边（落在覆盖带边线上的）中点闭集计数 → 接触区间
  //     以合并后的覆盖带边线为一组，沿直线用单位方向参数 t 记录区间。
  //     性能：先用数值（number）粗筛共线，再用 Decimal 精确确认；
  //     相邻单元共享边，按端点键去重后只处理一次。
  const stripLinesNum = stripLines.map((sl) => ({
    owner: sl, a: sl.a.toNumber(), b: sl.b.toNumber(), c: sl.c.toNumber(),
  }));
  const numScreen = 1e-9 * Math.max(1, scale);
  const edgeSeen = new Set();
  const segGroups = new Map(); // stripLines 元素 → { intervals:[{t0,t1,covering}] }
  for (const c of cellRecs) {
    const n = c.vertices.length;
    for (let i = 0; i < n; i++) {
      const p = c.vertices[i];
      const q = c.vertices[(i + 1) % n];
      const len2 = p.x.minus(q.x).pow(2).plus(p.y.minus(q.y).pow(2));
      if (len2.lte(lineTol.mul(lineTol))) continue;
      const pk = vtxKey(p);
      const qk = vtxKey(q);
      const edgeKey = pk < qk ? `${pk}|${qk}` : `${qk}|${pk}`;
      if (edgeSeen.has(edgeKey)) continue;
      edgeSeen.add(edgeKey);
      // 数值粗筛：先用 number 构造单位法向，只有接近某条覆盖带边线时才做 Decimal 精确确认。
      // 同时尝试两种定向，避免近竖直/近水平线在数值与 Decimal 规范定向之间符号不一致而漏配。
      const npx = p.x.toNumber();
      const npy = p.y.toNumber();
      const nqx = q.x.toNumber();
      const nqy = q.y.toNumber();
      let ndx = nqx - npx;
      let ndy = nqy - npy;
      const nlen = Math.hypot(ndx, ndy);
      ndx /= nlen; ndy /= nlen;
      const na0 = -ndy;
      const nb0 = ndx;
      const nc0 = na0 * npx + nb0 * npy;
      const matchesOrient = (na, nb, nc) => stripLinesNum.find((sl) =>
        Math.abs(sl.a - na) <= numScreen &&
        Math.abs(sl.b - nb) <= numScreen &&
        Math.abs(sl.c - nc) <= numScreen);
      const candidate = matchesOrient(na0, nb0, nc0) || matchesOrient(-na0, -nb0, -nc0);
      if (!candidate) continue; // 绝大多数单元边与覆盖带边线无关：廉价跳过
      const L = canonical(lineFromPoints(p, q));
      if (!sameLine(candidate.owner, L, eps, lineTol)) continue;
      const owner = candidate.owner;
      const mid = { x: p.x.plus(q.x).div(2), y: p.y.plus(q.y).div(2) };
      const covering = coveringAt(mid, eps);
      if (covering.length > contactMax) contactMax = covering.length;
      if (covering.length < 3) continue;
      // 邻接正面积三重单元的边归入三重区域，不另立零面积风险
      if (tripleEdgeKeys.has(edgeKey)) continue;
      // 单位方向 d=(-b,a)，直线上取原点投影 o=c(a,b)，参数 t=d·(r-o)
      const dx = L.b.neg();
      const dy = L.a;
      const ox = L.a.mul(L.c);
      const oy = L.b.mul(L.c);
      const tOf = (r) => dx.mul(r.x.minus(ox)).plus(dy.mul(r.y.minus(oy)));
      let t0 = tOf(p);
      let t1 = tOf(q);
      if (t1.lt(t0)) [t0, t1] = [t1, t0];
      let g = segGroups.get(owner);
      if (!g) { g = { line: L, dx, dy, ox, oy, intervals: [] }; segGroups.set(owner, g); }
      g.intervals.push({ t0, t1, covering });
    }
  }
  if (contactMax > stats.maxMultiplicity) stats.maxMultiplicity = contactMax;

  // 4c) 合并同一直线上首尾相接/重叠且覆盖集合相同的区间 → 极大接触线段
  const coverKey = (cv) => cv.join(',');
  const contactSegments = [];
  for (const g of segGroups.values()) {
    const merged = [];
    const its = g.intervals.sort((u, v) => (u.t0.lt(v.t0) ? -1 : u.t0.gt(v.t0) ? 1 : 0));
    for (const it of its) {
      const last = merged[merged.length - 1];
      if (last && coverKey(last.covering) === coverKey(it.covering) &&
          it.t0.lte(last.t1.plus(lineTol))) {
        if (it.t1.gt(last.t1)) last.t1 = it.t1;
      } else {
        merged.push({ ...it, covering: [...it.covering] });
      }
    }
    const pointAt = (t) => ({ x: g.ox.plus(g.dx.mul(t)), y: g.oy.plus(g.dy.mul(t)) });
    const tParam = (r) => g.dx.mul(r.x.minus(g.ox)).plus(g.dy.mul(r.y.minus(g.oy)));
    for (const m of merged) {
      if (m.t1.minus(m.t0).lte(lineTol)) continue; // 退化区间交给接触点
      contactSegments.push({
        from: pointAt(m.t0), to: pointAt(m.t1), line: g.line,
        t0: m.t0, t1: m.t1, tParam, covering: m.covering,
      });
    }
  }

  // 4d) 吸收端点接触点：覆盖集合与相邻接触线段一致时，端点只是线段的闭包端点，
  //     不得再把整段风险拆成两个点风险；端点另有额外覆盖带（层数更高）时保留点风险。
  const sameCover = (a, b) => a.length === b.length && a.every((v, i) => v === b[i]);
  const pointLiesOnSegment = (p, seg) => {
    if (lineValue(seg.line, p).abs().gt(lineTol)) return false;
    const t = seg.tParam(p);
    return t.gte(seg.t0.minus(lineTol)) && t.lte(seg.t1.plus(lineTol));
  };
  const loneContacts = contacts.filter((ct) => !contactSegments.some((seg) =>
    sameCover(ct.covering, seg.covering) && pointLiesOnSegment(ct.point, seg)));

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
    for (const L of allLines) {
      if (sameLine(L, seg.line, eps, lineTol)) {
        for (const o of L.owners) labels.add(ownerLabel(o));
      }
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
  contactSegments.forEach((seg, i) => {
    const a = [num(seg.from.x), num(seg.from.y)];
    const b = [num(seg.to.x), num(seg.to.y)];
    const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    risks.push({
      id: `L${i + 1}`,
      kind: 'triple',
      shape: 'segment',
      multiplicity: seg.covering.length,
      area: 0,
      representative: mid,
      vertices: [a, b],
      boundary: segmentEvidence(seg),
      strips: seg.covering,
    });
  });
  loneContacts.forEach((ct, i) => {
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
