package coordbook

import kotlin.math.PI

/** A calibration transform, versioned independently of the URDF. */
data class Calibration(
    val id: Long,
    val serial: String,
    val name: String,
    val parentFrame: String,
    val childFrame: String,
    val xyz: DoubleArray,
    val rpy: DoubleArray,
    val angleUnit: String, // "rad" | "deg"
    val validFrom: Long?, // epoch seconds, inclusive
    val validTo: Long?,   // epoch seconds, inclusive
    val version: Int,
    val status: String = "approved",
) {
    fun transform(): Transform {
        val r = if (angleUnit == "deg") DoubleArray(3) { rpy[it] * PI / 180.0 } else rpy
        return Transform.fromXyzRpy(xyz, r)
    }

    fun activeAt(time: Long?): String = when {
        time == null && (validFrom != null || validTo != null) -> "inactive:no-query-time"
        time == null -> "active"
        validFrom != null && time < validFrom -> "inactive:before-valid-from"
        validTo != null && time > validTo -> "inactive:after-valid-to"
        (validFrom != null && time == validFrom) || (validTo != null && time == validTo) -> "active:boundary-inclusive"
        else -> "active"
    }
}

data class Edge(
    val id: String,
    val from: String,
    val to: String,
    val kind: String,          // "joint" | "calibration"
    val source: String,        // "urdf" | "calibration:<name>@v<n>"
    val jointName: String? = null,
    val jointType: String? = null,
    val value: Double? = null, // joint value applied (rad or m)
    val valueUnit: String? = null, // "rad" | "m"
    val state: String = "ok",  // "ok" | "missing_joint_value" | "mimic_cycle"
    val transform: Transform? = null,
    val duplicate: Boolean = false,
)

data class GraphContext(
    val robot: UrdfRobot,
    val serial: String,
    val jointValues: Map<String, Double>, // radians / meters, URDF units
    val calibrations: List<Calibration>,
    val time: Long?,
)

object Kinematics {

    /** Resolve joint values including mimic chains; detects mimic cycles. */
    fun resolveJointValues(robot: UrdfRobot, given: Map<String, Double>): Map<String, Pair<Double?, String>> {
        val byName = robot.joints.associateBy { it.name }
        val cache = mutableMapOf<String, Pair<Double?, String>>()
        fun resolve(name: String, visiting: Set<String>): Pair<Double?, String> {
            cache[name]?.let { return it }
            given[name]?.let { return (it to "ok").also { v -> cache[name] = v } }
            val j = byName[name] ?: return (null to "missing_joint_value").also { v -> cache[name] = v }
            if (j.type == "fixed") return (0.0 to "ok").also { v -> cache[name] = v }
            val m = j.mimic ?: return (null to "missing_joint_value").also { v -> cache[name] = v }
            if (m.joint in visiting || m.joint == name) {
                return (null to "mimic_cycle").also { v -> cache[name] = v }
            }
            val (ref, st) = resolve(m.joint, visiting + name)
            val out = if (ref == null) null to (if (st == "mimic_cycle") "mimic_cycle" else "missing_joint_value")
            else (m.multiplier * ref + m.offset) to "ok"
            cache[name] = out
            return out
        }
        for (j in robot.joints) resolve(j.name, setOf(j.name))
        return cache
    }

    fun buildEdges(ctx: GraphContext): List<Edge> {
        val values = resolveJointValues(ctx.robot, ctx.jointValues)
        val edges = mutableListOf<Edge>()
        for (j in ctx.robot.joints) {
            val (v, st) = values[j.name] ?: (null to "missing_joint_value")
            val unit = if (j.type == "prismatic") "m" else "rad"
            val t = when {
                v == null -> null
                j.type == "fixed" -> j.origin.toTransform()
                j.type == "prismatic" -> j.origin.toTransform().compose(Transform.translation(
                    doubleArrayOf(j.axis[0] * v, j.axis[1] * v, j.axis[2] * v)))
                else -> j.origin.toTransform().compose(Transform.rotationAboutAxis(j.axis, v))
            }
            edges.add(
                Edge(
                    id = "joint:${j.name}", from = j.parent, to = j.child,
                    kind = "joint", source = "urdf", jointName = j.name, jointType = j.type,
                    value = v, valueUnit = unit, state = st, transform = t,
                )
            )
        }
        for (c in ctx.calibrations) {
            if (c.serial != ctx.serial) continue
            val st = c.activeAt(ctx.time)
            if (!st.startsWith("active")) continue
            edges.add(
                Edge(
                    id = "cal:${c.id}", from = c.parentFrame, to = c.childFrame,
                    kind = "calibration", source = "calibration:${c.name}@v${c.version}",
                    value = null, valueUnit = c.angleUnit, state = st, transform = c.transform(),
                )
            )
        }
        // Duplicate declaration: same unordered frame pair from multiple sources.
        val groups = edges.groupBy { setOf(it.from, it.to) }
        return edges.map { e ->
            val g = groups[setOf(e.from, e.to)]!!
            if (g.size > 1) e.copy(duplicate = true) else e
        }
    }

    data class PathStep(val edge: Edge, val forward: Boolean)
    data class PathCandidate(
        val steps: List<PathStep>,
        val transform: Transform?,
        val state: String, // "ok" | "incomplete:<reason>"
        val errorTranslation: Double, // max residual vs other complete candidates
        val errorRotation: Double,
    )

    /** Enumerate ALL simple paths (bounded), keeping consistent duplicates as并列 candidates. */
    fun findPaths(edges: List<Edge>, from: String, to: String, maxDepth: Int = 8, maxPaths: Int = 128): List<PathCandidate> {
        if (from == to) return listOf(PathCandidate(emptyList(), Transform.IDENTITY, "ok", 0.0, 0.0))
        val adj = mutableMapOf<String, MutableList<Pair<Edge, Boolean>>>()
        for (e in edges.sortedBy { it.id }) {
            adj.getOrPut(e.from) { mutableListOf() }.add(e to true)
            adj.getOrPut(e.to) { mutableListOf() }.add(e to false)
        }
        val raw = mutableListOf<List<PathStep>>()
        fun dfs(cur: String, visited: MutableSet<String>, acc: MutableList<PathStep>) {
            if (raw.size >= maxPaths || acc.size >= maxDepth) return
            for ((e, fwd) in adj[cur] ?: return) {
                val nxt = if (fwd) e.to else e.from
                if (nxt in visited) continue
                acc.add(PathStep(e, fwd))
                if (nxt == to) raw.add(acc.toList())
                else {
                    visited.add(nxt)
                    dfs(nxt, visited, acc)
                    visited.remove(nxt)
                }
                acc.removeAt(acc.size - 1)
                if (raw.size >= maxPaths) return
            }
        }
        visited@ run {
            val v = mutableSetOf(from)
            dfs(from, v, mutableListOf())
        }
        val candidates = raw.map { steps ->
            var t = Transform.IDENTITY
            var state = "ok"
            for (s in steps) {
                val et = s.edge.transform
                if (et == null) {
                    state = "incomplete:${s.edge.state}"
                    break
                }
                t = t.compose(if (s.forward) et else et.inverse())
            }
            PathCandidate(steps, if (state == "ok") t else null, state, 0.0, 0.0)
        }
        val complete = candidates.filter { it.transform != null }
        return candidates.map { c ->
            if (c.transform == null || complete.size <= 1) c
            else {
                var maxT = 0.0; var maxR = 0.0
                for (o in complete) {
                    if (o === c) continue
                    val (dt, dr) = c.transform.residualTo(o.transform!!)
                    if (dt > maxT) maxT = dt
                    if (dr > maxR) maxR = dr
                }
                c.copy(errorTranslation = maxT, errorRotation = maxR)
            }
        }
    }

    data class CycleResidual(
        val edges: List<String>,      // edge ids around the cycle
        val closingEdge: String,      // non-tree edge that closes the cycle
        val translationError: Double,
        val rotationError: Double,    // radians
        val consistent: Boolean,
        val state: String,            // "ok" | "indeterminate:missing_joint_value"
    )

    /** Fundamental cycles vs a spanning forest; residual of composing around each cycle. */
    fun cycleResiduals(edges: List<Edge>, consistencyEps: Double = 1e-6): List<CycleResidual> {
        val nodes = edges.flatMap { listOf(it.from, it.to) }.distinct()
        val parent = nodes.associateWith { it }.toMutableMap()
        fun find(x: String): String {
            var r = x
            while (parent[r] != r) r = parent[r]!!
            var c = x
            while (parent[c] != c) { val n = parent[c]!!; parent[c] = r; c = n }
            return r
        }
        val treeAdj = mutableMapOf<String, MutableList<Pair<String, Edge>>>()
        val nonTree = mutableListOf<Edge>()
        for (e in edges) {
            val ra = find(e.from); val rb = find(e.to)
            if (ra != rb) {
                parent[ra] = rb
                treeAdj.getOrPut(e.from) { mutableListOf() }.add(e.to to e)
                treeAdj.getOrPut(e.to) { mutableListOf() }.add(e.from to e)
            } else nonTree.add(e)
        }
        fun treePath(a: String, b: String): List<Pair<Edge, Boolean>>? {
            // BFS in tree from a to b; returns steps (edge, forward)
            val prev = mutableMapOf<String, Pair<String, Edge>?>()
            val q = ArrayDeque<String>(); q.add(a); prev[a] = null
            while (q.isNotEmpty()) {
                val u = q.removeFirst()
                if (u == b) break
                for ((v, e) in treeAdj[u] ?: emptyList()) {
                    if (v !in prev) { prev[v] = u to e; q.add(v) }
                }
            }
            if (b !in prev) return null
            val steps = mutableListOf<Pair<Edge, Boolean>>()
            var cur = b
            while (cur != a) {
                val (p, e) = prev[cur]!!
                steps.add(e to (e.from == p && e.to == cur))
                cur = p
            }
            return steps.reversed()
        }
        val out = mutableListOf<CycleResidual>()
        for (e in nonTree) {
            val path = treePath(e.from, e.to) ?: continue
            val cycleEdges = path.map { it.first.id } + e.id
            var t = Transform.IDENTITY
            var state = "ok"
            // walk tree from e.from to e.to, then close the loop traversing e backwards
            val allSteps = path + (e to false)
            for ((edge, fwd) in allSteps) {
                val et = edge.transform
                if (et == null) { state = "indeterminate:${edge.state}"; break }
                t = t.compose(if (fwd) et else et.inverse())
            }
            if (state != "ok") {
                out.add(CycleResidual(cycleEdges, e.id, Double.NaN, Double.NaN, false, state))
            } else {
                val te = t.translationNorm(); val re = t.rotationAngle()
                out.add(CycleResidual(cycleEdges, e.id, te, re, te < consistencyEps && re < consistencyEps, "ok"))
            }
        }
        return out
    }

    /** Minimum hitting set of edges covering all inconsistent cycles (exact for small inputs). */
    fun minimalContradictionEdgeSet(cycles: List<CycleResidual>): List<String> {
        val bad = cycles.filter { !it.consistent && it.state == "ok" }.map { it.edges.toSet() }
        if (bad.isEmpty()) return emptyList()
        // prefer blaming re-declaration (calibration) edges over original URDF joints
        val universe = bad.flatten().distinct().sortedBy { if (it.startsWith("cal:")) 0 else 1 }
        for (size in 1..minOf(4, universe.size)) {
            val found = combinations(universe, size).firstOrNull { cand ->
                bad.all { cycle -> cycle.any { it in cand } }
            }
            if (found != null) return found.sorted()
        }
        // greedy fallback
        val remaining = bad.toMutableList()
        val chosen = mutableListOf<String>()
        while (remaining.isNotEmpty()) {
            val best = universe.maxByOrNull { e -> remaining.count { e in it } }!!
            chosen.add(best)
            remaining.removeAll { best in it }
        }
        return chosen.sorted()
    }

    private fun combinations(items: List<String>, k: Int): List<Set<String>> {
        val out = mutableListOf<Set<String>>()
        fun rec(start: Int, acc: MutableList<String>) {
            if (acc.size == k) { out.add(acc.toSet()); return }
            for (i in start until items.size) { acc.add(items[i]); rec(i + 1, acc); acc.removeAt(acc.size - 1) }
        }
        rec(0, mutableListOf())
        return out
    }
}
