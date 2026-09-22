package jcb

import kotlin.math.abs

data class CalTransform(
    val id: Long,
    val robotSerial: String,
    val parent: String,
    val child: String,
    val xyz: DoubleArray,
    val rpy: DoubleArray,
    val units: String, // "rad" or "deg"
    val validFrom: Long?, // epoch seconds, inclusive
    val validTo: Long?,   // epoch seconds, inclusive
    val version: Int,
) {
    fun rpyRad(): DoubleArray =
        if (units == "deg") rpy.map { Math.toRadians(it) }.toDoubleArray() else rpy

    fun transform(): Mat4 = Mat4.fromXyzRpy(xyz, rpyRad())

    fun validityAt(time: Long?): String = when {
        time == null -> "TIME_UNSPECIFIED"
        validFrom != null && time < validFrom -> "NOT_YET_VALID"
        validTo != null && time > validTo -> "EXPIRED"
        else -> "ACTIVE"
    }
}

sealed interface EdgeSource {
    val label: String
    data class Urdf(val docVersion: Int) : EdgeSource { override val label = "urdf:v$docVersion" }
    data class Calibration(val calId: Long, val version: Int) : EdgeSource {
        override val label = "calibration:$calId:v$version"
    }
}

data class Edge(
    val id: String,
    val parent: String,
    val child: String,
    val source: EdgeSource,
    val joint: JointDef?,        // null for calibration edges (treated as fixed)
    val cal: CalTransform?,      // null for URDF edges
) {
    val isMimic: Boolean get() = joint?.mimic != null

    fun forward(q: Double): Mat4 = when {
        cal != null -> cal.transform()
        joint != null -> {
            val origin = Mat4.fromXyzRpy(joint.xyz, joint.rpy)
            when (joint.type) {
                JointType.REVOLUTE, JointType.CONTINUOUS ->
                    origin.times(Mat4.axisAngle(joint.axis ?: doubleArrayOf(1.0, 0.0, 0.0), q))
                JointType.PRISMATIC -> {
                    val a = Mat4.norm(joint.axis ?: doubleArrayOf(1.0, 0.0, 0.0))
                    origin.times(Mat4.translation(a[0] * q, a[1] * q, a[2] * q))
                }
                else -> origin
            }
        }
        else -> Mat4.identity()
    }
}

/** Resolves joint values including mimic chains; detects mimic cycles. */
class JointResolver(val joints: Map<String, JointDef>, values: Map<String, Double>) {
    private val raw = values
    val mimicCycleJoints: List<String>
    private val resolved = HashMap<String, Double>()
    val missing = mutableListOf<String>()

    init {
        val visiting = mutableSetOf<String>()
        val cycle = linkedSetOf<String>()
        fun resolve(name: String): Double? {
            resolved[name]?.let { return it }
            raw[name]?.let { return it }
            val j = joints[name] ?: return null
            val m = j.mimic ?: return null
            if (name in visiting) {
                cycle.add(name)
                return null
            }
            visiting.add(name)
            val ref = resolve(m.joint)
            visiting.remove(name)
            if (ref == null) {
                if (joints[m.joint]?.mimic != null && m.joint !in raw) cycle.add(name)
                return null
            }
            val v = ref * m.multiplier + m.offset
            resolved[name] = v
            return v
        }
        for (name in joints.keys) resolve(name)
        // A mimic whose reference chain ends in an unresolved mimic is part of a cycle.
        mimicCycleJoints = joints.filter { (n, j) ->
            j.mimic != null && n !in resolved && n !in raw && dependsOnUnresolved(n, mutableSetOf())
        }.keys.toList()
    }

    private fun dependsOnUnresolved(name: String, seen: MutableSet<String>): Boolean {
        if (!seen.add(name)) return true // came back to start: cycle
        val j = joints[name] ?: return false
        val m = j.mimic ?: return false
        if (raw.containsKey(m.joint)) return false
        val ref = joints[m.joint] ?: return false
        if (ref.mimic == null) return raw.containsKey(m.joint).not()
        return dependsOnUnresolved(m.joint, seen)
    }

    fun value(name: String): Double? = resolved[name] ?: raw[name]
}

data class PathStep(
    val edgeId: String,
    val source: String,
    val from: String,
    val to: String,
    val jointName: String?,
    val jointValue: Double?,
    val jointUnit: String?, // "rad" | "m" | null for fixed
)

data class PathCandidate(
    val steps: List<PathStep>,
    val transform: Mat4,
    val missingJoints: List<String>,
    val inactiveCalibrations: List<String>,
    var error: Double = 0.0,
) {
    val status: String
        get() = when {
            missingJoints.isNotEmpty() -> "MISSING_JOINT_VALUES"
            inactiveCalibrations.isNotEmpty() -> "CALIBRATION_INACTIVE"
            else -> "OK"
        }
}

data class LoopReport(
    val edgeIds: List<String>,
    val residual: Double,
    val translationResidual: Double,
    val rotationResidualRad: Double,
    val consistent: Boolean,
    val minimalConflictEdges: List<String>,
)

data class QueryResult(
    val from: String,
    val to: String,
    val status: String,
    val candidates: List<PathCandidate>,
    val mimicCycles: List<String>,
)

class FrameGraph(
    val edges: List<Edge>,
    val joints: Map<String, JointDef>,
    val tolerance: Double = 1e-6,
) {
    val frames: List<String> = (edges.flatMap { listOf(it.parent, it.child) }).distinct().sorted()
    private val adjacency: Map<String, List<Pair<Edge, Boolean>>> = buildMap {
        for (e in edges) {
            getOrPut(e.parent) { mutableListOf() }.let { (it as MutableList).add(e to true) }
            getOrPut(e.child) { mutableListOf() }.let { (it as MutableList).add(e to false) }
        }
    }

    fun jointUnit(e: Edge): String? = when (e.joint?.type) {
        JointType.REVOLUTE, JointType.CONTINUOUS -> "rad"
        JointType.PRISMATIC -> "m"
        else -> null
    }

    /** Enumerate all simple paths from -> to (bounded), keeping every candidate. */
    fun query(from: String, to: String, resolver: JointResolver, time: Long?): QueryResult {
        if (from !in frames || to !in frames) {
            return QueryResult(from, to, "UNKNOWN_FRAME", emptyList(), resolver.mimicCycleJoints)
        }
        val maxDepth = edges.size + 1
        val paths = mutableListOf<List<Pair<Edge, Boolean>>>()
        fun dfs(node: String, visited: MutableSet<String>, acc: MutableList<Pair<Edge, Boolean>>) {
            if (acc.size > maxDepth) return
            if (node == to) { paths.add(acc.toList()); return }
            for ((e, fwd) in adjacency[node].orEmpty()) {
                val next = if (fwd) e.child else e.parent
                if (next in visited) continue
                visited.add(next); acc.add(e to fwd)
                dfs(next, visited, acc)
                acc.removeAt(acc.size - 1); visited.remove(next)
            }
        }
        dfs(from, mutableSetOf(from), mutableListOf())
        if (paths.isEmpty()) return QueryResult(from, to, "NO_PATH", emptyList(), resolver.mimicCycleJoints)

        val candidates = paths.map { path ->
            var t = Mat4.identity()
            val steps = mutableListOf<PathStep>()
            val missingJ = mutableListOf<String>()
            val inactiveC = mutableListOf<String>()
            for ((e, fwd) in path) {
                var m: Mat4
                var jv: Double? = null
                if (e.cal != null) {
                    val v = e.cal.validityAt(time)
                    if (v != "ACTIVE") inactiveC.add("${e.cal.id}:$v")
                    m = e.forward(0.0)
                } else {
                    val jn = e.joint!!.name
                    val needsQ = e.joint.type in setOf(JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC)
                    if (needsQ) {
                        val q = resolver.value(jn)
                        if (q == null) { missingJ.add(jn); jv = null } else jv = q
                        m = e.forward(q ?: 0.0)
                    } else {
                        m = e.forward(0.0)
                    }
                }
                if (!fwd) m = m.inverse()
                t = t.times(m)
                steps.add(
                    PathStep(
                        edgeId = e.id,
                        source = e.source.label,
                        from = if (fwd) e.parent else e.child,
                        to = if (fwd) e.child else e.parent,
                        jointName = e.joint?.name,
                        jointValue = jv,
                        jointUnit = jointUnit(e),
                    )
                )
            }
            PathCandidate(steps, t, missingJ.distinct(), inactiveC.distinct())
        }
        // Error of a candidate = max deviation to any other candidate (kept, not collapsed).
        for (c in candidates) {
            c.error = candidates.filter { it !== c }.maxOfOrNull { it.transform.deviation(c.transform) } ?: 0.0
        }
        val sorted = candidates.sortedWith(compareBy({ it.status != "OK" }, { it.error }, { it.steps.first().edgeId }))
        val status = when {
            sorted.any { it.status == "OK" } -> "OK"
            sorted.any { it.status == "MISSING_JOINT_VALUES" } -> "MISSING_JOINT_VALUES"
            else -> sorted.first().status
        }
        return QueryResult(from, to, status, sorted, resolver.mimicCycleJoints)
    }

    /** Fundamental cycles w.r.t. a spanning forest; residual per loop; minimal conflict edge set. */
    fun loops(resolver: JointResolver, time: Long?): List<LoopReport> {
        val parent = HashMap<String, String>()
        val parentEdge = HashMap<String, Edge>()
        val visited = mutableSetOf<String>()
        val treeEdges = mutableSetOf<String>()
        for (start in frames) {
            if (start in visited) continue
            visited.add(start)
            val queue = ArrayDeque<String>(); queue.add(start)
            while (queue.isNotEmpty()) {
                val n = queue.removeFirst()
                for ((e, fwd) in adjacency[n].orEmpty()) {
                    val next = if (fwd) e.child else e.parent
                    if (next !in visited) {
                        visited.add(next)
                        parent[next] = n
                        parentEdge[next] = e
                        treeEdges.add(e.id)
                        queue.add(next)
                    }
                }
            }
        }
        val reports = mutableListOf<LoopReport>()
        for (e in edges) {
            if (e.id in treeEdges) continue
            val lca = lca(e.parent, e.child, parent)
            val cycleEdges = mutableListOf<Pair<Edge, Boolean>>()
            var node = e.parent
            while (node != lca) {
                val pe = parentEdge[node]!!
                cycleEdges.add(pe to (pe.child == node))
                node = parent[node]!!
            }
            val down = mutableListOf<Pair<Edge, Boolean>>()
            node = e.child
            while (node != lca) {
                val pe = parentEdge[node]!!
                down.add(0, pe to (pe.parent == node))
                node = parent[node]!!
            }
            cycleEdges.addAll(down)
            cycleEdges.add(e to true)
            reports.add(buildReport(cycleEdges, resolver))
        }
        return reports
    }

    private fun lca(a: String, b: String, parent: Map<String, String>): String {
        val ancestors = mutableSetOf<String>()
        var n: String? = a
        while (n != null) { ancestors.add(n); n = parent[n] }
        n = b
        while (n != null && n !in ancestors) n = parent[n]
        return n ?: a
    }

    private fun edgeTransform(e: Edge, resolver: JointResolver): Mat4 {
        val q = if (e.joint != null) resolver.value(e.joint.name) ?: 0.0 else 0.0
        return e.forward(q)
    }

    private fun loopTransform(cycle: List<Pair<Edge, Boolean>>, resolver: JointResolver): Mat4 {
        var t = Mat4.identity()
        for ((e, fwd) in cycle) {
            var m = edgeTransform(e, resolver)
            if (!fwd) m = m.inverse()
            t = t.times(m)
        }
        return t
    }

    private fun residualOf(m: Mat4): Triple<Double, Double, Double> {
        val dx = m.r[0][3]; val dy = m.r[1][3]; val dz = m.r[2][3]
        val trans = kotlin.math.sqrt(dx * dx + dy * dy + dz * dz)
        val trace = m.r[0][0] + m.r[1][1] + m.r[2][2]
        val cosA = ((trace - 1.0) / 2.0).coerceIn(-1.0, 1.0)
        val rot = abs(kotlin.math.acos(cosA))
        return Triple(trans + rot, trans, rot)
    }

    private fun buildReport(cycle: List<Pair<Edge, Boolean>>, resolver: JointResolver): LoopReport {
        val (res, tres, rres) = residualOf(loopTransform(cycle, resolver))
        val consistent = res <= tolerance
        val minimal = if (consistent) emptyList() else minimalConflictSet(cycle, resolver)
        return LoopReport(
            edgeIds = cycle.map { it.first.id },
            residual = res,
            translationResidual = tres,
            rotationResidualRad = rres,
            consistent = consistent,
            minimalConflictEdges = minimal,
        )
    }

    /** Minimal set of edges that explains the contradiction: an edge is the
     *  culprit if the rest of the loop reproduces that edge's own transform
     *  within tolerance. If no single edge explains it, the whole loop is
     *  reported as contradictory. */
    private fun minimalConflictSet(cycle: List<Pair<Edge, Boolean>>, resolver: JointResolver): List<String> {
        for (k in cycle.indices) {
            var t = Mat4.identity()
            for (idx in cycle.indices) {
                if (idx == k) continue
                val (e, fwd) = cycle[idx]
                var m = edgeTransform(e, resolver)
                if (!fwd) m = m.inverse()
                t = t.times(m)
            }
            val (ek, fwdk) = cycle[k]
            val own = edgeTransform(ek, resolver)
            val expected = if (fwdk) own.inverse() else own
            if (t.deviation(expected) <= tolerance) return listOf(ek.id)
        }
        return cycle.map { it.first.id }
    }
}
