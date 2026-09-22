package atlas.kinematics

import atlas.urdf.JointModel
import atlas.urdf.JointType
import atlas.urdf.Origin
import atlas.urdf.RobotModel
import atlas.urdf.Xyz
import kotlinx.serialization.Serializable

@Serializable
data class CalibrationTransform(
    val parent: String,
    val child: String,
    val xyz: List<Double>,
    val rpy: List<Double>
)

@Serializable
data class CalibrationSet(
    val id: Long,
    val name: String,
    val robotSerial: String? = null,
    val validFrom: Long? = null,
    val validTo: Long? = null,
    val version: Int = 1,
    val transforms: List<CalibrationTransform> = emptyList()
)

enum class JointValueState { FIXED, PROVIDED, MIMIC, MIMIC_CYCLE, MISSING, UNSUPPORTED_TYPE }

@Serializable
data class ResolvedJoint(
    val joint: String,
    val value: Double,
    val state: String,
    val source: String? = null
)

data class TransformEdge(
    val id: String,
    val parent: String,
    val child: String,
    val source: String, // "urdf" | "calibration"
    val jointName: String? = null,
    val jointType: JointType? = null,
    val origin: Origin = Origin(),
    val axis: Xyz? = null,
    val calibrationId: Long? = null,
    val calibrationName: String? = null,
    val robotSerial: String? = null,
    val validFrom: Long? = null,
    val validTo: Long? = null,
    val atValidityBoundary: Boolean = false,
    val duplicateDeclaration: Boolean = false
)

@Serializable
data class EdgeDetail(
    val id: String,
    val parent: String,
    val child: String,
    val forward: Boolean,
    val source: String,
    val jointName: String? = null,
    val jointType: String? = null,
    val jointValue: Double? = null,
    val jointValueState: String? = null,
    val calibrationName: String? = null,
    val atValidityBoundary: Boolean = false,
    val duplicateDeclaration: Boolean = false
)

@Serializable
data class PathCandidate(
    val index: Int,
    val frames: List<String>,
    val edges: List<EdgeDetail>,
    val matrix: List<Double>,
    val error: Double,
    val consistentWithOthers: Boolean
)

@Serializable
data class QueryResult(
    val from: String,
    val to: String,
    val status: String,
    val issues: List<String>,
    val joints: List<ResolvedJoint>,
    val candidates: List<PathCandidate>,
    val units: String = "rad"
)

@Serializable
data class CycleInfo(
    val edges: List<String>,
    val frames: List<String>,
    val residual: Double,
    val consistent: Boolean
)

@Serializable
data class InactiveCalibration(
    val id: Long,
    val name: String,
    val reason: String
)

@Serializable
data class CycleReport(
    val cycles: List<CycleInfo>,
    val minimalConflictEdgeSets: List<List<String>>,
    val inactiveCalibrations: List<InactiveCalibration> = emptyList()
)

@Serializable
data class GraphView(
    val nodes: List<String>,
    val edges: List<EdgeDetail>,
    val issues: List<String>
)

class Engine(
    val robot: RobotModel,
    calibrations: List<CalibrationSet>,
    private val jointValues: Map<String, Double>,
    private val serial: String? = null,
    private val at: Long? = null,
    private val tolerance: Double = 1e-6
) {
    val edges: List<TransformEdge>
    val inactiveCalibrations: List<InactiveCalibration>
    val issues: MutableList<String> = mutableListOf()

    private val jointByName: Map<String, JointModel> = robot.joints.associateBy { it.name }
    private val resolvedCache = mutableMapOf<String, ResolvedJoint>()

    init {
        val active = mutableListOf<TransformEdge>()
        val inactive = mutableListOf<InactiveCalibration>()
        for (j in robot.joints) {
            active.add(
                TransformEdge(
                    id = "urdf:joint:${j.name}",
                    parent = j.parentLink, child = j.childLink,
                    source = "urdf", jointName = j.name, jointType = j.type,
                    origin = j.origin, axis = j.axis
                )
            )
        }
        for (cal in calibrations) {
            val reason = when {
                cal.robotSerial != null && serial != null && cal.robotSerial != serial -> "SERIAL_MISMATCH"
                cal.robotSerial != null && serial == null -> "SERIAL_UNKNOWN"
                cal.validFrom != null && at != null && at < cal.validFrom -> "BEFORE_VALID_FROM"
                cal.validTo != null && at != null && at > cal.validTo -> "AFTER_VALID_TO"
                else -> null
            }
            if (reason != null) {
                inactive.add(InactiveCalibration(cal.id, cal.name, reason))
                continue
            }
            val boundary = at != null && (at == cal.validFrom || at == cal.validTo)
            cal.transforms.forEachIndexed { i, t ->
                active.add(
                    TransformEdge(
                        id = "cal:${cal.id}:$i",
                        parent = t.parent, child = t.child,
                        source = "calibration",
                        origin = Origin(
                            Xyz(t.xyz[0], t.xyz[1], t.xyz[2]),
                            atlas.urdf.Rpy(t.rpy[0], t.rpy[1], t.rpy[2])
                        ),
                        calibrationId = cal.id, calibrationName = cal.name,
                        robotSerial = cal.robotSerial,
                        validFrom = cal.validFrom, validTo = cal.validTo,
                        atValidityBoundary = boundary
                    )
                )
            }
        }
        // duplicate declarations: same ordered parent->child pair from multiple sources
        val byPair = active.groupBy { it.parent to it.child }
        val deduped = active.map { e ->
            if ((byPair[e.parent to e.child]?.size ?: 0) > 1) e.copy(duplicateDeclaration = true) else e
        }
        edges = deduped
        inactiveCalibrations = inactive
        for (name in robot.duplicateLinkNames) issues.add("DUPLICATE_FRAME:$name")
    }

    fun resolveJoint(name: String, visiting: Set<String> = emptySet()): ResolvedJoint {
        resolvedCache[name]?.let { return it }
        val j = jointByName[name]
            ?: return ResolvedJoint(name, 0.0, JointValueState.MISSING.name).also { resolvedCache[name] = it }
        val result: ResolvedJoint = when {
            j.type == JointType.FIXED -> ResolvedJoint(name, 0.0, JointValueState.FIXED.name)
            j.type == JointType.FLOATING || j.type == JointType.PLANAR ->
                ResolvedJoint(name, 0.0, JointValueState.UNSUPPORTED_TYPE.name)
            j.mimic != null -> {
                val target = j.mimic.joint
                if (name in visiting || target == name) {
                    ResolvedJoint(name, 0.0, JointValueState.MIMIC_CYCLE.name, target)
                } else {
                    val base = resolveJoint(target, visiting + name)
                    if (base.state == JointValueState.MIMIC_CYCLE.name) {
                        ResolvedJoint(name, 0.0, JointValueState.MIMIC_CYCLE.name, target)
                    } else {
                        ResolvedJoint(
                            name, base.value * j.mimic.multiplier + j.mimic.offset,
                            JointValueState.MIMIC.name, target
                        )
                    }
                }
            }
            jointValues.containsKey(name) ->
                ResolvedJoint(name, jointValues.getValue(name), JointValueState.PROVIDED.name)
            else -> ResolvedJoint(name, 0.0, JointValueState.MISSING.name)
        }
        resolvedCache[name] = result
        return result
    }

    private fun edgeMatrix(e: TransformEdge): Mat4 {
        val base = Mat.fromOrigin(e.origin)
        if (e.source == "calibration") return base
        val j = jointByName[e.jointName] ?: return base
        val q = resolveJoint(j.name).value
        return when (j.type) {
            JointType.REVOLUTE, JointType.CONTINUOUS ->
                Mat.mul(base, Mat.rotAxis(j.axis ?: Xyz(0.0, 0.0, 1.0), q))
            JointType.PRISMATIC -> {
                val a = j.axis ?: Xyz(1.0, 0.0, 0.0)
                Mat.mul(base, Mat.translation(a.x * q, a.y * q, a.z * q))
            }
            else -> base
        }
    }

    private fun adjacency(): Map<String, List<Pair<TransformEdge, Boolean>>> {
        val adj = mutableMapOf<String, MutableList<Pair<TransformEdge, Boolean>>>()
        for (e in edges) {
            adj.getOrPut(e.parent) { mutableListOf() }.add(e to true)
            adj.getOrPut(e.child) { mutableListOf() }.add(e to false)
        }
        return adj
    }

    fun frames(): Set<String> {
        val s = linkedSetOf<String>()
        for (e in edges) { s.add(e.parent); s.add(e.child) }
        return s
    }

    fun query(from: String, to: String): QueryResult {
        val qIssues = issues.toMutableList()
        if (from == to) {
            return QueryResult(from, to, "OK", qIssues, emptyList(), listOf(
                PathCandidate(0, listOf(from), emptyList(), Mat.identity().toList(), 0.0, true)
            ))
        }
        val adj = adjacency()
        val knownFrames = robot.links.map { it.name }.toSet() + frames()
        if (from !in knownFrames || to !in knownFrames) {
            return QueryResult(from, to, "UNKNOWN_FRAME", qIssues, emptyList(), emptyList())
        }
        if (from !in adj || to !in adj) {
            return QueryResult(from, to, "NO_PATH", qIssues, emptyList(), emptyList())
        }
        val paths = mutableListOf<List<Pair<TransformEdge, Boolean>>>()
        val maxPaths = 64
        fun dfs(node: String, visited: MutableSet<String>, acc: MutableList<Pair<TransformEdge, Boolean>>) {
            if (paths.size >= maxPaths || acc.size > 40) return
            if (node == to) { paths.add(acc.toList()); return }
            for ((e, fwd) in adj[node].orEmpty()) {
                val next = if (fwd) e.child else e.parent
                if (next in visited) continue
                visited.add(next); acc.add(e to fwd)
                dfs(next, visited, acc)
                acc.removeAt(acc.size - 1); visited.remove(next)
            }
        }
        val start = mutableSetOf(from)
        dfs(from, start, mutableListOf())
        if (paths.isEmpty()) {
            return QueryResult(from, to, "NO_PATH", qIssues, emptyList(), emptyList())
        }
        if (paths.size > 1) qIssues.add("MULTIPLE_PATHS:${paths.size}")

        val jointsUsed = linkedMapOf<String, ResolvedJoint>()
        val matrices = paths.map { path ->
            var m = Mat.identity()
            for ((e, fwd) in path) {
                if (e.jointName != null) jointsUsed.putIfAbsent(e.jointName, resolveJoint(e.jointName))
                val em = edgeMatrix(e)
                m = Mat.mul(m, if (fwd) em else Mat.inverseRigid(em))
            }
            m
        }
        val candidates = paths.mapIndexed { i, path ->
            var err = 0.0
            for (j in paths.indices) if (j != i) {
                val d = Mat.maxDiff(matrices[i], matrices[j])
                if (d > err) err = d
            }
            PathCandidate(
                index = i,
                frames = buildList {
                    add(from)
                    for ((e, fwd) in path) add(if (fwd) e.child else e.parent)
                },
                edges = path.map { (e, fwd) -> edgeDetail(e, fwd) },
                matrix = matrices[i].toList(),
                error = err,
                consistentWithOthers = err <= tolerance
            )
        }
        for ((_, rj) in jointsUsed) {
            when (rj.state) {
                JointValueState.MISSING.name -> qIssues.add("MISSING_JOINT_VALUE:${rj.joint}")
                JointValueState.MIMIC_CYCLE.name -> qIssues.add("MIMIC_CYCLE:${rj.joint}")
                else -> {}
            }
        }
        val status = when {
            jointsUsed.values.any { it.state == JointValueState.MIMIC_CYCLE.name } -> "MIMIC_CYCLE"
            jointsUsed.values.any { it.state == JointValueState.MISSING.name } -> "MISSING_JOINT_VALUE"
            else -> "OK"
        }
        return QueryResult(from, to, status, qIssues.distinct(), jointsUsed.values.toList(), candidates)
    }

    private fun edgeDetail(e: TransformEdge, fwd: Boolean): EdgeDetail {
        val rj = e.jointName?.let { resolveJoint(it) }
        return EdgeDetail(
            id = e.id, parent = e.parent, child = e.child, forward = fwd, source = e.source,
            jointName = e.jointName, jointType = e.jointType?.tag,
            jointValue = rj?.value, jointValueState = rj?.state,
            calibrationName = e.calibrationName,
            atValidityBoundary = e.atValidityBoundary,
            duplicateDeclaration = e.duplicateDeclaration
        )
    }

    fun graphView(): GraphView = GraphView(
        nodes = frames().toList(),
        edges = edges.map { edgeDetail(it, true) },
        issues = issues.toList()
    )

    /** Fundamental cycles of the frame graph with loop-closure residuals. */
    fun cycles(edgeSet: List<TransformEdge> = edges): List<CycleInfo> {
        val parent = mutableMapOf<String, String>()
        fun find(x: String): String {
            var r = x
            while (parent[r] != r) r = parent[r]!!
            var c = x
            while (parent[c] != c) { val n = parent[c]!!; parent[c] = r; c = n }
            return r
        }
        val nodes = linkedSetOf<String>()
        for (e in edgeSet) { nodes.add(e.parent); nodes.add(e.child) }
        nodes.forEach { parent[it] = it }
        val treeAdj = mutableMapOf<String, MutableList<Pair<TransformEdge, Boolean>>>()
        val nonTree = mutableListOf<TransformEdge>()
        for (e in edgeSet) {
            val rp = find(e.parent); val rc = find(e.child)
            if (rp != rc) {
                parent[rp] = rc
                treeAdj.getOrPut(e.parent) { mutableListOf() }.add(e to true)
                treeAdj.getOrPut(e.child) { mutableListOf() }.add(e to false)
            } else {
                nonTree.add(e)
            }
        }
        val result = mutableListOf<CycleInfo>()
        for (nt in nonTree) {
            // tree path from nt.parent to nt.child
            val path = mutableListOf<Pair<TransformEdge, Boolean>>()
            val prev = mutableMapOf<String, Pair<String, Pair<TransformEdge, Boolean>>>()
            val queue = ArrayDeque<String>()
            queue.add(nt.parent)
            val seen = mutableSetOf(nt.parent)
            while (queue.isNotEmpty()) {
                val n = queue.removeFirst()
                if (n == nt.child) break
                for ((e, fwd) in treeAdj[n].orEmpty()) {
                    val next = if (fwd) e.child else e.parent
                    if (next in seen) continue
                    seen.add(next)
                    prev[next] = n to (e to fwd)
                    queue.add(next)
                }
            }
            var cur = nt.child
            val framesRev = mutableListOf(cur)
            while (cur != nt.parent) {
                val (p, ef) = prev[cur] ?: break
                path.add(ef)
                cur = p
                framesRev.add(cur)
            }
            path.reverse()
            framesRev.reverse()
            // compose: tree path parent->child, then non-tree edge child->parent (reversed)
            var m = Mat.identity()
            for ((e, fwd) in path) {
                val em = edgeMatrix(e)
                m = Mat.mul(m, if (fwd) em else Mat.inverseRigid(em))
            }
            m = Mat.mul(m, Mat.inverseRigid(edgeMatrix(nt)))
            val edgeIds = path.map { it.first.id } + nt.id
            val residual = Mat.residualOf(m)
            result.add(CycleInfo(edgeIds, framesRev, residual, residual <= tolerance))
        }
        return result
    }

    fun cycleReport(): CycleReport {
        val cycles = cycles()
        val inconsistent = cycles.filter { !it.consistent }
        val conflictSets = mutableListOf<List<String>>()
        if (inconsistent.isNotEmpty()) {
            val pool = inconsistent.flatMap { it.edges }.distinct().take(12)
            val maxK = minOf(3, pool.size)
            var foundAt = -1
            for (k in 1..maxK) {
                for (combo in combinations(pool, k)) {
                    val removed = combo.toSet()
                    val remaining = edges.filter { it.id !in removed }
                    if (cycles(remaining).all { it.consistent }) {
                        conflictSets.add(combo)
                    }
                }
                if (conflictSets.isNotEmpty()) { foundAt = k; break }
            }
            if (foundAt == -1) conflictSets.add(pool) // fallback: whole inconsistent pool
        }
        return CycleReport(cycles, conflictSets, inactiveCalibrations)
    }

    private fun combinations(pool: List<String>, k: Int): List<List<String>> {
        val out = mutableListOf<List<String>>()
        fun rec(start: Int, acc: MutableList<String>) {
            if (acc.size == k) { out.add(acc.toList()); return }
            for (i in start until pool.size) {
                acc.add(pool[i])
                rec(i + 1, acc)
                acc.removeAt(acc.size - 1)
            }
        }
        rec(0, mutableListOf())
        return out
    }
}
