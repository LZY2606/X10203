package gsb

import kotlinx.serialization.Serializable

enum class EdgeKind { URDF_JOINT, CALIBRATION }

data class Edge(
    val id: String,
    val from: String,
    val to: String,
    val kind: EdgeKind,
    val joint: Joint? = null,
    val constant: Transform? = null,
    val source: String
)

enum class ValueSource { SNAPSHOT, MIMIC, DEFAULT_FIXED, MISSING }

data class JointValue(
    val joint: String,
    val value: Double?,
    val unit: String = "rad",
    val source: ValueSource,
    val detail: String = ""
)

class MimicCycleException(val cycle: List<String>) :
    Exception("mimic cycle: ${cycle.joinToString(" -> ")}")

/** Resolves joint values for one evaluation: snapshot values, mimic relations, fixed defaults. */
class JointResolver(val model: RobotModel, private val rawValuesRad: Map<String, Double>) {
    private val memo = HashMap<String, Double?>()
    val values = LinkedHashMap<String, JointValue>()

    fun resolve(name: String, stack: List<String> = emptyList()): Double? {
        if (name in stack) {
            throw MimicCycleException(stack.drop(stack.indexOf(name)) + name)
        }
        if (memo.containsKey(name)) return memo[name]
        val joint = model.joint(name)
        if (joint == null) {
            values[name] = JointValue(name, null, source = ValueSource.MISSING, detail = "unknown joint")
            memo[name] = null
            return null
        }
        if (!joint.movable) {
            values[name] = JointValue(name, 0.0, source = ValueSource.DEFAULT_FIXED, detail = "fixed joint")
            memo[name] = 0.0
            return 0.0
        }
        val mimic = joint.mimic
        val result: Double? = if (mimic != null) {
            val base = resolve(mimic.joint, stack + name)
            if (base == null) null else {
                val v = mimic.multiplier * base + mimic.offset
                values[name] = JointValue(
                    name, v, source = ValueSource.MIMIC,
                    detail = "mimic of ${mimic.joint} (x${mimic.multiplier} + ${mimic.offset})"
                )
                v
            }
        } else if (rawValuesRad.containsKey(name)) {
            values[name] = JointValue(name, rawValuesRad[name], source = ValueSource.SNAPSHOT)
            rawValuesRad[name]
        } else {
            null
        }
        if (result == null && !values.containsKey(name)) {
            values[name] = JointValue(name, null, source = ValueSource.MISSING, detail = "no value in snapshot")
        }
        memo[name] = result
        return result
    }
}

/** Transform of an edge in its declared direction; null when a required joint value is missing. */
fun edgeTransform(e: Edge, resolver: JointResolver): Transform? = when (e.kind) {
    EdgeKind.CALIBRATION -> e.constant
    EdgeKind.URDF_JOINT -> {
        val j = e.joint!!
        if (!j.movable) j.transform(0.0)
        else resolver.resolve(j.name)?.let { j.transform(it) }
    }
}

data class PathStep(val edge: Edge, val forward: Boolean)

/** Enumerate all simple paths between two frames (bounded); never picks one by traversal order. */
fun enumeratePaths(
    edges: List<Edge>,
    from: String,
    to: String,
    maxPaths: Int = 64,
    maxDepth: Int = 24
): List<List<PathStep>> {
    val adj = HashMap<String, MutableList<Pair<Edge, Boolean>>>()
    for (e in edges) {
        adj.getOrPut(e.from) { mutableListOf() }.add(e to true)
        adj.getOrPut(e.to) { mutableListOf() }.add(e to false)
    }
    val result = mutableListOf<List<PathStep>>()
    val visited = hashSetOf(from)
    fun dfs(node: String, path: List<PathStep>) {
        if (result.size >= maxPaths) return
        if (node == to) {
            if (path.isNotEmpty()) result.add(path)
            return
        }
        if (path.size >= maxDepth) return
        for ((e, fwd) in adj[node].orEmpty()) {
            val next = if (fwd) e.to else e.from
            if (next in visited) continue
            visited.add(next)
            dfs(next, path + PathStep(e, fwd))
            visited.remove(next)
        }
    }
    dfs(from, emptyList())
    return result
}

const val LOOP_EPS = 1e-6

data class LoopEval(
    val steps: List<PathStep>,
    val residual: Transform,
    val consistent: Boolean,
    val note: String = ""
)

/** Fundamental cycles from a spanning forest; loops are never collapsed into a tree. */
fun cycleBasis(edges: List<Edge>): List<List<PathStep>> {
    val adj = HashMap<String, MutableList<Pair<Edge, Boolean>>>()
    for (e in edges) {
        adj.getOrPut(e.from) { mutableListOf() }.add(e to true)
        adj.getOrPut(e.to) { mutableListOf() }.add(e to false)
    }
    val parent = HashMap<String, String?>()
    val parentStep = HashMap<String, PathStep>()
    val treeEdgeIds = mutableSetOf<String>()
    val nodes = edges.flatMap { listOf(it.from, it.to) }.distinct()
    for (start in nodes) {
        if (parent.containsKey(start)) continue
        parent[start] = null
        val queue = ArrayDeque<String>()
        queue.add(start)
        while (queue.isNotEmpty()) {
            val n = queue.removeFirst()
            for ((e, fwd) in adj[n].orEmpty()) {
                val next = if (fwd) e.to else e.from
                if (parent.containsKey(next)) continue
                parent[next] = n
                parentStep[next] = PathStep(e, fwd)
                treeEdgeIds.add(e.id)
                queue.add(next)
            }
        }
    }

    fun treePath(a: String, b: String): List<PathStep>? {
        // steps from a up to the root, recorded per ancestor
        val fromA = HashMap<String, List<PathStep>>()
        var cur: String? = a
        var steps = emptyList<PathStep>()
        fromA[a] = steps
        while (cur != null) {
            val pe = parentStep[cur] ?: break
            val p = parent[cur] ?: break
            steps = steps + PathStep(pe.edge, !pe.forward)
            cur = p
            fromA[cur] = steps
        }
        var up = emptyList<PathStep>()
        var node: String? = b
        while (node != null && !fromA.containsKey(node)) {
            val pe = parentStep[node] ?: return null // different component
            up = up + PathStep(pe.edge, !pe.forward)
            node = parent[node]
        }
        if (node == null) return null
        val down = up.reversed().map { PathStep(it.edge, !it.forward) }
        return fromA[node]!! + down
    }

    val cycles = mutableListOf<List<PathStep>>()
    for (e in edges) {
        if (e.id in treeEdgeIds) continue
        val path = treePath(e.from, e.to) ?: continue
        cycles.add(path + PathStep(e, true))
    }
    return cycles
}

fun evalLoop(steps: List<PathStep>, resolver: JointResolver): LoopEval {
    var t = Transform.identity()
    var note = ""
    for (s in steps) {
        val et = try {
            edgeTransform(s.edge, resolver)
        } catch (e: MimicCycleException) {
            return LoopEval(steps, Transform.identity(), false, "mimic cycle: ${e.cycle.joinToString(" -> ")}")
        }
        if (et == null) {
            note = "missing joint value, assumed 0"
            val j = s.edge.joint!!
            val assumed = j.transform(0.0)
            t = t * (if (s.forward) assumed else assumed.inverse())
        } else {
            t = t * (if (s.forward) et else et.inverse())
        }
    }
    return LoopEval(steps, t, t.errorNorm() < LOOP_EPS, note)
}

/**
 * Smallest set of edges whose removal makes every fundamental cycle consistent.
 * Tries subsets in increasing size over edges of inconsistent loops.
 */
fun minimalConflictSet(
    edges: List<Edge>,
    model: RobotModel,
    valuesRad: Map<String, Double>,
    maxSize: Int = 3
): List<String> {
    fun inconsistentLoops(es: List<Edge>): List<LoopEval> {
        val resolver = JointResolver(model, valuesRad)
        return cycleBasis(es).map { evalLoop(it, resolver) }.filter { !it.consistent }
    }
    val bad = inconsistentLoops(edges)
    if (bad.isEmpty()) return emptyList()
    val candidates = bad.flatMap { l -> l.steps.map { it.edge.id } }.distinct()
    val limit = minOf(maxSize, candidates.size)
    for (k in 1..limit) {
        for (combo in combinations(candidates, k)) {
            val remaining = edges.filter { it.id !in combo }
            if (inconsistentLoops(remaining).isEmpty()) return combo
        }
    }
    return candidates
}

fun <T> combinations(items: List<T>, k: Int): List<List<T>> {
    val out = mutableListOf<List<T>>()
    fun rec(start: Int, acc: List<T>) {
        if (acc.size == k) {
            out.add(acc)
            return
        }
        for (i in start until items.size) rec(i + 1, acc + items[i])
    }
    rec(0, emptyList())
    return out
}

// ---------- DTOs shared by engine and server ----------

@Serializable
data class PathStepDto(
    val edgeId: String,
    val from: String,
    val to: String,
    val direction: String,
    val kind: String,
    val source: String,
    val jointName: String? = null,
    val jointType: String? = null
)

@Serializable
data class JointValueDto(
    val joint: String,
    val valueRad: Double?,
    val valueDeg: Double?,
    val unit: String,
    val source: String,
    val detail: String
)

@Serializable
data class CandidateDto(
    val index: Int,
    val status: String,
    val notes: List<String>,
    val path: List<PathStepDto>,
    val jointValues: List<JointValueDto>,
    val matrix: List<Double>?,
    val deviationFromFirst: Double?
)

@Serializable
data class LoopDto(
    val id: String,
    val edges: List<String>,
    val frames: List<String>,
    val translationError: Double,
    val rotationErrorRad: Double,
    val rotationErrorDeg: Double,
    val consistent: Boolean,
    val residualMatrix: List<Double>,
    val note: String
)

@Serializable
data class EdgeDto(
    val id: String,
    val from: String,
    val to: String,
    val kind: String,
    val source: String,
    val jointName: String? = null,
    val jointType: String? = null
)

@Serializable
data class CalibStatusDto(
    val id: Long,
    val name: String,
    val version: Int,
    val serial: String,
    val fromFrame: String,
    val toFrame: String,
    val active: Boolean,
    val reason: String
)

@Serializable
data class GraphResponse(
    val nodes: List<String>,
    val edges: List<EdgeDto>,
    val loops: List<LoopDto>,
    val minimalConflictSet: List<String>,
    val duplicates: List<List<String>>
)

@Serializable
data class PoseResponse(
    val from: String,
    val to: String,
    val time: String,
    val urdfVersion: Int,
    val calibVersion: Int?,
    val statuses: List<String>,
    val calibrations: List<CalibStatusDto>,
    val candidates: List<CandidateDto>
)
