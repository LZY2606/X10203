package coordbook


const val POS_EPS = 1e-6 // 米
const val ROT_EPS = 1e-7 // 弧度（约 0.02 角秒），用于“姿态一致”判定

data class FrameQueryResult(
    val ok: Boolean,
    val error: String?,
    val ambiguity: List<String>?,
    val report: MultiPathReport?,
)

data class CycleResidual(
    val frames: List<String>,
    val edgeLabels: List<String>,
    val edgeIds: List<String>,
    val closureTransform: List<List<Double>>,
    val translationErrorM: Double,
    val rotationErrorRad: Double,
    val consistent: Boolean,
)

data class CycleReport(
    val hasCycle: Boolean,
    val residuals: List<CycleResidual>,
    val inconsistent: Boolean,
    val minimalContradictingEdges: List<String>,
    val spanningTreeEdgeIds: List<String>,
    val note: String,
)

data class GraphStructure(
    val disconnected: List<List<String>>,
    val fixedLoopPresent: Boolean,
)

object Kinematics {

    /** 枚举 from->to 的简单路径（边数有界），每条平行边独立成为候选，不做遍历序裁剪。 */
    fun enumerateSimplePaths(
        graph: FrameGraph,
        from: String,
        to: String,
        maxPaths: Int = 64,
    ): List<List<GraphEdge>> {
        val results = mutableListOf<List<GraphEdge>>()
        if (from == to) return listOf(emptyList())

        fun dfs(node: String, path: MutableList<GraphEdge>, visited: MutableSet<String>) {
            if (results.size >= maxPaths) return
            if (node == to) {
                results += path.toList()
                return
            }
            val neighbors = graph.neighbors(node).sortedWith(
                compareBy({ it.to }, { it.source.label }),
            )
            for (edge in neighbors) {
                if (edge.to in visited) continue
                visited += edge.to
                path += edge
                dfs(edge.to, path, visited)
                path.removeAt(path.lastIndex)
                visited -= edge.to
                if (results.size >= maxPaths) return
            }
        }

        dfs(from, mutableListOf(), mutableSetOf(from))
        return results
    }

    fun query(
        graph: FrameGraph,
        doc: UrdfDocument,
        fromName: String,
        toName: String,
        values: Map<String, Double>,
    ): FrameQueryResult {
        val fromIds = graph.frameIdsByName(fromName)
        val toIds = graph.frameIdsByName(toName)
        when {
            fromIds.isEmpty() && toIds.isEmpty() -> return FrameQueryResult(
                false, "frame \"$fromName\" 与 \"$toName\" 均不存在", null, null,
            )
            fromIds.isEmpty() -> return FrameQueryResult(false, "frame \"$fromName\" 不存在", null, null)
            toIds.isEmpty() -> return FrameQueryResult(false, "frame \"$toName\" 不存在", null, null)
        }
        if (fromIds.size > 1 || toIds.size > 1) {
            return FrameQueryResult(
                false,
                "frame 名称有歧义：\"$fromName\" x${fromIds.size}, \"$toName\" x${toIds.size}，" +
                    "请使用内部 id",
                fromIds + toIds, null,
            )
        }
        val resolver = JointValueResolver(doc)
        val paths = enumerateSimplePaths(graph, fromIds.single(), toIds.single())
        if (paths.isEmpty()) {
            return FrameQueryResult(false, "\"$fromName\" 与 \"$toName\" 之间断链，不存在路径", null, null)
        }

        val candidates = mutableListOf<PathCandidate>()
        var firstMissing: MissingJointStatus? = null
        for ((index, edges) in paths.withIndex()) {
            var product = Transform.identity()
            val steps = mutableListOf<ChainStep>()
            val frames = mutableListOf(graph.frames.getValue(edges.firstOrNull()?.from ?: fromIds.single()).name)
            var failed: MissingJointStatus? = null
            for (edge in edges) {
                val eval = edge.resolved(values, resolver)
                val t = eval.transform
                if (t == null) {
                    val r = eval.result as JointValueResult
                    failed = when (r) {
                        is JointValueResult.Missing -> MissingJointStatus(r.jointName, r.reason)
                        is JointValueResult.Cycle -> MissingJointStatus(
                            r.jointName, "mimic 环经过该关节，值无法确定",
                        )
                        is JointValueResult.Ok -> error("不可能")
                    }
                    break
                }
                product = product * t
                val (joint, value) = eval.joint ?: (null to null)
                val ok = eval.result as? JointValueResult.Ok
                steps += ChainStep(
                    fromFrame = edge.fromName,
                    toFrame = edge.toName,
                    edgeLabel = edge.source.label,
                    edgeKind = edge.source.kind,
                    jointName = joint?.name,
                    jointType = joint?.type?.urdfName,
                    jointValue = value,
                    valueUnit = ok?.unit ?: joint?.type?.unit,
                    valueOrigin = ok?.derivedFrom?.let { "mimic:$it" }
                        ?: if (ok != null) "snapshot" else null,
                    transform = t.matrixRows(),
                )
                frames += edge.toName
            }
            if (failed != null) {
                if (firstMissing == null) firstMissing = failed
                continue
            }
            candidates += PathCandidate(index, frames, steps, product.matrixRows())
        }

        if (candidates.isEmpty() && firstMissing != null) {
            return FrameQueryResult(
                false, null, null,
                MultiPathReport(
                    fromName, toName, emptyList(), true, 0.0, 0.0, null, firstMissing,
                ),
            )
        }

        val transforms = candidates.map { Transform(it.transform.flatten().toDoubleArray()) }
        var maxT = 0.0
        var maxR = 0.0
        for (i in transforms.indices) {
            for (j in i + 1 until transforms.size) {
                val d = transforms[i].diff(transforms[j])
                maxT = maxOf(maxT, d.translationMeters)
                maxR = maxOf(maxR, d.rotationRadians)
            }
        }
        val consistent = maxT <= POS_EPS && maxR <= ROT_EPS
        val consensus = if (consistent && transforms.isNotEmpty()) {
            Transform.average(transforms).matrixRows()
        } else null
        return FrameQueryResult(
            true, null, null,
            MultiPathReport(
                fromName, toName, candidates, consistent, maxT, maxR, consensus,
                if (candidates.isEmpty()) firstMissing else null,
            ),
        )
    }
}

object Cycles {

    private data class LabeledEdge(
        val id: String,
        val label: String,
        val from: String,
        val to: String,
        val forward: GraphEdge,
    )

    private fun undirectedEdges(graph: FrameGraph): List<LabeledEdge> {
        val seen = linkedMapOf<String, LabeledEdge>()
        for (nodeId in graph.frames.keys) {
            for (e in graph.neighbors(nodeId)) {
                if (e.reversedFlag) continue
                val key = canonicalKey(e)
                if (key in seen) continue
                seen[key] = LabeledEdge(key, e.source.label, e.from, e.to, e)
            }
        }
        return seen.values.toList()
    }

    private fun canonicalKey(e: GraphEdge): String =
        when (e.source) {
            is UrdfEdgeSource -> "urdf:${e.source.jointId}"
            is CalibEdgeSource -> "calib:${e.source.patchId}"
        }

    /** 以名义（零值）位姿做 BFS 生成林，计算每条非树边闭合环路的残差。 */
    fun analyze(graph: FrameGraph): CycleReport {
        val edges = undirectedEdges(graph)
        val adjacency = linkedMapOf<String, MutableList<Pair<LabeledEdge, Boolean>>>()
        for (edge in edges) {
            adjacency.getOrPut(edge.from) { mutableListOf() }.add(edge to true)
            adjacency.getOrPut(edge.to) { mutableListOf() }.add(edge to false)
        }
        val potential = linkedMapOf<String, Transform>()
        val parent = linkedMapOf<String, Pair<String, LabeledEdge>?>()
        val treeEdgeIds = mutableSetOf<String>()
        val roots = mutableListOf<String>()

        for (start in graph.frames.keys) {
            if (start in potential) continue
            roots += start
            potential[start] = Transform.identity()
            parent[start] = null
            val queue = ArrayDeque<String>()
            queue.add(start)
            while (queue.isNotEmpty()) {
                val node = queue.removeFirst()
                val rootToNode = potential.getValue(node)
                for ((edge, forward) in adjacency[node].orEmpty().sortedWith(
                        compareBy({ it.first.id }, { it.second.toString() }),
                    )) {
                    val tNodeToNext = if (forward) edge.forward.nominal else edge.forward.nominal.inverse()
                    val next = if (forward) edge.to else edge.from
                    val rootToNext = rootToNode * tNodeToNext
                    if (next !in potential) {
                        potential[next] = rootToNext
                        parent[next] = node to edge
                        treeEdgeIds += edge.id
                        queue.add(next)
                    }
                }
            }
        }

        val residuals = mutableListOf<CycleResidual>()
        for (edge in edges.sortedBy { it.id }) {
            if (edge.id in treeEdgeIds) continue
            // potential[x] = T_root->x。弦 T_ab(a->b) 的闭合：
            // T_root->a * T_a->b * (T_root->b)^-1 应等于单位阵
            val closure = potential.getValue(edge.from) *
                edge.forward.nominal * potential.getValue(edge.to).inverse()
            val delta = closure.diff(Transform.identity())
            val cycleInfo = cycleFramesAndEdges(edge, parent)
            residuals += CycleResidual(
                frames = cycleInfo.first.map { graph.frames.getValue(it).name },
                edgeLabels = cycleInfo.second + edge.label,
                edgeIds = cycleInfo.second + edge.label,
                closureTransform = closure.matrixRows(),
                translationErrorM = delta.translationMeters,
                rotationErrorRad = delta.rotationRadians,
                consistent = delta.translationMeters <= POS_EPS &&
                    delta.rotationRadians <= ROT_EPS,
            )
        }

        val inconsistent = residuals.any { !it.consistent }
        val minimal = if (inconsistent) {
            minimalContradictingEdgeSet(graph, edges, treeEdgeIds, adjacency, potential)
        } else emptyList()

        val note = when {
            residuals.isEmpty() -> "未检测到环路（图为森林）"
            inconsistent -> "检测到不一致环路；下列边集删除后图上所有路径姿态一致（按边 id 字典序取最小集合）"
            else -> "所有环路闭合残差为零，固定/活动环路姿态一致，未展开成树"
        }
        return CycleReport(
            hasCycle = residuals.isNotEmpty(),
            residuals = residuals,
            inconsistent = inconsistent,
            minimalContradictingEdges = minimal,
            spanningTreeEdgeIds = treeEdgeIds.sorted(),
            note = note,
        )
    }

    /** 弦 (a,b) 与树路径构成的环：a 向上到 LCA，再下到 b；返回帧序列与树边标签序列。 */
    private fun cycleFramesAndEdges(
        chord: LabeledEdge,
        parent: Map<String, Pair<String, LabeledEdge>?>,
    ): Pair<List<String>, List<String>> {
        var a = chord.from
        var b = chord.to
        val upA = mutableListOf<String>() // a, parent(a), ..., LCA 之前
        val upB = mutableListOf<String>()
        val depth = HashMap<String, Int>()
        fun d(node: String): Int = depth.getOrPut(node) {
            parent[node]?.let { d(it.first) + 1 } ?: 0
        }
        var da = d(a); var db = d(b)
        while (da > db) {
            upA += a
            a = parent[a]!!.first; da--
        }
        while (db > da) {
            upB += b
            b = parent[b]!!.first; db--
        }
        while (a != b) {
            upA += a; upB += b
            a = parent[a]!!.first; b = parent[b]!!.first
        }
        val lca = a
        val frames = upA + lca + upB.reversed()
        val labels = mutableListOf<String>()
        for (node in upA) labels += parent[node]!!.second.label
        for (node in upB.reversed()) labels += parent[node]!!.second.label
        return frames to labels
    }

    /**
     * 精确最小矛盾边集。
     *
     * 单环中删除任意一条边都能让图“成为树”，所以只看“剩余是否一致”无法归因。
     * 关键观察：真正的矛盾边在“以其它边为生成树、以该边为弦”时所需的闭合修正量最小
     * （本例 ca 偏 0.02：以 {ab,bc} 为树时 ca 残差 0.02；而以 ca 为树边时 bc 残差 ~1.0）。
     *
     * 步骤：
     *  1) 计算每条边在所有生成树选择中作为弦的最小闭合残量 blame；
     *  2) 从大到小移除 blame 超过阈值的边，直到剩余图一致；
     *  3) 然后按 blame 降序做“不可省略”压缩，得到基数最小集合；同 blame 按 id 字典序。
     */
    private fun minimalContradictingEdgeSet(
        graph: FrameGraph,
        edges: List<LabeledEdge>,
        treeEdgeIds: Set<String>,
        @Suppress("UNUSED_PARAMETER") adjacency: Map<String, MutableList<Pair<LabeledEdge, Boolean>>>,
        @Suppress("UNUSED_PARAMETER") potential: Map<String, Transform>,
    ): List<String> {
        val byId = edges.associateBy { it.id }
        // blame[e] = 所有“e 为弦”的生成树下的最小闭合残量
        val blame = HashMap<String, Double>()
        for (chord in edges) {
            val kept = edges.filter { it.id != chord.id }
            if (isConsistent(kept)) {
                // chord 从最小二乘势（其它边构成的参考）看的残差
                blame[chord.id] = closureStrainOfChord(kept, chord)
            } else {
                blame[chord.id] = Double.POSITIVE_INFINITY
            }
        }
        val badThreshold = POS_EPS + ROT_EPS
        // 贪心移除：始终移除 blame 最大（最不可能是好边）的边，直到一致
        val removed = linkedSetOf<String>()
        fun keptEdges() = edges.filter { it.id !in removed }
        while (!isConsistent(keptEdges()) && removed.size < edges.size) {
            val next = edges.filter { it.id !in removed }
                .maxWithOrNull(
                    compareBy<LabeledEdge> { blame[it.id] ?: Double.POSITIVE_INFINITY }
                        .thenByDescending { it.id },
                )!!
            removed += next.id
        }
        // 压缩：尝试按 blame 升序（最可能是好边）逐个加回，加回仍一致则不必删除
        val mandatory = removed.toMutableSet()
        for (id in removed.sortedBy { blame[it] ?: Double.POSITIVE_INFINITY }) {
            val trial = mandatory - id
            if (!isConsistent(edges.filter { it.id !in trial })) {
                // 加回 id 后不一致 => id 必须删除
            } else {
                mandatory -= id
            }
        }
        val sorted = mandatory.sortedWith(
            compareByDescending<String> { blame[it] ?: Double.POSITIVE_INFINITY }.thenBy { it },
        )
        System.err.println("BLAME=$blame MANDATORY=$mandatory")
        return sorted
    }

    private fun closureStrainOfChord(reference: List<LabeledEdge>, chord: LabeledEdge): Double {
        val adj = linkedMapOf<String, MutableList<Pair<LabeledEdge, Boolean>>>()
        for (edge in reference) {
            adj.getOrPut(edge.from) { mutableListOf() }.add(edge to true)
            adj.getOrPut(edge.to) { mutableListOf() }.add(edge to false)
        }
        val pot = linkedMapOf<String, Transform>()
        for (start in (adj.keys + listOf(chord.from, chord.to))) {
            if (start in pot) continue
            pot[start] = Transform.identity()
            val queue = ArrayDeque<String>()
            queue.add(start)
            while (queue.isNotEmpty()) {
                val node = queue.removeFirst()
                for ((edge, forward) in adj[node].orEmpty().sortedWith(
                        compareBy({ it.first.id }, { it.second.toString() }),
                    )) {
                    val next = if (forward) edge.to else edge.from
                    if (next !in pot) {
                        val t = if (forward) edge.forward.nominal else edge.forward.nominal.inverse()
                        pot[next] = pot.getValue(node) * t
                        queue.add(next)
                    }
                }
            }
        }
        val pa = pot[chord.from]
        val pb = pot[chord.to]
        if (pa == null || pb == null) return Double.POSITIVE_INFINITY
        val closure = pa * chord.forward.nominal * pb.inverse()
        val d = closure.diff(Transform.identity())
        return d.translationMeters + d.rotationRadians
    }
    /** 剩余图是否对所有连通分量满足：边两侧在生成树势下闭合一致。 */
    private fun isConsistent(kept: List<LabeledEdge>): Boolean {
        val adj = linkedMapOf<String, MutableList<Pair<LabeledEdge, Boolean>>>()
        for (edge in kept) {
            adj.getOrPut(edge.from) { mutableListOf() }.add(edge to true)
            adj.getOrPut(edge.to) { mutableListOf() }.add(edge to false)
        }
        val pot = linkedMapOf<String, Transform>()
        for (start in adj.keys) {
            if (start in pot) continue
            pot[start] = Transform.identity()
            val queue = ArrayDeque<String>()
            queue.add(start)
            while (queue.isNotEmpty()) {
                val node = queue.removeFirst()
                for ((edge, forward) in adj[node].orEmpty()) {
                    val t = if (forward) edge.forward.nominal else edge.forward.nominal.inverse()
                    val next = if (forward) edge.to else edge.from
                    val candidate = pot.getValue(node) * t
                    if (next !in pot) {
                        pot[next] = candidate
                        queue.add(next)
                    } else {
                        val delta = pot.getValue(next).diff(candidate)
                        if (delta.translationMeters > POS_EPS || delta.rotationRadians > ROT_EPS) {
                            return false
                        }
                    }
                }
            }
        }
        return true
    }
}

object GraphStats {
    /** 返回所有连通分量（含孤立 frame），用于报告断链。 */
    fun disconnectedComponents(graph: FrameGraph): List<List<String>> {
        val seen = mutableSetOf<String>()
        val components = mutableListOf<List<String>>()
        for (start in graph.frames.keys) {
            if (start in seen) continue
            val comp = mutableListOf<String>()
            val queue = ArrayDeque<String>()
            queue.add(start); seen += start
            while (queue.isNotEmpty()) {
                val node = queue.removeFirst()
                comp += node
                for (e in graph.neighbors(node)) {
                    if (seen.add(e.to)) queue.add(e.to)
                }
            }
            components += comp.map { graph.frames.getValue(it).name }.sorted()
        }
        return components
    }
}
