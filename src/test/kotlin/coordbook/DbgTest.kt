package coordbook
import kotlin.test.Test
class DbgTest {
  @Test fun dbg() {
    val xml = """<?xml version="1.0"?>
<robot name="badloop">
  <link name="a"/><link name="b"/><link name="c"/>
  <joint name="ab" type="fixed"><parent link="a"/><child link="b"/>
    <origin xyz="1 0 0" rpy="0 0 0"/></joint>
  <joint name="bc" type="fixed"><parent link="b"/><child link="c"/>
    <origin xyz="0 1 0" rpy="0 0 0"/></joint>
  <joint name="ca" type="fixed"><parent link="c"/><child link="a"/>
    <origin xyz="0 -1.02 0" rpy="0 0 0"/></joint>
</robot>
"""
    val doc = UrdfParser.parse(xml)
    val (g,_) = GraphBuilder.build(doc)
    println("FRAMES=" + g.frames.keys.toList())
    for ((n, es) in g.outgoing) println("OUT $n -> " + es.map { "${it.toName}(${it.source.label}) rev=${it.reversedFlag}" })
    val r = Cycles.analyze(g)
    println("TREE=" + r.spanningTreeEdgeIds)
    println("RES=" + r.residuals.map { listOf(it.frames, it.edgeLabels, it.translationErrorM) })
    println("MIN=" + r.minimalContradictingEdges)
  }
}
