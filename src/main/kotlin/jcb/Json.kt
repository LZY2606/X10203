package jcb

/** Tiny JSON writer/reader sufficient for the API (no external dependency). */
object Json {
    fun write(v: Any?): String = when (v) {
        null -> "null"
        is String -> "\"" + v.replace("\\", "\\\\").replace("\"", "\\\"")
            .replace("\n", "\\n").replace("\r", "\\r").replace("\t", "\\t") + "\""
        is Number, is Boolean -> v.toString()
        is Map<*, *> -> v.entries.joinToString(",", "{", "}") { write(it.key.toString()) + ":" + write(it.value) }
        is Iterable<*> -> v.joinToString(",", "[", "]") { write(it) }
        is DoubleArray -> v.joinToString(",", "[", "]")
        else -> write(v.toString())
    }

    fun parse(s: String): Any? = Reader(s).readValue()

    private class Reader(val s: String) {
        var i = 0
        fun readValue(): Any? {
            ws()
            return when {
                i >= s.length -> null
                s[i] == '{' -> readObj()
                s[i] == '[' -> readArr()
                s[i] == '"' -> readStr()
                s.startsWith("true", i) -> { i += 4; true }
                s.startsWith("false", i) -> { i += 5; false }
                s.startsWith("null", i) -> { i += 4; null }
                else -> readNum()
            }
        }
        fun readObj(): Map<String, Any?> {
            val m = LinkedHashMap<String, Any?>()
            i++ // {
            ws()
            if (i < s.length && s[i] == '}') { i++; return m }
            while (true) {
                ws(); val k = readStr(); ws(); i++ // :
                m[k] = readValue(); ws()
                if (i < s.length && s[i] == ',') { i++; continue }
                if (i < s.length && s[i] == '}') { i++; break }
                break
            }
            return m
        }
        fun readArr(): List<Any?> {
            val l = mutableListOf<Any?>()
            i++ // [
            ws()
            if (i < s.length && s[i] == ']') { i++; return l }
            while (true) {
                l.add(readValue()); ws()
                if (i < s.length && s[i] == ',') { i++; continue }
                if (i < s.length && s[i] == ']') { i++; break }
                break
            }
            return l
        }
        fun readStr(): String {
            i++ // "
            val sb = StringBuilder()
            while (i < s.length && s[i] != '"') {
                if (s[i] == '\\' && i + 1 < s.length) {
                    i++
                    sb.append(
                        when (s[i]) {
                            'n' -> '\n'; 't' -> '\t'; 'r' -> '\r'
                            'u' -> {
                                val hex = s.substring(i + 1, i + 5); i += 4
                                hex.toInt(16).toChar()
                            }
                            else -> s[i]
                        }
                    )
                } else sb.append(s[i])
                i++
            }
            i++ // closing "
            return sb.toString()
        }
        fun readNum(): Number {
            val start = i
            while (i < s.length && (s[i].isDigit() || s[i] in ".-+eE")) i++
            val t = s.substring(start, i)
            return t.toDoubleOrNull() ?: 0.0
        }
        fun ws() { while (i < s.length && s[i].isWhitespace()) i++ }
    }

    @Suppress("UNCHECKED_CAST")
    fun asMap(v: Any?): Map<String, Any?> = v as? Map<String, Any?> ?: emptyMap()
    fun asList(v: Any?): List<Any?> = v as? List<Any?> ?: emptyList()
    fun asDouble(v: Any?): Double? = (v as? Number)?.toDouble()
    fun asLong(v: Any?): Long? = (v as? Number)?.toLong()
    fun asStr(v: Any?): String? = v as? String
    fun asDoubleArray(v: Any?): DoubleArray =
        asList(v).map { asDouble(it) ?: 0.0 }.toDoubleArray()
    fun asStringDoubleMap(v: Any?): Map<String, Double> =
        asMap(v).mapValues { asDouble(it.value) ?: 0.0 }
}
