package atlas

fun Any?.toJson(): String = when (this) {
    null -> "null"
    is String -> "\"" + this.replace("\\", "\\\\").replace("\"", "\\\"")
        .replace("\n", "\\n").replace("\r", "\\r").replace("\t", "\\t") + "\""
    is Number, is Boolean -> toString()
    is Mat4 -> m.toList().toJson()
    is DoubleArray -> toList().toJson()
    is Map<*, *> -> entries.joinToString(",", "{", "}") { "\"${it.key}\":${it.value.toJson()}" }
    is Iterable<*> -> joinToString(",", "[", "]") { it.toJson() }
    is Array<*> -> toList().toJson()
    else -> toString().toJson()
}
