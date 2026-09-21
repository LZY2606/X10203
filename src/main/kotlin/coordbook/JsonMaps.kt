package coordbook

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.double
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive

object JsonMaps {
    private val json = Json { ignoreUnknownKeys = true }

    fun encodeDoubles(map: Map<String, Double>): String {
        val obj = JsonObject(map.mapValues { JsonPrimitive(it.value) })
        return json.encodeToString(JsonObject.serializer(), obj)
    }

    fun decodeDoubles(text: String): Map<String, Double> {
        val obj = json.parseToJsonElement(text).jsonObject
        return obj.mapValues { it.value.jsonPrimitive.double }
    }
}
