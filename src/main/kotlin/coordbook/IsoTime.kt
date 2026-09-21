package coordbook

import java.time.Instant
import java.time.LocalDateTime
import java.time.ZoneOffset
import java.time.format.DateTimeParseException

/** 仅接受 UTC ISO-8601（带 Z 或 +00:00），避免现场时区歧义。 */
object IsoTime {
    fun parse(text: String): Instant = try {
        if (text.endsWith("Z")) Instant.parse(text)
        else LocalDateTime.parse(text).toInstant(ZoneOffset.UTC)
    } catch (e: DateTimeParseException) {
        throw XmlException("时间 \"$text\" 不是合法的 UTC ISO-8601 时刻")
    }

    fun now(): String = Instant.now().toString()
}
