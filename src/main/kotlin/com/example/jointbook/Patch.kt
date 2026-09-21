package com.example.jointbook

import kotlinx.serialization.Serializable

enum class PatchStatus { DRAFT, APPROVED, REJECTED }

@Serializable
data class XmlPatch(
    val id: String,
    val robotId: String,
    val baseVersion: Int,
    val status: PatchStatus = PatchStatus.DRAFT,
    val xpath: String,
    val attributes: Map<String, String> = emptyMap(),
    val text: String? = null,
    val reason: String = ""
)
