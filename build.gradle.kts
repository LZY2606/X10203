plugins {
    kotlin("jvm") version "2.2.21"
    kotlin("plugin.serialization") version "2.2.21"
    application
}

group = "coordbook"
version = "1.0.0"

repositories {
    mavenCentral()
}

dependencies {
    implementation("io.ktor:ktor-server-core-jvm:3.6.0")
    implementation("io.ktor:ktor-server-netty-jvm:3.6.0")
    implementation("io.ktor:ktor-server-content-negotiation-jvm:3.6.0")
    implementation("io.ktor:ktor-serialization-kotlinx-json-jvm:3.6.0")
    implementation("io.ktor:ktor-server-status-pages-jvm:3.6.0")
    implementation("org.xerial:sqlite-jdbc:3.53.4.0")
    implementation("ch.qos.logback:logback-classic:1.6.3")

    testImplementation(kotlin("test"))
    testImplementation("io.ktor:ktor-server-test-host-jvm:3.6.0")
    testImplementation("io.ktor:ktor-client-content-negotiation-jvm:3.6.0")
}

application {
    mainClass.set("coordbook.MainKt")
}

tasks.test {
    useJUnitPlatform()
    testLogging {
        events("passed", "skipped", "failed")
    }
}

kotlin {
    jvmToolchain(17)
}
