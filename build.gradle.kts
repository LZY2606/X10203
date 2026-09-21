plugins {
    kotlin("jvm") version "2.2.20"
    kotlin("plugin.serialization") version "2.2.20"
    application
}

group = "com.example.jointbook"
version = "1.0.0"

repositories {
    mavenCentral()
}

application {
    mainClass.set("com.example.jointbook.MainKt")
}

kotlin {
    jvmToolchain(17)
}

dependencies {
    implementation("io.ktor:ktor-server-core:3.1.3")
    implementation("io.ktor:ktor-server-netty:3.1.3")
    implementation("io.ktor:ktor-server-content-negotiation:3.1.3")
    implementation("io.ktor:ktor-serialization-kotlinx-json:3.1.3")
    implementation("org.xerial:sqlite-jdbc:3.48.0.0")
    implementation("org.slf4j:slf4j-simple:2.0.17")

    testImplementation(kotlin("test"))
    testImplementation("io.ktor:ktor-server-test-host:3.1.3")
}

tasks.test {
    useJUnitPlatform()
}
